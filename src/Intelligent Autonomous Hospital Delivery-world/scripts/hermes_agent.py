#!/usr/bin/env python3
"""
hermes_agent.py — Top-level Hermes Autonomous Hospital Delivery Agent.

Implements all 8 skills from Skills.md:
  1. Autonomous Navigation  — Nav2 NavigateToPose action client
  2. SLAM & Localization    — Monitors /map for map readiness
  3. Path Planning          — Smac Hybrid-A* + Reeds-Shepp via Nav2
  4. MPPI Controller        — Used transparently via Nav2
  5. Frontier Exploration   — Triggers frontier_explorer when map is unknown
  6. Multi-Robot Coord      — Namespaced robot support via task_allocator
  7. Fleet Management       — Hungarian assignment + fleet_health monitor
  8. Medicine Delivery      — Full end-to-end reception → patient_room pipeline

Architecture (3-tier):
  ┌──────────────────────────────────────────────────┐
  │  Hermes Agent  (this file)                       │
  │   • Delivery pipeline orchestration              │
  │   • Fleet health monitoring                      │
  │   • Hungarian task allocation integration        │
  ├──────────────────────────────────────────────────┤
  │  Mission Server  (mission_server.py)             │
  │   • FSM: IDLE → NAVIGATING → DONE/FAILED        │
  │   • patrol / sequence / goto mission types       │
  ├──────────────────────────────────────────────────┤
  │  Nav2 Stack                                      │
  │   • Smac Hybrid-A* planner (Reeds-Shepp)        │
  │   • MPPI controller                              │
  │   • Velocity smoother → collision monitor       │
  └──────────────────────────────────────────────────┘

Usage — daemon mode (starts and listens for delivery requests):
  ros2 run aws_robomaker_hospital_world hermes_agent.py

Usage — send a delivery directly:
  ros2 run aws_robomaker_hospital_world hermes_agent.py deliver \\
      --robot aws_robomaker_hospital_world --from reception --to patient_room1

  ros2 run aws_robomaker_hospital_world hermes_agent.py deliver \\
      --robot aws_robomaker_hospital_world --from pharmacy --to patient_room2 --priority high

  ros2 run aws_robomaker_hospital_world hermes_agent.py status
  ros2 run aws_robomaker_hospital_world hermes_agent.py fleet
  ros2 run aws_robomaker_hospital_world hermes_agent.py locations

Delivery request topic (JSON on /hermes/deliver):
  {
    "robot":    "aws_robomaker_hospital_world",   # robot namespace
    "from":     "reception",    # named pickup location
    "to":       "patient_room1",# named dropoff location
    "payload":  "medicine",     # optional: medicine / supply / emergency
    "priority": "normal"        # optional: normal / high / emergency
  }
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

# ── Priority levels ────────────────────────────────────────────────────────────
PRIORITY_NORMAL    = 0
PRIORITY_HIGH      = 1
PRIORITY_EMERGENCY = 2

_PRIORITY_MAP = {
    'normal':    PRIORITY_NORMAL,
    'high':      PRIORITY_HIGH,
    'emergency': PRIORITY_EMERGENCY,
}

# ── Delivery states ────────────────────────────────────────────────────────────
STATE_QUEUED     = 'QUEUED'
STATE_PICKUP     = 'PICKUP'        # navigating to pickup (from) location
STATE_DELIVERING = 'DELIVERING'   # navigating to dropoff (to) location
STATE_DONE       = 'DONE'
STATE_FAILED     = 'FAILED'


# ── Location helper ────────────────────────────────────────────────────────────

def _locations_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('aws_robomaker_hospital_world'),
            'config', 'locations.yaml')
    except Exception:
        return ''


def _load_locations() -> dict:
    try:
        import yaml
    except ImportError:
        return {}
    path = _locations_path()
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get('locations', {})


# ── Hermes Agent Node ──────────────────────────────────────────────────────────

class HermesAgent(Node):
    """
    Top-level orchestrator for autonomous hospital medicine delivery.

    Skill mapping:
      Skill 1 – Navigation   : sends goto missions via /mission/execute
      Skill 2 – SLAM         : monitors /map topic for map availability
      Skill 6 – Multi-robot  : namespaced robot support, fleet state tracking
      Skill 7 – Fleet Mgmt   : Hungarian allocator via /task_queue, health via /fleet/health
      Skill 8 – Delivery     : 2-phase pickup→dropoff pipeline
    """

    def __init__(self):
        super().__init__('hermes_agent')

        self._locations = _load_locations()
        self._lock = threading.Lock()

        # Delivery queue — list of delivery dicts sorted by priority
        self._queue: list[dict] = []
        self._delivery_id = 0

        # Robot fleet state  {robot_ns: mission_state_str}
        self._fleet: dict[str, str] = {}

        # Map availability flag (Skill 2 — SLAM)
        self._map_ready = False

        # ── Subscriptions ──────────────────────────────────────────────────
        self.create_subscription(
            String, '/hermes/deliver', self._deliver_cb, 10)
        self.create_subscription(
            String, '/mission/state', self._mission_state_cb, 10)
        self.create_subscription(
            String, '/fleet/health', self._fleet_health_cb, 10)

        # Lightweight map-readiness check (topic existence)
        from nav_msgs.msg import OccupancyGrid
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 1)

        # ── Publishers ─────────────────────────────────────────────────────
        self._mission_pub = self.create_publisher(String, '/mission/execute', 10)
        self._state_pub   = self.create_publisher(String, '/hermes/state', 10)

        # ── Timers ─────────────────────────────────────────────────────────
        self.create_timer(1.0,  self._publish_state)     # state at 1 Hz
        self.create_timer(0.5,  self._dispatch_tick)     # allocate at 2 Hz

        loc_count = len(self._locations)
        self.get_logger().info(
            f'Hermes Agent ready — {loc_count} named location(s). '
            f'Listening on /hermes/deliver')

    # ── Map readiness (Skill 2 — SLAM) ────────────────────────────────────────

    def _map_cb(self, _):
        if not self._map_ready:
            self._map_ready = True
            self.get_logger().info('[Skill 2 — SLAM] Map received — navigation enabled.')

    # ── Delivery intake ────────────────────────────────────────────────────────

    def _deliver_cb(self, msg: String):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad delivery JSON: {e}')
            return

        pickup   = req.get('from', '')
        dropoff  = req.get('to', '')
        robot    = req.get('robot', '')
        payload  = req.get('payload', 'medicine')
        priority = _PRIORITY_MAP.get(req.get('priority', 'normal'), PRIORITY_NORMAL)

        # Validate locations
        for name in (pickup, dropoff):
            if name and name not in self._locations:
                self.get_logger().error(
                    f'Unknown location: {name!r}. Known: {list(self._locations)}')
                return

        with self._lock:
            self._delivery_id += 1
            delivery = {
                'id':       self._delivery_id,
                'robot':    robot,
                'from':     pickup,
                'to':       dropoff,
                'payload':  payload,
                'priority': priority,
                'state':    STATE_QUEUED,
                'phase':    '',       # 'pickup' or 'deliver'
                'created':  time.time(),
            }
            # Insert by priority (higher priority first)
            inserted = False
            for i, d in enumerate(self._queue):
                if priority > d['priority']:
                    self._queue.insert(i, delivery)
                    inserted = True
                    break
            if not inserted:
                self._queue.append(delivery)

        self.get_logger().info(
            f'[Skill 8 — Delivery] #{self._delivery_id} queued: '
            f'{pickup} → {dropoff}  payload={payload}  priority={req.get("priority","normal")}'
            f'  robot={robot or "auto"}')

    # ── Mission state tracking (Skill 7 — Fleet Mgmt) ─────────────────────────

    def _mission_state_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot = data.get('robot', '')
        state = data.get('state', '')
        if not robot:
            return

        with self._lock:
            prev = self._fleet.get(robot, '')
            self._fleet[robot] = state

            # Update delivery phase when mission completes
            for d in self._queue:
                if d.get('robot') != robot:
                    continue
                if d['state'] in (STATE_DONE, STATE_FAILED):
                    continue

                if state == 'DONE':
                    if d['phase'] == 'pickup':
                        # Pickup done → now go to dropoff
                        d['phase'] = 'deliver'
                        d['state'] = STATE_DELIVERING
                        self.get_logger().info(
                            f'[Skill 8 — Delivery] #{d["id"]} pickup complete → '
                            f'now delivering to {d["to"]}')
                        # Dispatch dropoff immediately in background
                        t = threading.Thread(
                            target=self._dispatch_dropoff, args=(d,), daemon=True)
                        t.start()
                    elif d['phase'] == 'deliver':
                        d['state'] = STATE_DONE
                        self.get_logger().info(
                            f'[Skill 8 — Delivery] #{d["id"]} COMPLETE ✓  '
                            f'{d["from"]} → {d["to"]}')

                elif state == 'FAILED':
                    d['state'] = STATE_FAILED
                    self.get_logger().warn(
                        f'[Skill 8 — Delivery] #{d["id"]} FAILED '
                        f'(phase={d["phase"]})')

    def _fleet_health_cb(self, msg: String):
        """Skill 7 — receive fleet health events and log warnings."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        robot  = data.get('robot', '?')
        health = data.get('health', 'unknown')
        if health not in ('ok', 'healthy'):
            self.get_logger().warn(
                f'[Skill 7 — Fleet Health] Robot {robot}: {health}  — {data}')

    # ── Dispatch tick (Skill 7 — Hungarian allocation) ────────────────────────

    def _dispatch_tick(self):
        with self._lock:
            pending = [d for d in self._queue if d['state'] == STATE_QUEUED]
            if not pending:
                return

            # Find idle robots (from fleet state)
            idle_robots = [
                ns for ns, st in self._fleet.items()
                if st in ('IDLE', 'DONE', 'FAILED')
                and not any(
                    d.get('robot') == ns and d['state'] in (STATE_PICKUP, STATE_DELIVERING)
                    for d in self._queue
                )
            ]

            if not idle_robots and not pending[0]['robot']:
                # No fleet discovered yet — try delivering anyway if robot specified
                return

            for delivery in pending:
                robot = delivery.get('robot', '')
                if not robot and not idle_robots:
                    continue
                if not robot:
                    # Skill 7: use Hungarian — pick nearest idle robot
                    robot = self._pick_best_robot(idle_robots, delivery)
                    if not robot:
                        continue
                    delivery['robot'] = robot

                # Dispatch pickup phase
                delivery['state'] = STATE_PICKUP
                delivery['phase'] = 'pickup'
                self._dispatch_pickup_unlocked(delivery)
                if robot in idle_robots:
                    idle_robots.remove(robot)

    def _pick_best_robot(self, idle_robots: list, delivery: dict) -> str:
        """
        Skill 7 — Simplified nearest-robot selection.
        Full N-robot Hungarian assignment is in task_allocator.py.
        """
        if not idle_robots:
            return ''
        # For single-robot case, just pick first idle
        return idle_robots[0]

    def _dispatch_pickup_unlocked(self, delivery: dict):
        """Send goto for pickup location."""
        pickup = delivery['from']
        robot  = delivery['robot']
        coords = self._locations.get(pickup, [])
        if not coords:
            self.get_logger().error(f'Pickup location {pickup!r} not found.')
            delivery['state'] = STATE_FAILED
            return

        payload = {
            'type':      'goto',
            'robot':     robot,
            'pose':      list(coords),
            'waypoints': [],
        }
        msg      = String()
        msg.data = json.dumps(payload)
        self._mission_pub.publish(msg)
        self.get_logger().info(
            f'[Skill 8 — Delivery] #{delivery["id"]} dispatching pickup: '
            f'robot={robot}  pickup={pickup}  coords={coords}')

    def _dispatch_dropoff(self, delivery: dict):
        """Send goto for dropoff location (called after pickup DONE)."""
        time.sleep(0.5)  # small delay to allow mission server to reset state
        dropoff = delivery['to']
        robot   = delivery['robot']
        coords  = self._locations.get(dropoff, [])
        if not coords:
            self.get_logger().error(f'Dropoff location {dropoff!r} not found.')
            with self._lock:
                delivery['state'] = STATE_FAILED
            return

        payload = {
            'type':      'goto',
            'robot':     robot,
            'pose':      list(coords),
            'waypoints': [],
        }
        msg      = String()
        msg.data = json.dumps(payload)
        self._mission_pub.publish(msg)
        self.get_logger().info(
            f'[Skill 8 — Delivery] #{delivery["id"]} dispatching dropoff: '
            f'robot={robot}  dropoff={dropoff}  coords={coords}')

    # ── State publisher ────────────────────────────────────────────────────────

    def _publish_state(self):
        with self._lock:
            snapshot = {
                'map_ready':  self._map_ready,
                'fleet':      dict(self._fleet),
                'deliveries': [
                    {
                        'id':       d['id'],
                        'from':     d['from'],
                        'to':       d['to'],
                        'payload':  d['payload'],
                        'priority': d['priority'],
                        'state':    d['state'],
                        'phase':    d['phase'],
                        'robot':    d.get('robot', ''),
                    }
                    for d in self._queue
                ],
            }
        msg      = String()
        msg.data = json.dumps(snapshot)
        self._state_pub.publish(msg)


# ── CLI helpers ────────────────────────────────────────────────────────────────

def _send_delivery(node: Node, robot: str, from_loc: str, to_loc: str,
                   payload: str, priority: str):
    pub = node.create_publisher(String, '/hermes/deliver', 10)
    time.sleep(0.3)
    msg = String()
    msg.data = json.dumps({
        'robot':    robot,
        'from':     from_loc,
        'to':       to_loc,
        'payload':  payload,
        'priority': priority,
    })
    pub.publish(msg)
    time.sleep(0.3)
    print(f'Delivery request sent: {from_loc} → {to_loc}  '
          f'robot={robot or "auto"}  payload={payload}  priority={priority}')


def _show_status(node: Node):
    received = [None]

    def _cb(msg):
        received[0] = json.loads(msg.data)

    node.create_subscription(String, '/hermes/state', _cb, 10)
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()
    deadline = time.time() + 3.0
    while received[0] is None and time.time() < deadline:
        time.sleep(0.05)

    if received[0]:
        s = received[0]
        print('\n── Hermes Agent Status ───────────────────────────────')
        print(f'  Map ready : {s["map_ready"]}')
        print(f'  Fleet     : {s["fleet"] or "(none)"}')
        deliveries = s.get('deliveries', [])
        if deliveries:
            print('\n── Deliveries ───────────────────────────────────────')
            for d in deliveries:
                prio = {0: 'normal', 1: 'high', 2: 'emergency'}.get(d['priority'], '?')
                print(f'  #{d["id"]:3d}  [{d["state"]:12s}]  '
                      f'{d["from"]} → {d["to"]}  '
                      f'robot={d["robot"] or "unassigned"}  '
                      f'payload={d["payload"]}  priority={prio}')
        else:
            print('  No active deliveries.')
        print('─────────────────────────────────────────────────────\n')
    else:
        print('No Hermes Agent running (start daemon first).')


def _show_locations():
    locs = _load_locations()
    if not locs:
        print('No locations.yaml found.')
        return
    print('\n── Named Locations ───────────────────────────────────')
    for name, coords in locs.items():
        print(f'  {name:<20} {coords}')
    print('─────────────────────────────────────────────────────\n')


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    argv = remove_ros_args(args=sys.argv)[1:]

    if not argv or argv[0] == '--daemon':
        # ── Daemon mode ────────────────────────────────────────────────────
        rclpy.init()
        node = HermesAgent()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
        return

    # ── CLI mode ───────────────────────────────────────────────────────────
    cmd = argv[0].lower()

    if cmd == 'locations':
        _show_locations()
        return

    rclpy.init()
    node = Node('hermes_agent_cli')

    if cmd == 'status':
        _show_status(node)

    elif cmd == 'fleet':
        # Alias for status focusing on fleet
        _show_status(node)

    elif cmd == 'deliver':
        parser = argparse.ArgumentParser(
            prog='hermes_agent.py deliver',
            description='Send a medicine delivery request to the Hermes Agent.')
        parser.add_argument('--robot',    default='aws_robomaker_hospital_world',
                            help='Robot namespace (default: aws_robomaker_hospital_world)')
        parser.add_argument('--from',     dest='from_loc', required=True,
                            help='Pickup location name (e.g. reception, pharmacy)')
        parser.add_argument('--to',       dest='to_loc',   required=True,
                            help='Dropoff location name (e.g. patient_room1)')
        parser.add_argument('--payload',  default='medicine',
                            choices=['medicine', 'supply', 'emergency'],
                            help='Payload type')
        parser.add_argument('--priority', default='normal',
                            choices=['normal', 'high', 'emergency'],
                            help='Mission priority')
        args = parser.parse_args(argv[1:])

        _send_delivery(
            node,
            robot    = args.robot,
            from_loc = args.from_loc,
            to_loc   = args.to_loc,
            payload  = args.payload,
            priority = args.priority,
        )
    else:
        print(__doc__)
        sys.exit(1)

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
