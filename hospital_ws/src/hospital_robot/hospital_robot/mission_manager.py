#!/usr/bin/env python3
"""
Hospital Delivery Mission Manager
===================================
Manages the full medicine/supply delivery workflow:
  1. Accept delivery requests (topic + direct API)
  2. Priority queue: STAT(0) > URGENT(1) > ROUTINE(2)
  3. Execute via Nav2 NavigateToPose action
  4. Log results and publish status

Hospital locations match AWS RoboMaker hospital world coordinates.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import json
import math
import time
import heapq
import threading
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


# =============================================================================
# HOSPITAL LOCATIONS
# Coordinates from AWS RoboMaker hospital world (hospital.world)
# =============================================================================
HOSPITAL_LOCATIONS = {
    "reception":        ( 0.0,  -10.0,  0.0),
    "pharmacy":         ( 5.0,    8.0,  1.57),
    "supply_room":      (-5.0,    8.0, -1.57),
    "patient_room1":    ( 9.0,   -1.0,  1.57),
    "patient_room2":    ( 9.0,   -4.0,  1.57),
    "patient_room3":    ( 9.0,   -7.0,  1.57),
    "patient_room4":    (-9.0,   -1.0, -1.57),
    "patient_room5":    (-9.0,   -4.0, -1.57),
    "nurse_station":    ( 0.0,    0.0,  0.0),
    "lab":              (-5.0,   -8.0,  0.0),
    "home":             ( 0.0,    0.0,  0.0),
}


class Priority(IntEnum):
    STAT    = 0   # Emergency: blood products, resus drugs
    URGENT  = 1   # Time-sensitive: scheduled meds, surgical kits
    ROUTINE = 2   # Standard: lab samples, supplies


class TaskState:
    PENDING    = "PENDING"
    NAVIGATING = "NAVIGATING"
    DELIVERED  = "DELIVERED"
    FAILED     = "FAILED"
    CANCELLED  = "CANCELLED"


@dataclass(order=True)
class DeliveryTask:
    priority:    int
    task_id:     str    = field(compare=False)
    origin:      str    = field(compare=False)
    destination: str    = field(compare=False)
    payload:     str    = field(compare=False)
    state:       str    = field(compare=False, default=TaskState.PENDING)
    created_at:  float  = field(compare=False, default_factory=time.time)
    completed_at: Optional[float] = field(compare=False, default=None)


class MissionManager(Node):
    """
    Central delivery task manager for the hospital robot.
    Subscribes to /delivery_request, queues by priority,
    and sends NavigateToPose goals to Nav2.
    """

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('robot_id',   'robot_1')
        self.declare_parameter('robot_ns',   '')
        self.robot_id = self.get_parameter('robot_id').value
        self._ns      = self.get_parameter('robot_ns').value

        prefix = f'/{self._ns}' if self._ns else ''

        # State
        self._queue:     list[DeliveryTask] = []
        self._lock       = threading.Lock()
        self._active:    Optional[DeliveryTask] = None
        self._counter    = 0
        self._completed: list[DeliveryTask] = []

        # Nav2 action client
        self._nav = ActionClient(
            self, NavigateToPose, f'{prefix}/navigate_to_pose'
        )

        # Publishers
        self._status_pub = self.create_publisher(String, f'{prefix}/mission_status', 10)
        self._log_pub    = self.create_publisher(String, f'{prefix}/task_log', 10)

        # Subscriptions
        self.create_subscription(
            String, f'{prefix}/delivery_request',
            self._on_request, 10
        )
        self.create_subscription(
            String, f'{prefix}/cancel_task',
            self._on_cancel, 10
        )

        # Timers
        self.create_timer(1.0, self._tick)
        self.create_timer(5.0, self._publish_status)

        self.get_logger().info(
            f'[{self.robot_id}] Mission Manager ready. '
            f'Locations: {list(HOSPITAL_LOCATIONS.keys())}'
        )

    # ─────────────── Public API ──────────────────────────────────────────────

    def submit(self, origin: str, destination: str,
               payload: str = 'medication',
               priority: Priority = Priority.ROUTINE) -> str:
        """Submit a delivery task. Returns task ID."""
        self._counter += 1
        tid = f'{self.robot_id}_T{self._counter:04d}'
        task = DeliveryTask(
            priority    = int(priority),
            task_id     = tid,
            origin      = origin,
            destination = destination,
            payload     = payload,
        )
        with self._lock:
            heapq.heappush(self._queue, task)
        self._log(tid, 'SUBMITTED', destination)
        self.get_logger().info(
            f'Task {tid}: {origin}->{destination} '
            f'[{Priority(priority).name}] {payload}'
        )
        return tid

    # ─────────────── Callbacks ───────────────────────────────────────────────

    def _on_request(self, msg: String):
        try:
            d    = json.loads(msg.data)
            prio = getattr(Priority, d.get('priority', 'ROUTINE').upper(), Priority.ROUTINE)
            self.submit(
                origin      = d.get('origin',      'pharmacy'),
                destination = d.get('destination', 'nurse_station'),
                payload     = d.get('payload',     'medication'),
                priority    = prio,
            )
        except Exception as e:
            self.get_logger().error(f'Bad request: {e}')

    def _on_cancel(self, msg: String):
        tid = msg.data.strip()
        with self._lock:
            for t in self._queue:
                if t.task_id == tid:
                    t.state = TaskState.CANCELLED
                    self.get_logger().warn(f'Cancelled queued {tid}')

    # ─────────────── Queue processing ────────────────────────────────────────

    def _tick(self):
        """Process queue: pop next task and send to Nav2."""
        if self._active:
            return
        with self._lock:
            while self._queue:
                task = heapq.heappop(self._queue)
                if task.state != TaskState.CANCELLED:
                    break
            else:
                return  # Queue empty

        if task.destination not in HOSPITAL_LOCATIONS:
            self.get_logger().error(f'Unknown destination: {task.destination}')
            task.state = TaskState.FAILED
            self._completed.append(task)
            return

        if not self._nav.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 not ready, requeueing...')
            with self._lock:
                heapq.heappush(self._queue, task)
            return

        self._active = task
        task.state   = TaskState.NAVIGATING
        self._log(task.task_id, 'NAVIGATING', task.destination)

        x, y, yaw = HOSPITAL_LOCATIONS[task.destination]
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)

        future = self._nav.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_feedback(fb)
        )
        future.add_done_callback(
            lambda f, t=task: self._on_goal_response(f, t)
        )

    def _on_goal_response(self, future, task: DeliveryTask):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f'{task.task_id}: goal rejected')
            task.state   = TaskState.FAILED
            self._active = None
            self._completed.append(task)
            return
        gh.get_result_async().add_done_callback(
            lambda f, t=task: self._on_result(f, t)
        )

    def _on_feedback(self, fb):
        dist = fb.feedback.distance_remaining
        if int(dist * 2) % 5 == 0:
            self.get_logger().debug(f'Distance remaining: {dist:.2f}m')

    def _on_result(self, future, task: DeliveryTask):
        result       = future.result()
        task.completed_at = time.time()
        self._active = None

        if result.status == 4:
            task.state = TaskState.DELIVERED
            self.get_logger().info(f'{task.task_id}: DELIVERED to {task.destination}')
            self._log(task.task_id, 'DELIVERED', task.destination)
        else:
            task.state = TaskState.FAILED
            self.get_logger().warn(
                f'{task.task_id}: FAILED (Nav2 status={result.status})'
            )
            self._log(task.task_id, 'FAILED', task.destination)

        self._completed.append(task)

    # ─────────────── Helpers ─────────────────────────────────────────────────

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp    = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2)
        p.pose.orientation.w = math.cos(yaw / 2)
        return p

    def _log(self, tid: str, event: str, location: str):
        msg      = String()
        msg.data = json.dumps({
            'task_id':   tid,
            'event':     event,
            'location':  location,
            'robot_id':  self.robot_id,
            'timestamp': time.time(),
        })
        self._log_pub.publish(msg)

    def _publish_status(self):
        with self._lock:
            queued = len(self._queue)
        msg      = String()
        msg.data = json.dumps({
            'robot_id':   self.robot_id,
            'queued':     queued,
            'active':     self._active.task_id if self._active else None,
            'completed':  len(self._completed),
            'delivered':  sum(1 for t in self._completed if t.state == TaskState.DELIVERED),
            'timestamp':  time.time(),
        })
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    # Demo tasks on startup
    node.get_logger().info('Submitting demo delivery tasks...')
    node.submit('pharmacy',     'patient_room1', 'morphine',    Priority.STAT)
    node.submit('pharmacy',     'patient_room2', 'antibiotics', Priority.URGENT)
    node.submit('supply_room',  'nurse_station', 'bandages',    Priority.ROUTINE)
    node.submit('pharmacy',     'patient_room3', 'IV_drip',     Priority.URGENT)
    node.submit('lab',          'nurse_station', 'blood_sample',Priority.ROUTINE)
    node.submit('reception',    'patient_room4', 'lunch_tray',  Priority.ROUTINE)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
