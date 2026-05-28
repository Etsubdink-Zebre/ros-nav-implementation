#!/usr/bin/env python3
"""Send delivery tasks to all hospital locations"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

rclpy.init()
node = Node("batch_delivery")

pub = node.create_publisher(String, "/delivery_request", 10)
print(f"Subs: {pub.get_subscription_count()}")
for i in range(20):
    if pub.get_subscription_count() > 0:
        break
    time.sleep(0.3)
    print(f"  waiting... subs: {pub.get_subscription_count()}")

tasks = [
    {"origin": "pharmacy",      "destination": "patient_room1", "payload": "morphine",     "priority": "STAT"},
    {"origin": "pharmacy",      "destination": "patient_room2", "payload": "antibiotics",  "priority": "URGENT"},
    {"origin": "supply_room",   "destination": "nurse_station", "payload": "bandages",     "priority": "ROUTINE"},
    {"origin": "pharmacy",      "destination": "patient_room3", "payload": "IV_drip",      "priority": "URGENT"},
    {"origin": "lab",           "destination": "nurse_station", "payload": "blood_sample", "priority": "ROUTINE"},
    {"origin": "reception",     "destination": "patient_room4", "payload": "visitor_pass", "priority": "ROUTINE"},
    {"origin": "supply_room",   "destination": "patient_room5", "payload": "linens",       "priority": "URGENT"},
    {"origin": "pharmacy",      "destination": "reception",     "payload": "prescription", "priority": "ROUTINE"},
]

for task in tasks:
    msg = String()
    msg.data = json.dumps(task)
    pub.publish(msg)
    print(f"  -> {task['origin']} -> {task['destination']} [{task['priority']}] {task['payload']}")
    time.sleep(0.4)

print(f"\nSent {len(tasks)} delivery tasks!")
node.destroy_node()
rclpy.shutdown()