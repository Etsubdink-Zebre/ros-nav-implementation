#!/bin/bash
echo "Starting Delivery Task: Medicines from Reception (Nurse Station) to Patient Room 1"

# We send the patrol command. The mission server will automatically navigate 
# from the current location -> nurse_station (reception) -> patient_room1
ros2 run diff_drive_robot mission_server.py patrol diff_drive nurse_station patient_room1

echo "Delivery mission dispatched! Run 'ros2 run diff_drive_robot mission_server.py status' to monitor progress."
