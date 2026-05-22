#!/bin/bash
cd "/mnt/c/Users/Hello/OneDrive/Documents/Robotics new project/aws-robomaker-hospital-world"
export GAZEBO_MODEL_PATH="$(pwd)/models:$(pwd)/fuel_models"
gazebo --verbose worlds/hospital.world
