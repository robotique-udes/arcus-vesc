#!/bin/bash
echo "Setting permissions on /dev/ttyACM0..."
sudo chmod 777 /dev/ttyACM0


echo "Launching packages..."
ros2 launch arcus_bringup arcus_bringup.launch.py
