#!/bin/bash
echo "Setting permissions on /dev/ttyACM0..."
sudo chmod 777 /dev/ttyACM0
sudo nmcli con modify "ARCUS-5Ghz" 802-11-wireless.powersave "2"

echo "Launching packages..."
ros2 launch arcus_bringup arcus_bringup.launch.py
