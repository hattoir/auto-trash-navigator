#!/bin/bash
echo "🧹 Starting complete environment cleanup..."
pkill -9 -f "parameter_bridge" || true
pkill -9 -f "gz" || true
pkill -9 -f "ros2" || true
pkill -9 -f "ruby" || true
pkill -9 -f "rviz" || true
pkill -9 -f "robot_state_publisher" || true
pkill -9 -f "python" || true
pkill -9 -f "image_synchronizer" || true
pkill -9 -f "patrol_and_collect" || true
pkill -9 -f "trash_detector" || true
pkill -9 -u $USER -f "Xvfb" || true
rm -f /tmp/.X101-lock || true
rm -f /tmp/.X98-lock || true
rm -f /tmp/.X99-lock || true

ros2 daemon stop || true
rm -f /dev/shm/fastrtps_*

echo "⏳ Waiting 12 seconds for FastDDS network discovery cache to expire..."
sleep 12

ros2 daemon start || true
echo "✅ Cleanup complete. Active nodes list:"
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pakku/auto-trash-navigator/fastdds_udp_only.xml
ros2 node list
