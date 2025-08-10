#!/bin/bash

# Unitree LiDAR Topic Monitoring Script
# This script helps monitor ROS2 topics from the LiDAR

CONTAINER_NAME="unitree_lidar_ros2"

echo "=========================================="
echo "Unitree LiDAR ROS2 Topic Monitor"
echo "=========================================="

if ! docker ps | grep -q $CONTAINER_NAME; then
    echo "Error: Container $CONTAINER_NAME is not running."
    echo "Start the LiDAR service first with: ./scripts/start-lidar.sh"
    exit 1
fi

function run_ros_command() {
    docker exec $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && $*"
}

case "${1:-help}" in
    list)
        echo "Available ROS2 topics:"
        run_ros_command "ros2 topic list"
        ;;
    cloud)
        echo "Point cloud data (showing first message):"
        run_ros_command "ros2 topic echo /unilidar/cloud --once"
        ;;
    imu)
        echo "IMU data (showing first message):"
        run_ros_command "ros2 topic echo /unilidar/imu --once"
        ;;
    hz)
        echo "Topic frequencies:"
        echo "Point cloud frequency:"
        run_ros_command "timeout 5 ros2 topic hz /unilidar/cloud" || true
        echo ""
        echo "IMU frequency:"
        run_ros_command "timeout 5 ros2 topic hz /unilidar/imu" || true
        ;;
    info)
        echo "Topic information:"
        echo ""
        echo "=== Point Cloud Topic ==="
        run_ros_command "ros2 topic info /unilidar/cloud"
        echo ""
        echo "=== IMU Topic ==="
        run_ros_command "ros2 topic info /unilidar/imu"
        ;;
    tf)
        echo "Available TF frames:"
        run_ros_command "ros2 run tf2_tools view_frames"
        ;;
    monitor)
        echo "Monitoring all topics (Press Ctrl+C to stop):"
        echo "Point cloud points and IMU data will be displayed..."
        while true; do
            echo "--- $(date) ---"
            echo "Point cloud points: $(run_ros_command "ros2 topic echo /unilidar/cloud --once" | grep -E 'width:' | awk '{print $2}')"
            echo "IMU acceleration Z: $(run_ros_command "ros2 topic echo /unilidar/imu --once" | grep -A1 'linear_acceleration:' | tail -1 | awk '{print $2}')"
            sleep 2
        done
        ;;
    help|*)
        echo "Usage: $0 [COMMAND]"
        echo ""
        echo "Commands:"
        echo "  list     Show all available ROS2 topics"
        echo "  cloud    Show one point cloud message"
        echo "  imu      Show one IMU message"
        echo "  hz       Show topic publishing frequencies"
        echo "  info     Show detailed topic information"
        echo "  tf       Show TF frame tree"
        echo "  monitor  Continuously monitor data (Ctrl+C to stop)"
        echo "  help     Show this help message"
        echo ""
        echo "Examples:"
        echo "  $0 list          # List all topics"
        echo "  $0 cloud         # Show point cloud data"
        echo "  $0 hz            # Check publication rates"
        echo "  $0 monitor       # Continuous monitoring"
        ;;
esac