#!/bin/bash

# Unitree LiDAR ROS2 Bag Recording Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Default settings
OUTPUT_DIR="/workspace/bags"
DURATION=""
TOPICS="/unilidar/cloud /unilidar/imu /tf /tf_static"
BAG_NAME=""
COMPRESSION=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --output|-o)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --duration|-d)
            DURATION="$2"
            shift 2
            ;;
        --name|-n)
            BAG_NAME="$2"
            shift 2
            ;;
        --topics|-t)
            TOPICS="$2"
            shift 2
            ;;
        --compress|-c)
            COMPRESSION="--compression-mode file --compression-format zstd"
            shift
            ;;
        --cloud-only)
            TOPICS="/unilidar/cloud /tf /tf_static"
            shift
            ;;
        --imu-only)
            TOPICS="/unilidar/imu /tf /tf_static"
            shift
            ;;
        --all|-a)
            TOPICS="-a"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --output, -o DIR      Output directory (default: /workspace/bags -> ./bags)"
            echo "  --duration, -d SEC    Recording duration in seconds"
            echo "  --name, -n NAME       Bag file name (default: auto-generated)"
            echo "  --topics, -t TOPICS   Topics to record (default: cloud, imu, tf)"
            echo "  --compress, -c        Enable compression (zstd)"
            echo "  --cloud-only          Record only point cloud and TF"
            echo "  --imu-only            Record only IMU and TF"
            echo "  --all, -a             Record all topics"
            echo "  --help, -h            Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Record cloud, imu, tf for unlimited time"
            echo "  $0 --duration 60                     # Record for 60 seconds"
            echo "  $0 --cloud-only --duration 30        # Record only cloud data for 30 seconds"
            echo "  $0 --compress --name test_run         # Record with compression"
            echo "  $0 --output /data/bags                # Save to custom directory"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if LiDAR service is running
if ! docker compose ps unitree_lidar 2>/dev/null | grep -q "Up"; then
    echo "Error: Unitree LiDAR service is not running."
    echo "Please start it first with: ./scripts/start-lidar.sh --detach"
    exit 1
fi

# Create output directory (will be created inside Docker container)
# mkdir -p "$OUTPUT_DIR" # Done inside container

# Generate bag name if not provided
if [ -z "$BAG_NAME" ]; then
    BAG_NAME="unitree_lidar_$(date +%Y%m%d_%H%M%S)"
fi

BAG_PATH="$OUTPUT_DIR/$BAG_NAME"

# Build ros2 bag command
ROS2_BAG_CMD="ros2 bag record"

if [ -n "$DURATION" ]; then
    ROS2_BAG_CMD="$ROS2_BAG_CMD --max-bag-duration $DURATION"
fi

if [ -n "$COMPRESSION" ]; then
    ROS2_BAG_CMD="$ROS2_BAG_CMD $COMPRESSION"
fi

ROS2_BAG_CMD="$ROS2_BAG_CMD -o $BAG_PATH $TOPICS"

echo "=========================================="
echo "Unitree LiDAR ROS2 Bag Recording"
echo "=========================================="
echo "Output path: $BAG_PATH"
echo "Topics: $TOPICS"
if [ -n "$DURATION" ]; then
    echo "Duration: ${DURATION}s"
else
    echo "Duration: Unlimited (press Ctrl+C to stop)"
fi
if [ -n "$COMPRESSION" ]; then
    echo "Compression: Enabled (zstd)"
fi
echo ""
echo "Starting recording..."
echo ""

# Execute recording inside Docker container
docker compose exec unitree_lidar bash -c "
    source /opt/ros/humble/setup.bash && 
    mkdir -p $OUTPUT_DIR &&
    cd /workspace && 
    $ROS2_BAG_CMD
"

echo ""
echo "Recording completed!"
echo "Bag file saved in container: $BAG_PATH"
echo ""
echo "Copying to local directory..."
LOCAL_BAG_PATH="$PROJECT_ROOT/bags/$BAG_NAME"

# Copy bag to local directory using tar (more reliable)
echo "Copying to local directory..."
mkdir -p "$PROJECT_ROOT/bags"
docker compose exec unitree_lidar bash -c "cd $OUTPUT_DIR && tar -cf - $BAG_NAME" | tar -xf - -C "$PROJECT_ROOT/bags/" 2>/dev/null || {
    echo "Warning: Auto-copy failed. Use './scripts/copy-bags.sh --bag $BAG_NAME' to copy manually."
    LOCAL_BAG_PATH="$BAG_PATH (in container only)"
}

if [ -d "$PROJECT_ROOT/bags/$BAG_NAME" ]; then
    LOCAL_SIZE=$(du -sh "$PROJECT_ROOT/bags/$BAG_NAME" | cut -f1)
    LOCAL_FILES=$(find "$PROJECT_ROOT/bags/$BAG_NAME" -name '*.db3' | wc -l)
    echo "Local copy: $LOCAL_BAG_PATH ($LOCAL_SIZE, $LOCAL_FILES files)"
else
    echo "Container location: $BAG_PATH"
fi
echo ""
echo "To replay the bag:"
echo "  ros2 bag play $BAG_PATH"
echo ""
echo "To inspect the bag:"
echo "  ros2 bag info $BAG_PATH"