#!/bin/bash

# Unitree LiDAR Docker Compose Startup Script
# This script starts the Unitree LiDAR ROS2 node with Docker Compose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "=========================================="
echo "Unitree LiDAR ROS2 Docker Startup"
echo "=========================================="

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "Error: Docker is not running. Please start Docker first."
    exit 1
fi

# Check if Docker Compose is available
if ! command -v docker-compose > /dev/null 2>&1 && ! docker compose version > /dev/null 2>&1; then
    echo "Error: Docker Compose is not available."
    exit 1
fi

# Function to detect serial devices
detect_serial_device() {
    echo "Detecting serial devices..."
    
    # Check for common Unitree LiDAR devices
    for device in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1 /dev/unitree_lidar; do
        if [ -e "$device" ]; then
            echo "Found serial device: $device"
            export SERIAL_DEVICE="$device"
            return 0
        fi
    done
    
    echo "Warning: No common serial devices found."
    echo "Available serial devices:"
    ls -la /dev/tty* 2>/dev/null | grep -E "(ACM|USB)" || echo "No ACM or USB devices found"
    
    # Default to /dev/ttyACM0
    export SERIAL_DEVICE="/dev/ttyACM0"
    echo "Using default device: $SERIAL_DEVICE"
}

# Function to set up X11 forwarding for GUI applications
setup_x11() {
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding for RViz..."
        xhost +local:docker > /dev/null 2>&1 || echo "Warning: Could not set X11 permissions"
    else
        echo "Warning: DISPLAY not set. RViz may not work."
    fi
}

# Parse command line arguments
PROFILE=""
BUILD=false
DETACH=false
BRIDGES=true

while [[ $# -gt 0 ]]; do
    case $1 in
        --rviz-only)
            PROFILE="rviz-only"
            shift
            ;;
        --build)
            BUILD=true
            shift
            ;;
        --detach|-d)
            DETACH=true
            shift
            ;;
        --no-bridges)
            BRIDGES=false
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --rviz-only    Start only RViz (requires main service to be running)"
            echo "  --build        Force rebuild of Docker images"
            echo "  --detach, -d   Run in detached mode"
            echo "  --no-bridges   Skip starting Foxglove Bridge and ROS Bridge"
            echo "  --help, -h     Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                    # Start LiDAR node with bridges and RViz"
            echo "  $0 --build           # Rebuild and start all services"
            echo "  $0 --detach          # Start in background"
            echo "  $0 --no-bridges      # Start only LiDAR node (no network bridges)"
            echo "  $0 --rviz-only       # Start only RViz"
            echo ""
            echo "Network Access:"
            echo "  Foxglove Bridge: ws://[HOST_IP]:8765"
            echo "  ROS Bridge:      ws://[HOST_IP]:9090"
            echo "  Replace [HOST_IP] with the actual IP address of this machine"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Detect serial device
detect_serial_device

# Set up X11
setup_x11

# Prepare Docker Compose command - prefer newer docker compose
COMPOSE_CMD="docker compose"
if ! docker compose version > /dev/null 2>&1; then
    if command -v docker-compose > /dev/null 2>&1; then
        COMPOSE_CMD="docker-compose"
    else
        echo "Error: Neither 'docker compose' nor 'docker-compose' is available."
        exit 1
    fi
fi

COMPOSE_ARGS=""
if [ -n "$PROFILE" ]; then
    COMPOSE_ARGS="--profile $PROFILE"
fi

if [ "$BUILD" = true ]; then
    echo "Building Docker images..."
    $COMPOSE_CMD $COMPOSE_ARGS build
fi

# Determine which services to start
SERVICES=""
if [ "$BRIDGES" = false ]; then
    SERVICES="unitree_lidar"
    echo "Starting Unitree LiDAR service only (no bridges)..."
else
    echo "Starting Unitree LiDAR services with network bridges..."
    echo ""
    echo "Network endpoints will be available at:"
    echo "  Foxglove Bridge: ws://$(hostname -I | awk '{print $1}'):8765"
    echo "  ROS Bridge:      ws://$(hostname -I | awk '{print $1}'):9090"
fi

echo "Serial device: $SERIAL_DEVICE"
echo ""

if [ "$DETACH" = true ]; then
    $COMPOSE_CMD $COMPOSE_ARGS up -d $SERVICES
    echo ""
    echo "Services started in detached mode."
    echo "To view logs, run: $COMPOSE_CMD logs -f"
    echo "To stop services, run: $COMPOSE_CMD down"
    if [ "$BRIDGES" = true ]; then
        echo ""
        echo "Network endpoints:"
        echo "  Foxglove Bridge: ws://$(hostname -I | awk '{print $1}'):8765"
        echo "  ROS Bridge:      ws://$(hostname -I | awk '{print $1}'):9090"
    fi
else
    echo "Press Ctrl+C to stop the services"
    echo ""
    $COMPOSE_CMD $COMPOSE_ARGS up $SERVICES
fi