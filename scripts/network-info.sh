#!/bin/bash

# Network Information Script for Unitree LiDAR
# Shows network endpoints and connection information

set -e

echo "=========================================="
echo "Unitree LiDAR Network Information"
echo "=========================================="

# Get the primary IP address
HOST_IP=$(hostname -I | awk '{print $1}')

echo "Host Information:"
echo "  Hostname: $(hostname)"
echo "  Primary IP: $HOST_IP"
echo ""

echo "Network Endpoints:"
echo "  Foxglove Bridge: ws://$HOST_IP:8765"
echo "  ROS Bridge:      ws://$HOST_IP:9090"
echo ""

echo "Service Status:"
# Check if containers are running
if docker ps --format "table {{.Names}}\t{{.Status}}" | grep -q "unitree_lidar_ros2"; then
    echo "  ✅ LiDAR Service: Running"
else
    echo "  ❌ LiDAR Service: Not running"
fi

if docker ps --format "table {{.Names}}\t{{.Status}}" | grep -q "unitree_foxglove_bridge"; then
    echo "  ✅ Foxglove Bridge: Running"
else
    echo "  ❌ Foxglove Bridge: Not running"
fi

if docker ps --format "table {{.Names}}\t{{.Status}}" | grep -q "unitree_ros_bridge"; then
    echo "  ✅ ROS Bridge: Running"
else
    echo "  ❌ ROS Bridge: Not running"
fi

echo ""

# Test connectivity if services are running
if docker ps --format "{{.Names}}" | grep -q "unitree_foxglove_bridge"; then
    echo "Testing Foxglove Bridge connectivity..."
    if timeout 3 bash -c "</dev/tcp/localhost/8765" 2>/dev/null; then
        echo "  ✅ Foxglove Bridge port 8765 is accessible"
    else
        echo "  ❌ Foxglove Bridge port 8765 is not accessible"
    fi
fi

if docker ps --format "{{.Names}}" | grep -q "unitree_ros_bridge"; then
    echo "Testing ROS Bridge connectivity..."
    if timeout 3 bash -c "</dev/tcp/localhost/9090" 2>/dev/null; then
        echo "  ✅ ROS Bridge port 9090 is accessible"
    else
        echo "  ❌ ROS Bridge port 9090 is not accessible"
    fi
fi

echo ""
echo "Client Connection Instructions:"
echo ""
echo "For Foxglove Studio:"
echo "  1. Open Foxglove Studio"
echo "  2. Click 'Open connection'"
echo "  3. Select 'Foxglove WebSocket'"
echo "  4. Enter URL: ws://$HOST_IP:8765"
echo "  5. Click 'Open'"
echo ""
echo "For ROS Bridge clients:"
echo "  WebSocket URL: ws://$HOST_IP:9090"
echo "  Available topics: /unilidar/cloud, /unilidar/imu"
echo ""
echo "For web browsers (testing):"
echo "  You can test the WebSocket connections using browser developer tools"
echo "  or online WebSocket testing tools with the URLs above."
echo ""

echo "Firewall Notes:"
echo "  Make sure ports 8765 and 9090 are open in your firewall"
echo "  Ubuntu/Debian: sudo ufw allow 8765 && sudo ufw allow 9090"
echo "  CentOS/RHEL: sudo firewall-cmd --add-port=8765/tcp --add-port=9090/tcp --permanent"