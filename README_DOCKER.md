# Unitree LiDAR ROS2 Docker Setup

This Docker setup allows you to run the Unitree L2 LiDAR in serial mode with ROS2 using Docker Compose, including network bridges for remote access via Foxglove Studio and other ROS clients.

## Quick Start

1. **Setup serial device permissions:**
   ```bash
   ./scripts/setup-udev.sh
   ```
   *Note: You may need to log out and log back in for the group changes to take effect.*

2. **Connect your Unitree L2 LiDAR via USB cable**

3. **Start the LiDAR service with network bridges:**
   ```bash
   ./scripts/start-lidar.sh
   ```

4. **Check network endpoints:**
   ```bash
   ./scripts/network-info.sh
   ```

5. **Stop the service:**
   ```bash
   ./scripts/stop-lidar.sh
   ```

## Available Scripts

### `./scripts/start-lidar.sh`
Main startup script with the following options:
- `--build`: Force rebuild of Docker images
- `--detach` or `-d`: Run in background (detached mode)
- `--no-bridges`: Skip starting Foxglove Bridge and ROS Bridge
- `--rviz-only`: Start only RViz (requires main service to be running)
- `--help`: Show help message

Examples:
```bash
./scripts/start-lidar.sh                # Start with bridges and RViz
./scripts/start-lidar.sh --build        # Rebuild and start all services
./scripts/start-lidar.sh --detach       # Start in background
./scripts/start-lidar.sh --no-bridges   # Start only LiDAR node (no network bridges)
./scripts/start-lidar.sh --rviz-only    # Start only RViz
```

### `./scripts/stop-lidar.sh`
Stop script with options:
- `--volumes` or `-v`: Remove volumes as well
- `--images`: Remove images as well

### `./scripts/setup-udev.sh`
Sets up udev rules for proper serial device access.

### `./scripts/network-info.sh`
Shows network endpoint information and connection status for bridges.

## Manual Docker Commands

If you prefer to use Docker Compose directly:

```bash
# Build and start
docker-compose up --build

# Start in background
docker-compose up -d

# Stop services
docker-compose down

# View logs
docker-compose logs -f
```

## Configuration

### Serial Device
The system automatically detects common serial devices:
- `/dev/ttyACM0`, `/dev/ttyACM1` (USB CDC devices)
- `/dev/ttyUSB0`, `/dev/ttyUSB1` (USB-Serial adapters)
- `/dev/unitree_lidar` (if udev rules are set up)

### Parameters
You can modify LiDAR parameters in:
- `config/lidar_params.yaml` - ROS2 parameter file
- `unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py` - Launch file parameters

### Environment Variables
Copy `.env.example` to `.env` and modify as needed:
```bash
cp .env.example .env
# Edit .env file with your preferred settings
```

## ROS2 Topics and Network Access

### ROS2 Topics
When running, the following topics will be available:
- `/unilidar/cloud` - Point cloud data (sensor_msgs/PointCloud2)
- `/unilidar/imu` - IMU data (sensor_msgs/Imu)

### Network Bridges
The system includes two network bridges for remote access:

#### Foxglove Bridge (Port 8765)
- **URL**: `ws://[HOST_IP]:8765`
- **Compatible with**: Foxglove Studio, other Foxglove-compatible clients
- **Protocol**: Foxglove WebSocket protocol
- **Usage**: 
  1. Open Foxglove Studio
  2. Connect to WebSocket
  3. Enter the URL with your host IP address

#### ROS Bridge (Port 9090)
- **URL**: `ws://[HOST_IP]:9090`
- **Compatible with**: Web applications, custom ROS clients
- **Protocol**: ROSBridge WebSocket protocol
- **Usage**: Connect any ROSBridge-compatible client to the WebSocket endpoint

### Accessing from Other Machines
1. Find your host machine's IP address: `./scripts/network-info.sh`
2. Ensure ports 8765 and 9090 are open in your firewall:
   ```bash
   sudo ufw allow 8765
   sudo ufw allow 9090
   ```
3. Connect from remote machines using the URLs shown above

### Connection Examples

#### Foxglove Studio Connection
1. Open Foxglove Studio
2. Click "Open connection"
3. Select **"Foxglove WebSocket"**
4. Enter URL: `ws://[HOST_IP]:8765`
5. Click "Open"
6. You should see `/unilidar/cloud` and `/unilidar/imu` topics available

#### ROS Bridge Connection  
1. Open Foxglove Studio or any ROS-compatible client
2. Click "Open connection"
3. Select **"Rosbridge (ROS 1 & ROS 2)"**
4. Enter URL: `ws://[HOST_IP]:9090`
5. Click "Open"
6. Topics will be available for subscription

**Note**: Both connection methods provide access to the same ROS2 topics, but may have different performance characteristics. Foxglove WebSocket is optimized for Foxglove Studio, while ROS Bridge provides broader compatibility.

## Troubleshooting

### Serial Device Not Found
1. Check if the device is connected: `ls -la /dev/tty* | grep -E '(ACM|USB)'`
2. Run the udev setup script: `./scripts/setup-udev.sh`
3. Make sure your user is in the dialout group: `groups $USER`

### Permission Denied
1. Run the udev setup script: `./scripts/setup-udev.sh`
2. Log out and log back in
3. Or run with sudo (not recommended): `sudo ./scripts/start-lidar.sh`

### RViz Not Displaying
1. Make sure X11 forwarding is set up: `echo $DISPLAY`
2. Allow Docker to access X11: `xhost +local:docker`
3. Check that the DISPLAY environment variable is set

### Build Errors
1. Make sure Docker has enough resources (RAM/disk space)
2. Try cleaning up: `docker system prune -a`
3. Rebuild from scratch: `./scripts/start-lidar.sh --build`

### Network Bridge Issues
1. Check if ports are available: `netstat -tuln | grep -E '(8765|9090)'`
2. Test connectivity: `./scripts/network-info.sh`
3. Check firewall settings: `sudo ufw status`
4. Ensure no other services are using ports 8765 or 9090

### Remote Connection Issues
1. Verify the host IP address: `./scripts/network-info.sh`
2. Test port accessibility from remote machine: `telnet [HOST_IP] 8765`
3. Check network routing and firewall rules
4. Try connecting from the same network first before external networks

## Hardware Requirements

- Unitree L2 LiDAR
- USB cable for serial connection
- Computer with Docker installed
- Linux system (tested on Ubuntu 20.04+)

## System Requirements

- Docker Engine 20.10+
- Docker Compose 2.0+ (or docker-compose 1.29+)
- X11 server for RViz display (optional)
- USB port for LiDAR connection
- Network connectivity for remote access

## Features

✅ **Serial Mode Connection** - Direct USB connection to Unitree L2 LiDAR  
✅ **ROS2 Integration** - Full ROS2 Humble support with standard message types  
✅ **Dual Network Bridges** - Both Foxglove WebSocket and ROS Bridge protocols  
✅ **Remote Access** - Access from any device on the network  
✅ **Point Cloud Visualization** - Real-time 3D point cloud data  
✅ **IMU Data** - 6-axis IMU data with quaternion orientation  
✅ **TF Broadcasting** - Proper coordinate frame transformations  
✅ **Docker Containerized** - Easy deployment and consistent environment