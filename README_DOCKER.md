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

### `./scripts/record-bag.sh`

Records ROS2 topics to bag files for data analysis and replay:

- `--duration, -d SEC`: Recording duration in seconds
- `--name, -n NAME`: Bag file name (default: auto-generated timestamp)
- `--output, -o DIR`: Output directory (default: /workspace/bags)
- `--compress, -c`: Enable compression (zstd format)
- `--cloud-only`: Record only point cloud and TF data
- `--imu-only`: Record only IMU and TF data
- `--all, -a`: Record all available topics
- `--help, -h`: Show help message

Examples:
```bash
# Basic recording (10 seconds)
./scripts/record-bag.sh --duration 10

# Record with compression and custom name
./scripts/record-bag.sh --duration 30 --compress --name experiment_01

# Record only point cloud data
./scripts/record-bag.sh --cloud-only --duration 60

# Record only IMU data for lightweight collection
./scripts/record-bag.sh --imu-only --duration 120
```

### `./scripts/copy-bags.sh`

Copy recorded bag files from container to local host:

- `--list, -l`: List available bags in container
- `--bag, -b NAME`: Copy specific bag by name
- `--help, -h`: Show help message

Examples:
```bash
# List all available bags
./scripts/copy-bags.sh --list

# Copy specific bag to local directory
./scripts/copy-bags.sh --bag experiment_01

# Copy all bags to local directory
./scripts/copy-bags.sh
```

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

## Data Recording and Analysis

### Recording ROS2 Bag Files

The system provides convenient scripts for recording LiDAR data to ROS2 bag files for later analysis and replay.

#### Basic Recording Workflow

1. **Start the LiDAR service:**
   ```bash
   ./scripts/start-lidar.sh --detach
   ```

2. **Record data:**
   ```bash
   # Record for 60 seconds with automatic timestamped filename
   ./scripts/record-bag.sh --duration 60
   
   # Record with custom name and compression
   ./scripts/record-bag.sh --duration 120 --name "outdoor_test_01" --compress
   ```

3. **Copy to local storage:**
   ```bash
   # List available recordings
   ./scripts/copy-bags.sh --list
   
   # Copy specific recording to local ./bags/ directory
   ./scripts/copy-bags.sh --bag outdoor_test_01
   ```

#### Recording Options

- **Full recording**: All topics including point cloud, IMU, and TF data
- **Cloud-only**: `--cloud-only` for point cloud and coordinate frame data
- **IMU-only**: `--imu-only` for lightweight motion data collection
- **Compression**: `--compress` to reduce file size by ~60-70%

#### Data Storage Locations

- **Container storage**: `/workspace/bags/[bag_name]/`
- **Local storage**: `./bags/[bag_name]/` (after copying)

#### File Size Expectations

| Duration | Uncompressed | Compressed | Content |
|----------|-------------|------------|---------|
| 10 seconds | ~280MB | ~100MB | Full data (cloud + IMU + TF) |
| 30 seconds | ~840MB | ~300MB | Full data |
| 60 seconds | ~1.7GB | ~600MB | Full data |
| 60 seconds | ~20MB | ~8MB | IMU-only |

#### Bag File Replay

To replay recorded data:

```bash
# From local directory
ros2 bag play ./bags/outdoor_test_01

# From container (if not copied locally)
docker compose exec unitree_lidar bash -c "
  source /opt/ros/humble/setup.bash && 
  ros2 bag play /workspace/bags/outdoor_test_01
"
```

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
✅ **Data Recording** - ROS2 bag recording with automatic compression and local storage  
✅ **Selective Topic Publishing** - Optimize network transmission by selecting specific topics