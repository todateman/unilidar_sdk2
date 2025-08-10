# ROS2 Cross-Host Network Setup

This guide explains how to set up ROS2 FastDDS for cross-host communication with the Unitree LiDAR Docker container.

## Configuration Overview

The Docker setup includes FastDDS configuration for multi-host communication using environment variables:

- **Environment Variables**: Configure automatic discovery and network range
- **Network Ports**: UDP ports 7400, 7410, 7411-7420 are exposed for DDS communication
- **Multicast Discovery**: Automatic discovery within subnet range

## Quick Start

1. **Start the container**:
   ```bash
   docker-compose up unitree_lidar
   ```

2. **On other PCs in the network**, set these environment variables:
   ```bash
   export ROS_DOMAIN_ID=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export ROS_LOCALHOST_ONLY=0
   export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
   ```

3. **Test communication**:
   ```bash
   # On other PC - list topics
   ros2 topic list
   
   # Subscribe to LiDAR data
   ros2 topic echo /unilidar/cloud
   ```

## Network Requirements

- All PCs must be on the same subnet
- Multicast traffic must be allowed
- UDP ports 7400, 7410, 7411-7420 must be accessible
- Firewall should allow these ports

## Troubleshooting

### No topics visible from other PCs

1. Check if multicast is working:
   ```bash
   ping 239.255.0.1
   ```

2. Verify network connectivity:
   ```bash
   nc -u <container-host-ip> 7400
   ```

3. Check ROS_DOMAIN_ID matches between all nodes

### Performance Issues

- Increase FastDDS buffer sizes in `fastdds_profiles.xml`
- Check network MTU settings
- Monitor network bandwidth usage

## Custom Configuration

Edit `fastdds_profiles.xml` to customize:
- Port numbers
- Multicast addresses  
- Buffer sizes
- Timeout values
- Quality of Service settings

Restart the container after making changes:
```bash
docker-compose restart unitree_lidar
```