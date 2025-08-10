#!/bin/bash

# Unitree LiDAR Serial Device Setup Script
# This script sets up udev rules for Unitree LiDAR serial devices

set -e

echo "Setting up udev rules for Unitree LiDAR..."

# Create udev rules file
sudo tee /etc/udev/rules.d/99-unitree-lidar.rules > /dev/null << 'EOF'
# Unitree LiDAR Serial Device Rules
# This file creates stable device names and sets proper permissions for Unitree LiDAR devices

# Generic USB-Serial converters (common for Unitree LiDAR)
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="unitree_lidar", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="unitree_lidar", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="unitree_lidar", GROUP="dialout", MODE="0666"

# Add common USB-Serial adapter vendors
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="unitree_lidar", GROUP="dialout", MODE="0666"

# Generic rule for ACM devices (USB CDC)
SUBSYSTEM=="tty", KERNEL=="ttyACM*", SYMLINK+="unitree_lidar_acm%n", GROUP="dialout", MODE="0666"

# Generic rule for USB devices
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", SYMLINK+="unitree_lidar_usb%n", GROUP="dialout", MODE="0666"
EOF

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Adding current user to dialout group..."
sudo usermod -a -G dialout $USER

echo ""
echo "Setup completed!"
echo ""
echo "NOTE: You may need to log out and log back in for group changes to take effect."
echo ""
echo "Available device names after connecting the LiDAR:"
echo "  - /dev/ttyACM0, /dev/ttyACM1, etc. (standard names)"
echo "  - /dev/unitree_lidar (if device is recognized by vendor/product ID)"
echo "  - /dev/unitree_lidar_acm0, /dev/unitree_lidar_acm1, etc. (ACM devices)"
echo "  - /dev/unitree_lidar_usb0, /dev/unitree_lidar_usb1, etc. (USB devices)"
echo ""
echo "To check connected devices, run:"
echo "  ls -la /dev/tty* | grep -E '(ACM|USB)'"
echo "  ls -la /dev/unitree_lidar*"