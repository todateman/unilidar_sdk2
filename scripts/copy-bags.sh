#!/bin/bash

# Copy ROS bags from Docker container to local host

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Default settings
LOCAL_BAGS_DIR="./bags"
CONTAINER_BAGS_DIR="/workspace/bags"

# Parse command line arguments
SPECIFIC_BAG=""
LIST_ONLY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --bag|-b)
            SPECIFIC_BAG="$2"
            shift 2
            ;;
        --list|-l)
            LIST_ONLY=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --bag, -b NAME        Copy specific bag by name"
            echo "  --list, -l            List available bags in container"
            echo "  --help, -h            Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                              # Copy all bags"
            echo "  $0 --list                       # List available bags"  
            echo "  $0 --bag test_local_mount       # Copy specific bag"
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
    echo "Warning: Unitree LiDAR service is not running."
    echo "Some bags may not be accessible."
fi

echo "=========================================="
echo "ROS Bag Copy from Container"
echo "=========================================="

# Create local bags directory
mkdir -p "$LOCAL_BAGS_DIR"

if [ "$LIST_ONLY" = true ]; then
    echo "Available bags in container:"
    echo ""
    docker compose exec unitree_lidar bash -c "
        if [ -d $CONTAINER_BAGS_DIR ]; then
            for bag_dir in $CONTAINER_BAGS_DIR/*/; do
                if [ -d \"\$bag_dir\" ]; then
                    bag_name=\$(basename \"\$bag_dir\")
                    size=\$(du -sh \"\$bag_dir\" | cut -f1)
                    files=\$(find \"\$bag_dir\" -name '*.db3' | wc -l)
                    echo \"  \$bag_name - \$size (\$files files)\"
                fi
            done
        else
            echo '  No bags directory found in container'
        fi
    " 2>/dev/null || echo "  Could not access container"
    exit 0
fi

if [ -n "$SPECIFIC_BAG" ]; then
    # Copy specific bag
    echo "Copying specific bag: $SPECIFIC_BAG"
    
    if docker compose exec unitree_lidar test -d "$CONTAINER_BAGS_DIR/$SPECIFIC_BAG" 2>/dev/null; then
        docker cp "unitree_lidar_ros2:$CONTAINER_BAGS_DIR/$SPECIFIC_BAG" "$LOCAL_BAGS_DIR/"
        
        # Fix permissions
        sudo chown -R $USER:$USER "$LOCAL_BAGS_DIR/$SPECIFIC_BAG" 2>/dev/null || true
        
        echo "Bag '$SPECIFIC_BAG' copied to: $LOCAL_BAGS_DIR/$SPECIFIC_BAG"
        
        # Show size info
        size=$(du -sh "$LOCAL_BAGS_DIR/$SPECIFIC_BAG" | cut -f1)
        files=$(find "$LOCAL_BAGS_DIR/$SPECIFIC_BAG" -name '*.db3' | wc -l)
        echo "Size: $size ($files database files)"
    else
        echo "Error: Bag '$SPECIFIC_BAG' not found in container"
        exit 1
    fi
else
    # Copy all bags
    echo "Copying all bags from container..."
    
    # Get list of bags
    bag_list=$(docker compose exec unitree_lidar bash -c "
        if [ -d $CONTAINER_BAGS_DIR ]; then
            find $CONTAINER_BAGS_DIR -maxdepth 1 -type d ! -path $CONTAINER_BAGS_DIR | xargs -I {} basename {}
        fi
    " 2>/dev/null | tr '\n' ' ')
    
    if [ -z "$bag_list" ]; then
        echo "No bags found in container."
        exit 0
    fi
    
    echo "Found bags: $bag_list"
    echo ""
    
    for bag in $bag_list; do
        if [ -n "$bag" ]; then
            echo "Copying: $bag"
            docker cp "unitree_lidar_ros2:$CONTAINER_BAGS_DIR/$bag" "$LOCAL_BAGS_DIR/" 2>/dev/null || echo "  Failed to copy $bag"
        fi
    done
    
    # Fix permissions for all copied bags
    sudo chown -R $USER:$USER "$LOCAL_BAGS_DIR" 2>/dev/null || true
    
    echo ""
    echo "All bags copied to: $LOCAL_BAGS_DIR"
fi

echo ""
echo "Local bags summary:"
if [ -d "$LOCAL_BAGS_DIR" ]; then
    for bag_dir in "$LOCAL_BAGS_DIR"/*/; do
        if [ -d "$bag_dir" ]; then
            bag_name=$(basename "$bag_dir")
            size=$(du -sh "$bag_dir" | cut -f1)
            files=$(find "$bag_dir" -name '*.db3' | wc -l 2>/dev/null || echo "0")
            echo "  $bag_name - $size ($files files)"
        fi
    done
else
    echo "  No local bags directory found"
fi