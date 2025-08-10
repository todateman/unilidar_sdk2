#!/bin/bash

# Unitree LiDAR Docker Compose Stop Script
# This script stops the Unitree LiDAR ROS2 services

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "=========================================="
echo "Stopping Unitree LiDAR ROS2 Services"
echo "=========================================="

# Prepare Docker Compose command
COMPOSE_CMD="docker-compose"
if ! command -v docker-compose > /dev/null 2>&1; then
    COMPOSE_CMD="docker compose"
fi

# Parse command line arguments
REMOVE_VOLUMES=false
REMOVE_IMAGES=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --volumes|-v)
            REMOVE_VOLUMES=true
            shift
            ;;
        --images)
            REMOVE_IMAGES=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --volumes, -v  Remove volumes as well"
            echo "  --images       Remove images as well"
            echo "  --help, -h     Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0             # Stop services"
            echo "  $0 --volumes   # Stop services and remove volumes"
            echo "  $0 --images    # Stop services and remove images"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "Stopping services..."
$COMPOSE_CMD down

if [ "$REMOVE_VOLUMES" = true ]; then
    echo "Removing volumes..."
    $COMPOSE_CMD down -v
fi

if [ "$REMOVE_IMAGES" = true ]; then
    echo "Removing images..."
    $COMPOSE_CMD down --rmi all
fi

echo "Services stopped successfully."