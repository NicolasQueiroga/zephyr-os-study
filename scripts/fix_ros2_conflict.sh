#!/bin/bash
# Fix ROS 2 conflict on systems with ROS 2 installed
# This script isolates the micro-ROS build from system ROS 2 installation

set -e

echo "========================================"
echo "Fixing ROS 2 Conflict"
echo "========================================"
echo ""

# Check if ROS_DISTRO is set
if [ -n "$ROS_DISTRO" ]; then
    echo "⚠️  Detected ROS 2 $ROS_DISTRO in environment"
    echo "   This may cause conflicts with micro-ROS build"
    echo ""
fi

# Unset ROS environment variables
echo "Unsetting ROS 2 environment variables..."
unset ROS_DISTRO
unset ROS_VERSION
unset ROS_PYTHON_VERSION
unset CMAKE_PREFIX_PATH
unset AMENT_PREFIX_PATH

echo "✓ ROS 2 environment cleared"
echo ""

# Check if ZEPHYR_VENV is set
if [ -z "$ZEPHYR_VENV" ]; then
    echo "⚠️  ZEPHYR_VENV not set"
    echo "   Using default: ~/.zephyr_venv"
    export ZEPHYR_VENV=~/.zephyr_venv
fi

echo "Environment:"
echo "  ZEPHYR_VENV: $ZEPHYR_VENV"
echo ""

# Clean and rebuild
echo "Cleaning build directory..."
make clean

echo ""
echo "Starting build (this may take a few minutes)..."
echo "========================================"
make build

echo ""
echo "========================================"
echo "✓ Build completed successfully!"
echo "========================================"
