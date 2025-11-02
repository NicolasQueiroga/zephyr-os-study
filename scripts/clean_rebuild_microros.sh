#!/bin/bash
# Complete cleanup and rebuild of micro-ROS
# Use this when encountering persistent header file errors

set -e

echo "========================================"
echo "Complete micro-ROS Cleanup and Rebuild"
echo "========================================"
echo ""

# Check if ZEPHYR_WS is set
if [ -z "$ZEPHYR_WS" ]; then
    echo "⚠️  ZEPHYR_WS not set, using default: ~/zephyr-ws"
    ZEPHYR_WS=~/zephyr-ws
fi

# Check if ZEPHYR_VENV is set
if [ -z "$ZEPHYR_VENV" ]; then
    echo "⚠️  ZEPHYR_VENV not set, using default: ~/.zephyr_venv"
    ZEPHYR_VENV=~/.zephyr_venv
fi

echo "Environment:"
echo "  ZEPHYR_WS: $ZEPHYR_WS"
echo "  ZEPHYR_VENV: $ZEPHYR_VENV"
echo ""

# Unset ROS environment variables (in case on Ubuntu with ROS 2)
echo "Clearing ROS 2 environment variables..."
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
echo "✓ ROS environment cleared"
echo ""

# Clean project build
echo "Cleaning project build directory..."
make clean
echo "✓ Project build cleaned"
echo ""

# Remove ALL micro-ROS build artifacts
echo "Removing micro-ROS build artifacts..."
MICROROS_DIR="$ZEPHYR_WS/modules/micro_ros_zephyr_module/modules/libmicroros"

if [ -d "$MICROROS_DIR" ]; then
    echo "  Removing micro_ros_src..."
    rm -rf "$MICROROS_DIR/micro_ros_src"

    echo "  Removing micro_ros_dev..."
    rm -rf "$MICROROS_DIR/micro_ros_dev"

    echo "  Removing include directory..."
    rm -rf "$MICROROS_DIR/include"

    echo "  Removing libmicroros.a..."
    rm -f "$MICROROS_DIR/libmicroros.a"

    echo "  Removing configuration files..."
    rm -f "$MICROROS_DIR/configured_colcon.meta"
    rm -f "$MICROROS_DIR/zephyr_toolchain.cmake"

    echo "✓ All micro-ROS artifacts removed"
else
    echo "⚠️  micro-ROS directory not found at: $MICROROS_DIR"
    echo "   Please check your ZEPHYR_WS path"
    exit 1
fi
echo ""

# Clean CMake cache
echo "Cleaning CMake cache..."
rm -rf build/CMakeCache.txt build/CMakeFiles
echo "✓ CMake cache cleaned"
echo ""

echo "========================================"
echo "Starting complete rebuild..."
echo "This will take 5-10 minutes..."
echo "========================================"
echo ""

# Start the build - it will fail at rcutils, then we patch and retry
echo "Step 1: Initial build (may fail at rcutils - this is expected)..."
export ZEPHYR_VENV
make build 2>&1 | tee /tmp/microros_build.log || true

# Apply the patch if the file now exists
PATCH_FILE="$MICROROS_DIR/micro_ros_src/src/rcutils/src/time_unix.c"
if [ -f "$PATCH_FILE" ]; then
    echo ""
    echo "Step 2: Applying Zephyr 4.3+ compatibility patch..."
    if grep -q "zephyr/posix/time.h" "$PATCH_FILE"; then
        sed -i.bak 's|zephyr/posix/time.h|zephyr/posix/sys/time.h|g' "$PATCH_FILE"
        echo "✓ Patch applied to time_unix.c"

        echo ""
        echo "Step 3: Rebuilding with patch applied..."
        make build
    else
        echo "✓ Patch already applied or not needed"
    fi
else
    echo "⚠️  Warning: rcutils source file not found at: $PATCH_FILE"
    echo "   The build may have failed for a different reason."
    exit 1
fi

echo ""
echo "========================================"
echo "✓ Rebuild completed successfully!"
echo "========================================"
echo ""
echo "The micro-ROS library has been completely rebuilt."
echo "All headers should now be available."
