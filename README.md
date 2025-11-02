# Zephyr RTOS Project with micro-ROS Support

This repository contains a Zephyr RTOS project for the STM32 DISCO_L475_IOT1 board with integrated micro-ROS support for ROS 2 communication.

**Platform Support:** Linux x86_64 and aarch64 only

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Installation Guide](#installation-guide)
  - [1. Install Zephyr SDK](#1-install-zephyr-sdk)
  - [2. Create Python Virtual Environment](#2-create-python-virtual-environment)
  - [3. Install West and Dependencies](#3-install-west-and-dependencies)
  - [4. Initialize Zephyr Workspace](#4-initialize-zephyr-workspace)
  - [5. Checkout Zephyr Version](#5-checkout-zephyr-version)
  - [6. Update Workspace](#6-update-workspace)
  - [7. Install micro-ROS Module](#7-install-micro-ros-module)
  - [8. Clone This Repository](#8-clone-this-repository)
  - [9. Build the Project](#9-build-the-project)
- [Building and Flashing](#building-and-flashing)
- [micro-ROS Integration](#micro-ros-integration)
- [Project Structure](#project-structure)
- [Makefile Commands](#makefile-commands)
- [Troubleshooting](#troubleshooting)
- [Additional Resources](#additional-resources)

## Features

- Zephyr RTOS application for STM32L4 microcontroller
- Distance measurement with ultrasonic sensor
- micro-ROS integration for ROS 2 communication
- Serial transport for micro-ROS
- Modular code structure with separate libraries
- Comprehensive Makefile for easy building and flashing

## Hardware Requirements

- **Board**: STM32 DISCO_L475_IOT1 Discovery Kit
- **Sensors**: Ultrasonic distance sensor (HC-SR04 or similar)
- **Connection**: USB cable for flashing and serial communication

## Installation Guide

This guide is specifically for **Linux x86_64 and aarch64** systems.

### 1. Install Zephyr SDK

The Zephyr SDK contains toolchains for all supported architectures.

**For x86_64:**
```bash
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.4/zephyr-sdk-0.17.4_linux-x86_64.tar.xz
tar xvf zephyr-sdk-0.17.4_linux-x86_64.tar.xz
cd zephyr-sdk-0.17.4
./setup.sh
```

**For aarch64:**
```bash
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.4/zephyr-sdk-0.17.4_linux-aarch64.tar.xz
tar xvf zephyr-sdk-0.17.4_linux-aarch64.tar.xz
cd zephyr-sdk-0.17.4
./setup.sh
```

### 2. Create Python Virtual Environment

```bash
# Create Python virtual environment for Zephyr
python3 -m venv ~/.zephyr_venv

# Activate the virtual environment
source ~/.zephyr_venv/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### 3. Install West and Dependencies

```bash
# Still in activated venv
pip install west

# Install micro-ROS build dependencies
pip install catkin_pkg lark-parser 'empy<4.0' colcon-common-extensions

# Deactivate for now
deactivate
```

### 4. Initialize Zephyr Workspace

```bash
# Create workspace directory
mkdir -p ~/zephyr-ws
cd ~/zephyr-ws

# Initialize workspace with west
source ~/.zephyr_venv/bin/activate
west init .
cd zephyr
```

### 5. Checkout Zephyr Version

```bash
# Checkout specific version (recommended for stability)
git checkout v4.3.0-rc2

# Or use latest stable
# git checkout main
```

### 6. Update Workspace

```bash
# Update all modules and dependencies
cd ~/zephyr-ws/zephyr
west update
```

### 7. Install micro-ROS Module

```bash
# Create modules directory if it doesn't exist
mkdir -p ~/zephyr-ws/modules

# Clone micro-ROS module
cd ~/zephyr-ws/modules
git clone -b jazzy https://github.com/micro-ROS/micro_ros_zephyr_module.git

# Apply Zephyr 4.3+ compatibility patch
sed -i.bak 's|zephyr/posix/time.h|zephyr/posix/sys/time.h|g' \
  micro_ros_zephyr_module/modules/libmicroros/micro_ros_src/src/rcutils/src/time_unix.c 2>/dev/null || true
```

### 8. Clone This Repository

```bash
# Clone to your preferred location
cd ~
git clone git@github.com:NicolasQueiroga/zephyr-os-study.git
cd zephyr-os-study
```

### 9. Build the Project

```bash
# Set environment variables
export ZEPHYR_VENV=~/.zephyr_venv
export ZEPHYR_WS=~/zephyr-ws

# Add to your ~/.bashrc for persistence
echo 'export ZEPHYR_VENV=~/.zephyr_venv' >> ~/.bashrc
echo 'export ZEPHYR_WS=~/zephyr-ws' >> ~/.bashrc

# Build the project (includes micro-ROS)
make build
```

## Building and Flashing

### Quick Start

```bash
export ZEPHYR_VENV=~/.zephyr_venv
export ZEPHYR_WS=~/zephyr-ws

# Build the project
make build

# Flash to board
make flash

# Or do both in one command
make build-flash
```

### Manual Commands with West

If you prefer using west directly:

```bash
# Activate virtual environment
source ~/.zephyr_venv/bin/activate

# Build
west build -b disco_l475_iot1 -p auto

# Flash
west flash

# Debug
west debug
```

## micro-ROS Integration

### What is micro-ROS?

micro-ROS brings ROS 2 to microcontrollers, enabling:
- Publish/subscribe messaging
- Service calls
- Parameter management
- Integration with ROS 2 ecosystem

### Transport Configuration

This project uses **serial transport** over UART. Configuration in [prj.conf](prj.conf):

```conf
CONFIG_MICROROS=y
CONFIG_MICROROS_TRANSPORT_SERIAL=y
CONFIG_MICROROS_SERIAL_PORT="1"
```

### Using micro-ROS in Your Code

The project includes a micro-ROS wrapper in [src/uros/](src/uros/):

```c
#include "uros/uros.h"

int main(void) {
    // Initialize micro-ROS
    if (uros_init() == 0) {
        // Start ping-pong example
        uros_ping_pong_start();
    }

    // Main loop
    while (1) {
        // Process micro-ROS messages
        uros_spin();
        k_msleep(10);
    }
}
```

### Testing with ROS 2

**Terminal 1 - Start micro-ROS Agent:**
```bash
# Find your board's serial port
ls /dev/ttyACM*

# Start agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6
```

**Terminal 2 - Monitor Messages:**
```bash
# Echo messages from device
ros2 topic echo /ping

# List all topics
ros2 topic list
```

**Terminal 3 - Send Messages:**
```bash
# Publish to device
ros2 topic pub /pong std_msgs/msg/Int32 "data: 42"
```

## Project Structure

```
zephyr-os-study/
├── src/
│   ├── main.c              # Main application
│   ├── distance/           # Distance sensor library
│   │   ├── distance.h
│   │   └── distance.c
│   ├── utils/              # Utility functions
│   │   ├── utils.h
│   │   └── utils.c
│   └── uros/               # micro-ROS wrapper
│       ├── uros.h          # micro-ROS API
│       └── uros.c          # Ping-pong implementation
├── CMakeLists.txt          # CMake build configuration
├── prj.conf                # Zephyr project configuration
├── custom_pins.overlay     # Device tree overlay
├── Makefile                # Convenient build commands
└── README.md               # This file
```

## Makefile Commands

Run `make help` to see all available commands:

| Command | Description |
|---------|-------------|
| `make build` | Build the project |
| `make clean` | Clean build artifacts |
| `make pristine` | Remove all build artifacts and start fresh |
| `make flash` | Flash firmware to board |
| `make build-flash` | Build and flash in one command |
| `make rebuild` | Clean and rebuild |
| `make debug` | Start debugger |
| `make menuconfig` | Open Kconfig menu |
| `make size` | Show binary size information |
| `make ram-report` | Show RAM usage report |
| `make uros-agent` | Run micro-ROS agent (requires ROS 2) |
| `make check` | Verify environment setup |
| `make info` | Show project information |
| `make update` | Update Zephyr and modules |

## Troubleshooting

### Build Fails with "ZEPHYR_VENV not set"

```bash
export ZEPHYR_VENV=~/.zephyr_venv
export ZEPHYR_WS=~/zephyr-ws

# Add to ~/.bashrc for persistence
echo 'export ZEPHYR_VENV=~/.zephyr_venv' >> ~/.bashrc
echo 'export ZEPHYR_WS=~/zephyr-ws' >> ~/.bashrc
source ~/.bashrc
```

### micro-ROS Build Fails

If micro-ROS build fails, ensure you applied the Zephyr 4.3+ compatibility patch during installation:

```bash
sed -i.bak 's|zephyr/posix/time.h|zephyr/posix/sys/time.h|g' \
  ~/zephyr-ws/modules/micro_ros_zephyr_module/modules/libmicroros/micro_ros_src/src/rcutils/src/time_unix.c

# Clean and rebuild
make clean
make build
```

### Missing micro-ROS Headers

If you see errors like `fatal error: action_msgs/msg/goal_info.h: No such file or directory`:

```bash
# Clean everything and rebuild from scratch
make pristine
make build
```

### ROS 2 Conflict on Ubuntu

The Makefile automatically isolates builds from system ROS 2. If you still see conflicts:

```bash
# Temporarily unset ROS environment
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
make clean
make build
```

Or comment out ROS 2 sourcing in `~/.bashrc` temporarily:
```bash
# Comment out this line during build:
# source /opt/ros/jazzy/setup.bash
```

### Board Not Detected

```bash
# Check for device
ls /dev/ttyACM*

# Add user to dialout group (requires logout/login)
sudo usermod -a -G dialout $USER

# Or use sudo for flash
sudo make flash
```

### Flash Fails

1. Check board is connected via USB
2. Try holding RESET button and run flash again
3. Verify permissions: `ls -l /dev/ttyACM*`
4. Try: `sudo make flash`

### System Dependencies Missing

```bash
# Install all system dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install -y git cmake ninja-build gperf ccache dfu-util \
  device-tree-compiler wget python3 python3-pip python3-setuptools \
  python3-wheel xz-utils file make gcc gcc-multilib g++-multilib \
  libsdl2-dev libmagic1

# Install Zephyr Python requirements
source ~/.zephyr_venv/bin/activate
pip install -r ~/zephyr-ws/zephyr/scripts/requirements.txt
```

## Additional Resources

- [Zephyr Documentation](https://docs.zephyrproject.org/)
- [Zephyr Getting Started](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
- [micro-ROS Documentation](https://micro.ros.org/)
- [micro-ROS Zephyr Module](https://github.com/micro-ROS/micro_ros_zephyr_module)
- [STM32 DISCO_L475_IOT1 Board](https://docs.zephyrproject.org/latest/boards/st/disco_l475_iot1/doc/index.html)
- [ROS 2 Documentation](https://docs.ros.org/)

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Authors

NicolasQueiroga
