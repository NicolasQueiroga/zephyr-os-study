# Zephyr RTOS Project with micro-ROS Support

This repository contains a Zephyr RTOS project for the STM32 DISCO_L475_IOT1 board with integrated micro-ROS support for ROS 2 communication.

## Table of Contents

- [Zephyr RTOS Project with micro-ROS Support](#zephyr-rtos-project-with-micro-ros-support)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
  - [Hardware Requirements](#hardware-requirements)
  - [Complete Installation Guide](#complete-installation-guide)
    - [1. Install System Dependencies](#1-install-system-dependencies)
    - [2. Install Zephyr SDK](#2-install-zephyr-sdk)
    - [3. Set Up Zephyr Workspace](#3-set-up-zephyr-workspace)
    - [4. Set Up Python Virtual Environment](#4-set-up-python-virtual-environment)
    - [5. Clone This Project](#5-clone-this-project)
    - [6. Install micro-ROS Module](#6-install-micro-ros-module)
    - [7. Configure Environment Variables](#7-configure-environment-variables)
  - [Building and Flashing](#building-and-flashing)
    - [Quick Start](#quick-start)
    - [Manual Commands](#manual-commands)
  - [micro-ROS Integration](#micro-ros-integration)
    - [What is micro-ROS?](#what-is-micro-ros)
    - [Transport Configuration](#transport-configuration)
    - [Using micro-ROS in Your Code](#using-micro-ros-in-your-code)
    - [Testing with ROS 2](#testing-with-ros-2)
  - [Project Structure](#project-structure)
  - [Makefile Commands](#makefile-commands)
  - [Troubleshooting](#troubleshooting)
    - [Build Fails with "ZEPHYR\_VENV not set"](#build-fails-with-zephyr_venv-not-set)
    - [micro-ROS Build Fails](#micro-ros-build-fails)
    - [Permission Errors During Build](#permission-errors-during-build)
    - [Board Not Detected](#board-not-detected)
    - [Flash Fails](#flash-fails)
  - [Additional Resources](#additional-resources)
  - [Contributing](#contributing)
  - [License](#license)
  - [Authors](#authors)

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

## Complete Installation Guide

This guide ensures reproducibility across different machines.

### 1. Install System Dependencies

**macOS:**
```bash
# Install Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install required tools
brew install cmake ninja gperf dtc python3 wget
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt update
sudo apt install -y git cmake ninja-build gperf ccache dfu-util \
  device-tree-compiler wget python3 python3-pip python3-setuptools \
  python3-wheel xz-utils file make gcc gcc-multilib g++-multilib \
  libsdl2-dev libmagic1
```

### 2. Install Zephyr SDK

The Zephyr SDK contains toolchains for all supported architectures.

```bash
# Download Zephyr SDK (version 0.17.4)
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.4/zephyr-sdk-0.17.4_macos-x86_64.tar.xz

# Extract SDK
tar xvf zephyr-sdk-0.17.4_macos-x86_64.tar.xz

# Run the setup script
cd zephyr-sdk-0.17.4
./setup.sh

# Register CMake package (recommended)
./setup.sh -c
```

**For Linux x86_64:**
```bash
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.4/zephyr-sdk-0.17.4_linux-x86_64.tar.xz
tar xvf zephyr-sdk-0.17.4_linux-x86_64.tar.xz
cd zephyr-sdk-0.17.4
./setup.sh
```

**For Apple Silicon (ARM64):**
```bash
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.4/zephyr-sdk-0.17.4_macos-aarch64.tar.xz
tar xvf zephyr-sdk-0.17.4_macos-aarch64.tar.xz
cd zephyr-sdk-0.17.4
./setup.sh
```

### 3. Set Up Zephyr Workspace

```bash
# Create workspace directory
mkdir -p ~/zephyr-ws
cd ~/zephyr-ws

# Clone Zephyr repository
git clone https://github.com/zephyrproject-rtos/zephyr.git
cd zephyr
git checkout v4.3.0-rc2  # Or latest stable version

# Note the workspace path - you'll need this later
pwd  # Should output: /Users/yourusername/zephyr-ws/zephyr
```

### 4. Set Up Python Virtual Environment

```bash
# Create Python virtual environment for Zephyr
python3 -m venv ~/.zephyr_venv

# Activate the virtual environment
source ~/.zephyr_venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install West (Zephyr's meta-tool)
pip install west

# Install Zephyr Python dependencies
pip install -r ~/zephyr-ws/zephyr/scripts/requirements.txt

# Install micro-ROS dependencies
pip install catkin_pkg lark-parser 'empy<4.0' colcon-common-extensions

# Deactivate for now (Makefile will activate when needed)
deactivate
```

### 5. Clone This Project

```bash
cd ~/Documents  # Or your preferred location
git clone <your-repo-url> zephyr-os-study
cd zephyr-os-study
```

### 6. Install micro-ROS Module

```bash
# Clone micro-ROS module into Zephyr workspace
cd ~/zephyr-ws/modules
git clone -b jazzy https://github.com/micro-ROS/micro_ros_zephyr_module.git

# Apply compatibility patch for Zephyr 4.3+
sed -i.bak 's|zephyr/posix/time.h|zephyr/posix/sys/time.h|g' \
  ~/zephyr-ws/modules/micro_ros_zephyr_module/modules/libmicroros/micro_ros_src/src/rcutils/src/time_unix.c
```

### 7. Configure Environment Variables

Add these to your shell profile (`~/.zshrc` or `~/.bashrc`):

```bash
# Zephyr environment variables
export ZEPHYR_VENV=~/.zephyr_venv
export ZEPHYR_WS=~/zephyr-ws

# Optional: Add to PATH for easy access
export PATH="$ZEPHYR_VENV/bin:$PATH"
```

Then reload your shell:
```bash
source ~/.zshrc  # or source ~/.bashrc
```

## Building and Flashing

### Quick Start

**First time build (or after clean install):**
```bash
# Set environment variables (if not in shell profile)
export ZEPHYR_VENV=~/.zephyr_venv
export ZEPHYR_WS=~/zephyr-ws

# Build micro-ROS library first (takes 5-10 minutes)
make build-microros

# Then build the project
make build

# Flash to board
make flash
```

**Subsequent builds:**
```bash
export ZEPHYR_VENV=~/.zephyr_venv

# Build the project
make build

# Flash to board
make flash

# Or do both in one command
make build-flash
```

### Manual Commands

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

The project includes a micro-ROS wrapper in `src/uros/`:

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
ls /dev/tty.*  # macOS
ls /dev/ttyACM*  # Linux

# Start agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/tty.usbmodem* -v6
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
├── west.yml                # West manifest (micro-ROS)
├── Makefile                # Convenient build commands
├── README.md               # This file
└── .env.example            # Environment variable template
```

## Makefile Commands

Run `make help` to see all available commands:

| Command | Description |
|---------|-------------|
| `make build` | Build the project |
| `make clean` | Clean build artifacts |
| `make flash` | Flash firmware to board |
| `make build-flash` | Build and flash in one command |
| `make rebuild` | Clean and rebuild |
| `make debug` | Start debugger |
| `make menuconfig` | Open Kconfig menu |
| `make size` | Show binary size information |
| `make uros-agent` | Run micro-ROS agent (requires ROS 2) |
| `make check` | Verify environment setup |
| `make install-deps` | Install all dependencies |
| `make update` | Update Zephyr and modules |

## Troubleshooting

### Build Fails with "ZEPHYR_VENV not set"

```bash
export ZEPHYR_VENV=~/.zephyr_venv
# Or add to ~/.zshrc or ~/.bashrc
```

### micro-ROS Build Fails

1. **Check empy version:**
```bash
source ~/.zephyr_venv/bin/activate
pip install 'empy<4.0'
```

2. **Verify colcon is installed:**
```bash
pip install colcon-common-extensions
```

3. **Reapply time.h patch:**
```bash
sed -i.bak 's|zephyr/posix/time.h|zephyr/posix/sys/time.h|g' \
  ~/zephyr-ws/modules/micro_ros_zephyr_module/modules/libmicroros/micro_ros_src/src/rcutils/src/time_unix.c
```

### ROS 2 Conflict on Ubuntu (fastcdr version error)

If you have ROS 2 installed on Ubuntu and see errors like:
```
Could not find a configuration file for package "fastcdr" that is compatible with requested version "2"
version: 2.2.5 (64bit)
```

**Solution:** The Makefile automatically isolates the build from system ROS 2. If the error persists, use the provided script:

```bash
# Run the fix script
./scripts/fix_ros2_conflict.sh
```

Or manually:
```bash
# Temporarily unset ROS environment before building
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
export ZEPHYR_VENV=~/.zephyr_venv
make clean
make build
```

**Alternative:** Comment out ROS 2 sourcing in `~/.bashrc` during the build:
```bash
# Comment out this line temporarily:
# source /opt/ros/jazzy/setup.bash
```

**Why this happens:** System ROS 2 uses 64-bit libraries, while micro-ROS for embedded targets needs 32-bit ARM libraries. The build system gets confused when both are present.

### Missing micro-ROS Headers (action_msgs/msg/goal_info.h)

If you see errors like:
```
fatal error: action_msgs/msg/goal_info.h: No such file or directory
```

This means the micro-ROS library build didn't complete properly or the build cache is stale.

**Solution - Use the cleanup script:**
```bash
# This script does a complete cleanup and rebuild
export ZEPHYR_VENV=~/.zephyr_venv
export ZEPHYR_WS=~/zephyr-ws
./scripts/clean_rebuild_microros.sh
```

The script will:
- Clean all build artifacts
- Remove all micro-ROS cached files
- Clear CMake cache
- Rebuild everything from scratch (takes 5-10 minutes)

### Permission Errors During Build

The project creates a `.cache` directory for writable cache. If you see permission errors:

```bash
mkdir -p .cache
chmod 755 .cache
```

### Board Not Detected

**macOS:**
```bash
ls /dev/tty.*
# Look for /dev/tty.usbmodem* or similar
```

**Linux:**
```bash
ls /dev/ttyACM*
# You may need to add user to dialout group
sudo usermod -a -G dialout $USER
```

### Flash Fails

1. Check board is connected via USB
2. Try holding RESET button and run flash again
3. Verify permissions: `ls -l /dev/tty.*`

## Additional Resources

- [Zephyr Documentation](https://docs.zephyrproject.org/)
- [Zephyr Getting Started](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
- [micro-ROS Documentation](https://micro.ros.org/)
- [micro-ROS Zephyr Module](https://github.com/micro-ROS/micro_ros_zephyr_module)
- [STM32 DISCO_L475_IOT1 Board](https://docs.zephyrproject.org/latest/boards/st/disco_l475_iot1/doc/index.html)
- [ROS 2 Documentation](https://docs.ros.org/)

## Contributing

When contributing to this project:

1. Test on clean environment to verify reproducibility
2. Update this README with any new dependencies
3. Document new features in appropriate sections
4. Follow Zephyr coding style guidelines

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors

NicolasQueiroga
