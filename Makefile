# Zephyr RTOS Project Makefile
# Project: embarcados-zephyr
# Board: disco_l475_iot1

# Configuration
BOARD := disco_l475_iot1
BUILD_DIR := build

# Environment variables (set these in your shell or .env file)
# ZEPHYR_VENV: path to your Zephyr Python virtual environment
# ZEPHYR_WS: path to your Zephyr workspace (optional, for updates)
ZEPHYR_VENV ?= $(error ZEPHYR_VENV not set. Export ZEPHYR_VENV=/path/to/.zephyr_venv)
ZEPHYR_WS ?= $(HOME)/zephyr-ws

# Set ZEPHYR_BASE to point to zephyr directory in workspace
export ZEPHYR_BASE := $(ZEPHYR_WS)/zephyr

# West command with venv activation and proper environment setup
# Unset ROS environment variables to prevent conflicts with system ROS 2 installation
# Add venv/bin to PATH so CMake subprocesses can find colcon
WEST := unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION CMAKE_PREFIX_PATH AMENT_PREFIX_PATH && \
        . $(ZEPHYR_VENV)/bin/activate && \
        export ZEPHYR_BASE=$(ZEPHYR_BASE) && \
        export USER_CACHE_DIR=$(USER_CACHE_DIR) && \
        export PATH=$(ZEPHYR_VENV)/bin:$$PATH && \
        west

.PHONY: help
help: ## Show this help message
	@echo "Zephyr Project Makefile - disco_l475_iot1"
	@echo ""
	@echo "Usage: ZEPHYR_VENV=/path/to/venv make [target]"
	@echo ""
	@echo "Available targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

.PHONY: build
build: ## Build the project
	@echo "Building for $(BOARD)..."
	$(WEST) build -b $(BOARD) -p auto

.PHONY: clean
clean: ## Clean build artifacts
	@echo "Cleaning build directory..."
	rm -rf $(BUILD_DIR)

.PHONY: pristine
pristine: ## Remove all build artifacts and start fresh
	@echo "Pristine build directory..."
	$(WEST) build -t pristine || rm -rf $(BUILD_DIR)

.PHONY: flash
flash: ## Flash the board
	@echo "Flashing $(BOARD)..."
	$(WEST) flash -r openocd

.PHONY: flash-openocd
flash-openocd: ## Flash the board using OpenOCD
	@echo "Flashing $(BOARD) with OpenOCD..."
	$(WEST) flash -r openocd

.PHONY: build-flash
build-flash: build flash ## Build and flash in one command

.PHONY: debug
debug: ## Start debugger
	@echo "Starting debugger for $(BOARD)..."
	$(WEST) debug

.PHONY: debug-server
debug-server: ## Start debug server
	@echo "Starting debug server for $(BOARD)..."
	$(WEST) debugserver

.PHONY: attach
attach: ## Attach debugger to running target
	@echo "Attaching debugger..."
	$(WEST) attach

.PHONY: menuconfig
menuconfig: ## Open Kconfig menu configuration
	@echo "Opening menuconfig..."
	$(WEST) build -t menuconfig

.PHONY: guiconfig
guiconfig: ## Open GUI configuration
	@echo "Opening guiconfig..."
	$(WEST) build -t guiconfig

.PHONY: size
size: ## Show binary size information
	@echo "Binary size information:"
	$(WEST) build -t rom_report

.PHONY: ram-report
ram-report: ## Show RAM usage report
	@echo "RAM usage report:"
	$(WEST) build -t ram_report

.PHONY: puncover
puncover: ## Generate code size analysis with puncover
	@echo "Generating puncover analysis..."
	$(WEST) build -t puncover

.PHONY: info
info: ## Show project information
	@echo "Project Information:"
	@echo "  Board:        $(BOARD)"
	@echo "  Build Dir:    $(BUILD_DIR)"
	@echo "  Zephyr Venv:  $(ZEPHYR_VENV)"
	@echo "  Zephyr WS:    $(ZEPHYR_WS)"
	@echo ""
	@$(WEST) --version 2>/dev/null || echo "Error: Cannot access west in venv"

.PHONY: list-boards
list-boards: ## List all available boards
	@echo "Available boards:"
	$(WEST) boards

.PHONY: update
update: ## Update Zephyr and modules
	@echo "Updating Zephyr workspace at $(ZEPHYR_WS)..."
	cd $(ZEPHYR_WS) && $(WEST) update

.PHONY: uros-agent
uros-agent: ## Run micro-ROS agent for serial connection
	@echo "Starting micro-ROS agent on serial..."
	@which ros2 > /dev/null 2>&1 || (echo "Error: ROS 2 not found. Install ROS 2 first." && exit 1)
	@echo "Note: Update the serial port to match your board"
	ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6

.PHONY: check
check: ## Check west installation and environment
	@echo "Checking Zephyr environment..."
	@echo ""
	@echo "=== Paths ==="
	@echo -n "Virtual env: "
	@test -d $(ZEPHYR_VENV) && echo "✓ $(ZEPHYR_VENV)" || echo "✗ MISSING"
	@echo -n "West in venv: "
	@test -f $(ZEPHYR_VENV)/bin/west && echo "✓ OK" || echo "✗ MISSING"
	@echo -n "Zephyr workspace: "
	@test -d $(ZEPHYR_WS) && echo "✓ $(ZEPHYR_WS)" || echo "✗ MISSING"
	@echo ""
	@echo "=== Build Tools ==="
	@echo -n "Ninja (venv): "
	@. $(ZEPHYR_VENV)/bin/activate && which ninja > /dev/null 2>&1 && echo "✓ $$(which ninja)" || echo "✗ MISSING"
	@echo -n "CMake: "
	@which cmake > /dev/null 2>&1 && echo "✓ $$(cmake --version | head -1)" || echo "✗ MISSING"
	@echo -n "DTC: "
	@which dtc > /dev/null 2>&1 && echo "✓ $$(which dtc)" || echo "✗ MISSING"
	@echo -n "gperf: "
	@which gperf > /dev/null 2>&1 && echo "✓ $$(which gperf)" || echo "✗ MISSING"
	@echo ""
	@echo "=== Python Packages ==="
	@echo -n "west: "
	@. $(ZEPHYR_VENV)/bin/activate && python3 -c "import west" 2>/dev/null && echo "✓ OK" || echo "✗ MISSING"
	@echo -n "jsonschema: "
	@. $(ZEPHYR_VENV)/bin/activate && python3 -c "import jsonschema" 2>/dev/null && echo "✓ OK" || echo "✗ MISSING"
	@echo -n "pyelftools: "
	@. $(ZEPHYR_VENV)/bin/activate && python3 -c "import elftools" 2>/dev/null && echo "✓ OK" || echo "✗ MISSING"
	@echo -n "colcon: "
	@. $(ZEPHYR_VENV)/bin/activate && python3 -c "import colcon_core" 2>/dev/null && echo "✓ OK" || echo "✗ MISSING"
	@echo ""
	@echo "=== Project Status ==="
	@echo -n "Build directory: "
	@test -d $(BUILD_DIR) && echo "✓ exists" || echo "✗ not found (run 'make build' first)"
	@echo -n "micro-ROS library: "
	@test -f $(ZEPHYR_WS)/modules/micro_ros_zephyr_module/modules/libmicroros/libmicroros.a && echo "✓ built" || echo "✗ not built (run 'make build-microros' first)"

.PHONY: install-deps
install-deps: ## Install required dependencies (Linux only)
	@echo "Installing system dependencies (Linux)..."
	@which apt > /dev/null 2>&1 || (echo "Error: This target is for Debian/Ubuntu only" && exit 1)
	sudo apt update
	sudo apt install -y git cmake ninja-build gperf ccache dfu-util \
		device-tree-compiler wget python3 python3-pip python3-setuptools \
		python3-wheel xz-utils file make gcc gcc-multilib g++-multilib \
		libsdl2-dev libmagic1
	@echo ""
	@echo "Installing Python requirements..."
	. $(ZEPHYR_VENV)/bin/activate && pip install --upgrade pip
	. $(ZEPHYR_VENV)/bin/activate && pip install west
	. $(ZEPHYR_VENV)/bin/activate && pip install catkin_pkg lark-parser 'empy<4.0' colcon-common-extensions
	@echo ""
	@echo "✓ Dependencies installed successfully"
	@echo "Note: You still need to install Zephyr Python requirements:"
	@echo "  source $(ZEPHYR_VENV)/bin/activate"
	@echo "  pip install -r $(ZEPHYR_WS)/zephyr/scripts/requirements.txt"

.PHONY: rebuild
rebuild: clean build ## Clean and rebuild

.PHONY: verify-build
verify-build: ## Build and verify project compiles without errors
	@echo "╔════════════════════════════════════════╗"
	@echo "║  Verifying Build for $(BOARD)         ║"
	@echo "╚════════════════════════════════════════╝"
	@echo ""
	@echo "→ Building project..."
	$(WEST) build -b $(BOARD) -p
	@echo ""
	@echo "✓ Build successful!"
	@echo ""

.PHONY: verify-memory
verify-memory: ## Show memory usage verification (RAM/ROM)
	@echo "╔════════════════════════════════════════╗"
	@echo "║  Memory Usage Report                   ║"
	@echo "╚════════════════════════════════════════╝"
	@echo ""
	@echo "→ RAM Usage:"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@$(WEST) build -t ram_report | grep -E "(Root|HEAP|STACK|SRAM)" || true
	@echo ""
	@echo "→ ROM Usage:"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@$(WEST) build -t rom_report | grep -E "(Root|Flash|ROM)" || true
	@echo ""
	@echo "→ Key Memory Sections:"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@$(WEST) build -t ram_report | grep -E "kheap|z_main_stack|CONFIG" || true
	@echo ""

.PHONY: verify-microros
verify-microros: ## Verify micro-ROS configuration and library
	@echo "╔════════════════════════════════════════╗"
	@echo "║  micro-ROS Configuration Check         ║"
	@echo "╚════════════════════════════════════════╝"
	@echo ""
	@echo "→ Checking micro-ROS library..."
	@test -f $(ZEPHYR_WS)/modules/micro_ros_zephyr_module/modules/libmicroros/libmicroros.a && \
		echo "  ✓ libmicroros.a found" || \
		echo "  ✗ libmicroros.a not found (build micro-ROS first)"
	@echo ""
	@echo "→ Checking prj.conf micro-ROS settings..."
	@grep -E "CONFIG_MICROROS|CONFIG_HEAP|CONFIG_MAIN_STACK|CONFIG_UART_CONSOLE" prj.conf || true
	@echo ""
	@echo "→ Checking DeviceTree overlay..."
	@test -f app.overlay && echo "  ✓ app.overlay exists" || echo "  ✗ app.overlay missing"
	@test -f app.overlay && grep -E "usart1|115200" app.overlay || true
	@echo ""

.PHONY: verify-all
verify-all: verify-build verify-memory verify-microros ## Run all verification checks
	@echo ""
	@echo "╔════════════════════════════════════════╗"
	@echo "║  ✓ All Verifications Complete          ║"
	@echo "╚════════════════════════════════════════╝"
	@echo ""
	@echo "Next steps:"
	@echo "  1. Flash to board:  make flash"
	@echo "  2. Start Agent:     make uros-agent-docker"
	@echo "  3. Monitor output:  make monitor"
	@echo ""

.PHONY: uros-agent-docker
uros-agent-docker: ## Run micro-ROS agent via Docker (recommended)
	@echo "╔════════════════════════════════════════╗"
	@echo "║  Starting micro-ROS Agent (Docker)     ║"
	@echo "╚════════════════════════════════════════╝"
	@echo ""
	@echo "Using serial port: /dev/ttyACM0"
	@echo "Verbose logging: enabled (-v6)"
	@echo ""
	@echo "Note: If connection fails, try:"
	@echo "  - Check port: ls -l /dev/ttyACM*"
	@echo "  - Fix permissions: sudo chmod 666 /dev/ttyACM0"
	@echo "  - Try /dev/ttyACM1 if ACM0 doesn't work"
	@echo ""
	docker run -it --rm -v /dev:/dev --privileged --net=host \
		microros/micro-ros-agent:kilted serial --dev /dev/ttyACM0 -v6

.PHONY: monitor
monitor: ## Monitor serial output (requires screen)
	@echo "Monitoring serial output on /dev/ttyACM0..."
	@echo "Press Ctrl-A, then K to exit"
	@which screen > /dev/null 2>&1 || (echo "Error: 'screen' not found. Install with: brew install screen" && exit 1)
	screen /dev/ttyACM0 115200

.PHONY: all
all: build ## Default target: build the project
