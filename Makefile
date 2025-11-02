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
WEST := unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION CMAKE_PREFIX_PATH AMENT_PREFIX_PATH && \
        . $(ZEPHYR_VENV)/bin/activate && \
        export ZEPHYR_BASE=$(ZEPHYR_BASE) && \
        export USER_CACHE_DIR=$(USER_CACHE_DIR) && \
        west

.PHONY: help
help: ## Show this help message
	@echo "Zephyr Project Makefile - disco_l475_iot1"
	@echo ""
	@echo "Usage: ZEPHYR_VENV=/path/to/venv make [target]"
	@echo ""
	@echo "Available targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

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
	$(WEST) flash

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
	@. $(ZEPHYR_VENV)/bin/activate && which ninja > /dev/null 2>&1 && echo "✓ $$(which ninja)" || echo "✗ MISSING (run: pip install -r $(ZEPHYR_WS)/zephyr/scripts/requirements.txt)"
	@echo -n "CMake: "
	@which cmake > /dev/null 2>&1 && echo "✓ $$(cmake --version | head -1)" || echo "✗ MISSING (install: brew install cmake)"
	@echo -n "DTC: "
	@which dtc > /dev/null 2>&1 && echo "✓ $$(which dtc)" || echo "✗ MISSING (install: brew install dtc)"
	@echo -n "gperf: "
	@which gperf > /dev/null 2>&1 && echo "✓ $$(which gperf)" || echo "✗ MISSING (install: brew install gperf)"
	@echo ""
	@echo "=== Python Packages ==="
	@echo -n "jsonschema: "
	@. $(ZEPHYR_VENV)/bin/activate && python3 -c "import jsonschema" 2>/dev/null && echo "✓ OK" || echo "✗ MISSING"
	@echo -n "pyelftools: "
	@. $(ZEPHYR_VENV)/bin/activate && python3 -c "import elftools" 2>/dev/null && echo "✓ OK" || echo "✗ MISSING"
	@echo ""
	@echo "=== Project Status ==="
	@echo -n "Build directory: "
	@test -d $(BUILD_DIR) && echo "✓ exists" || echo "✗ not found (run 'make build' first)"
	@echo ""
	@echo "If any dependencies are missing, run: make install-deps

.PHONY: install-deps
install-deps: ## Install required dependencies
	@echo "Installing system dependencies (macOS)..."
	@which brew > /dev/null 2>&1 || (echo "Error: Homebrew not found. Install from https://brew.sh" && exit 1)
	brew install cmake dtc gperf
	@echo ""
	@echo "Installing Python requirements from Zephyr..."
	. $(ZEPHYR_VENV)/bin/activate && pip install -r $(ZEPHYR_WS)/zephyr/scripts/requirements.txt
	@echo ""
	@echo "Installing micro-ROS dependencies..."
	. $(ZEPHYR_VENV)/bin/activate && pip install catkin_pkg lark-parser 'empy<4.0' colcon-common-extensions
	@echo ""
	@echo "✓ Dependencies installed successfully"
	@echo "Run 'make check' to verify installation"

.PHONY: rebuild
rebuild: clean build ## Clean and rebuild

.PHONY: all
all: build ## Default target: build the project
