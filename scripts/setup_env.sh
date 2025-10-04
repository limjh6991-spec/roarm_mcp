#!/bin/bash
#
# A script to set up the Python development environment for the RoArm MCP project.
#
# This script performs the following steps:
#   1. Determines the project's root directory.
#   2. Creates a Python virtual environment named `.venv` in the project root
#      if it does not already exist.
#   3. Activates the virtual environment.
#   4. Upgrades pip, setuptools, and wheel to the latest versions.
#   5. Installs all required Python packages from the `requirements.txt` file.
#   6. Checks if the current user is in the `dialout` group, which is
#      often necessary for serial port communication with hardware, and
#      prints a warning if they are not.
#
# Usage:
#   From the project root directory, run:
#   bash scripts/setup_env.sh
#

# --- Setup ---
# Get the project root directory (the parent of the 'scripts' directory)
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PROJECT_ROOT}/.venv"
LOG_DIR="${PROJECT_ROOT}/daily_log"
TODAY=$(date +"%Y%m%d")
LOG_FILE="${LOG_DIR}/${TODAY}.log"

# --- Functions ---

# Function to add a message to the daily log file.
log_message() {
    local message="$1"
    # Ensure log directory exists before trying to write to the file
    if [ ! -d "$LOG_DIR" ]; then
        mkdir -p "$LOG_DIR"
    fi
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] [SETUP] $message" >> "$LOG_FILE"
    echo "[INFO] $message"
}

# --- Main Execution ---

echo "=== Starting RoArm MCP Environment Setup ==="
log_message "Environment setup script started."

# 1. Create the Python virtual environment if it doesn't exist.
if [ ! -d "$VENV_DIR" ]; then
    log_message "Creating Python virtual environment at: $VENV_DIR"
    python3 -m venv "$VENV_DIR"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to create virtual environment. Please ensure python3 and the 'venv' module are installed."
        exit 1
    fi
    log_message "Virtual environment created successfully."
else
    log_message "Virtual environment already exists at: $VENV_DIR"
fi

# 2. Activate the virtual environment.
log_message "Activating the virtual environment..."
source "$VENV_DIR/bin/activate"
if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to activate virtual environment."
    exit 1
fi

# 3. Install/upgrade required packages.
log_message "Upgrading pip, setuptools, and wheel..."
pip install --upgrade pip setuptools wheel
log_message "Installing project dependencies from requirements.txt..."
pip install -r "${PROJECT_ROOT}/requirements.txt"
if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to install dependencies from requirements.txt."
    log_message "Error: Failed to install dependencies."
    exit 1
fi
log_message "Python packages installed successfully."

# 4. Check for 'dialout' group membership (for hardware communication).
if ! groups | grep -q "dialout"; then
    echo ""
    echo "[WARNING] User '$USER' is not in the 'dialout' group."
    echo "          This permission is required for serial communication with the physical robot arm."
    echo "          To add yourself to the group, run the following command:"
    echo "          sudo usermod -a -G dialout $USER"
    echo "          You will need to log out and log back in for the change to take effect."
    log_message "Warning: User is not in the 'dialout' group, which is needed for hardware access."
fi

echo ""
echo "=== RoArm MCP Environment Setup Complete ==="
log_message "Environment setup finished successfully."
echo "To activate the environment in your shell, run:"
echo "  source ${VENV_DIR}/bin/activate"
echo "To run the MCP server, you can use the helper script:"
echo "  bash ${PROJECT_ROOT}/scripts/run_mcp_server.sh"