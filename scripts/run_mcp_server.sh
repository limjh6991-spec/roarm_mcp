#!/bin/bash
#
# A convenience script to set up the environment and run the MCP server.
#
# This script performs the following actions:
#   1. Determines the project's root directory.
#   2. Creates a log directory (`daily_log`) if it doesn't exist.
#   3. Creates a daily log file with the current date.
#   4. Checks for and activates the Python virtual environment (`.venv`).
#   5. Starts the main MCP server using the `examples/run_server.py` script.
#   6. Logs key actions to the daily log file.
#
# Usage:
#   From the project root directory, simply run:
#   ./scripts/run_mcp_server.sh [server_options]
#
# Example:
#   ./scripts/run_mcp_server.sh --robot-type franka --headless
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
    local timestamp=$(date +"%Y-%m-%d %H:%M:%S")
    echo "[$timestamp] $message" >> "$LOG_FILE"
    echo "[LOG] $message"
}

# --- Main Execution ---

# 1. Ensure the log directory exists.
if [ ! -d "$LOG_DIR" ]; then
    echo "[INFO] Creating log directory: $LOG_DIR"
    mkdir -p "$LOG_DIR"
fi

# 2. Create a new log file for the day if it doesn't exist.
if [ ! -f "$LOG_FILE" ]; then
    echo "[INFO] Creating new log file: $LOG_FILE"
    cat > "$LOG_FILE" << EOF
# RoArm MCP Task Log
# Date: $(date +"%Y-%m-%d")
# Initiated by: run_mcp_server.sh
#
EOF
fi

# 3. Check and activate the Python virtual environment if not already active.
if [[ -z "$VIRTUAL_ENV" ]]; then
    if [ -f "$VENV_DIR/bin/activate" ]; then
        echo "[INFO] Activating Python virtual environment..."
        source "$VENV_DIR/bin/activate"
        log_message "Virtual environment activated."
    else
        echo "[ERROR] Virtual environment not found at $VENV_DIR."
        echo "[ERROR] Please run 'scripts/setup_env.sh' first."
        exit 1
    fi
fi

# 4. Run the main MCP server.
# Any arguments passed to this script (e.g., --headless) will be forwarded to the server script.
echo "[INFO] Starting the MCP server..."
cd "$PROJECT_ROOT"
log_message "Executing server start command: python -m examples.run_server $@"
python -m examples.run_server "$@"

# 5. Log the server shutdown.
log_message "Server process has been terminated."
echo "[INFO] Server has shut down."