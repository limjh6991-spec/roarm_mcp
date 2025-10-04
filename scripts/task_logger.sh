#!/bin/bash
#
# A utility script for running and logging shell commands.
#
# This script provides a structured way to execute commands while logging their
# initiation, execution, and result (success or failure) to a daily log file
# located in the `daily_log` directory. It is designed to create a traceable
# record of tasks performed within the project.
#
# Features:
#   - Automatically creates a `daily_log` directory and a new log file for each day.
#   - Provides a `run_and_log` function to wrap command execution with logging.
#   - Prints clear information about the task being performed to the console.
#   - Records the exit code of the command in the log.
#
# Usage:
#   To use this script's functions in another script, source it:
#   source scripts/task_logger.sh
#
#   Then, use the run_and_log function:
#   run_and_log "My Task" "A description of my task" "ls -l"
#
#   The script can also be run directly to see a help message or an example.
#

# --- Functions ---

# Prints a formatted header for a task to the console.
# Globals:
#   None
# Arguments:
#   $1 - The task name.
#   $2 - A description of the task.
print_task_info() {
    echo "======================================================"
    echo "Task: $1"
    echo "Time: $(date +"%Y-%m-%d %H:%M:%S")"
    echo "Description: $2"
    echo "======================================================"
}

# Creates a log entry in the daily log file.
# Globals:
#   None
# Arguments:
#   $1 - The task name.
#   $2 - The log message/description.
create_log_entry() {
    local task="$1"
    local description="$2"
    
    # Define project directories
    local project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    local log_dir="${project_root}/daily_log"
    local today=$(date +"%Y%m%d")
    local log_file="${log_dir}/${today}.log"
    
    # Ensure log directory and file exist
    mkdir -p "$log_dir"
    if [ ! -f "$log_file" ]; then
        # Create a header for a new log file
        echo "# RoArm MCP Task Log - $(date +"%Y-%m-%d")" > "$log_file"
    fi
    
    # Append the log message
    echo "[$(date +"%H:%M:%S")] [$task] $description" >> "$log_file"
}

# Executes a command and logs the process and result.
# Globals:
#   None
# Arguments:
#   $1 - The task name.
#   $2 - A description of the task.
#   $3 - The shell command to execute.
# Returns:
#   The exit code of the executed command.
run_and_log() {
    local task="$1"
    local description="$2"
    local command="$3"
    
    print_task_info "$task" "$description"
    create_log_entry "$task" "Starting: $description"
    
    echo "[EXEC] $command"
    
    # Execute the command
    eval "$command"
    local exit_code=$?
    
    # Log the result
    if [ $exit_code -eq 0 ]; then
        create_log_entry "$task" "Execution successful."
        echo "[RESULT] Success"
    else
        create_log_entry "$task" "Execution failed with exit code: $exit_code"
        echo "[RESULT] Failure (Exit Code: $exit_code)"
    fi
    
    return $exit_code
}

# --- Script Main Body ---

# Display a help message if requested.
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: $0 [command]"
    echo ""
    echo "A utility for running and logging tasks."
    echo ""
    echo "Commands:"
    echo "  --help, -h    Display this help message."
    echo "  --example     Run a pre-defined example task to demonstrate logging."
    echo ""
    echo "To use in your own scripts, source this file and call 'run_and_log \"Task\" \"Description\" \"command\"'."
    exit 0
fi

# Run an example task if requested.
if [ "$1" == "--example" ]; then
    run_and_log "Example Task" "This is an example command to test the logging system." "echo 'Example command executed successfully.'"
    exit 0
fi

# Default message if no arguments are provided.
if [ -z "$1" ]; then
    echo "RoArm MCP Task Logger"
    echo "Run with '--help' for usage information."
fi