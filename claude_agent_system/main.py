"""
Main executable for the Claude Agent System.
This script initializes the Orchestrator and assigns it a task to demonstrate
the end-to-end functionality of the autonomous agent.
"""
import sys
import os

# Ensure the 'claude_agent_system' directory is in the Python path
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from src.orchestrator import Orchestrator

def main():
    """
    Entry point of the application.
    - Creates an instance of the Orchestrator.
    - Defines a high-level task for the agent.
    - Runs the task and lets the agent work autonomously.
    """
    print("=============================================")
    print("=      Claude Agent System Demonstration    =")
    print("=============================================")

    # The Orchestrator is the main controller for the agent.
    orchestrator = Orchestrator(model="claude-sonnet-4-5")

    # Make paths absolute so the script can be run from any directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_path = os.path.join(script_dir, "input.txt")
    output_path = os.path.join(script_dir, "output.txt")

    # Define the high-level task for the agent with absolute paths.
    task = f"Read the content of '{input_path}' and write it to '{output_path}'."

    # Run the task. The orchestrator will now manage the agent loop.
    orchestrator.run_task(task)

    print("\n=============================================")
    print("=            Demonstration Finished         =")
    print("=============================================")
    print("You can now inspect 'output.txt' to verify the result.")


if __name__ == "__main__":
    main()