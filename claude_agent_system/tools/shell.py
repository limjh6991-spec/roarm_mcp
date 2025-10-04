"""
A tool for executing shell commands.
"""
import subprocess

def run(command: str) -> str:
    """
    Executes a shell command and returns its output.
    Args:
        command (str): The shell command to execute.
    Returns:
        str: The stdout and stderr from the command, or an error message.
    """
    print(f"  - Executing tool 'shell_run' with command: '{command}'")
    if not command:
        return "Error: command cannot be empty."
    try:
        result = subprocess.run(
            command,
            shell=True,
            check=True,
            capture_output=True,
            text=True,
            encoding='utf-8'
        )
        output = result.stdout
        if result.stderr:
            output += "\nSTDERR:\n" + result.stderr
        return output
    except subprocess.CalledProcessError as e:
        return f"Error executing command: {e}\nSTDOUT:\n{e.stdout}\nSTDERR:\n{e.stderr}"
    except Exception as e:
        return f"An unexpected error occurred: {e}"