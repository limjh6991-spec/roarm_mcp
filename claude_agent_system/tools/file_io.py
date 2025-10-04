"""
A tool for performing file system operations (read and write).
Each function is designed to be called by the orchestrator based on
the agent's requests.
"""
import os

def read(filepath: str) -> str:
    """
    Reads the content of a file.
    Args:
        filepath (str): The path to the file to be read.
    Returns:
        str: The content of the file, or an error message if it fails.
    """
    print(f"  - Executing tool 'file_io_read' on '{filepath}'")
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception as e:
        return f"Error reading file: {e}"

def write(filepath: str, content: str) -> str:
    """
    Writes content to a file. Creates directories if they don't exist.
    Args:
        filepath (str): The path to the file to be written.
        content (str): The content to write to the file.
    Returns:
        str: A success message or an error message.
    """
    print(f"  - Executing tool 'file_io_write' on '{filepath}'")
    try:
        # Ensure the directory exists if a path is provided
        directory = os.path.dirname(filepath)
        if directory:
            os.makedirs(directory, exist_ok=True)

        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        return f"File '{filepath}' written successfully."
    except Exception as e:
        return f"Error writing file: {e}"