"""
A mock implementation of the claude_agent_sdk for demonstration purposes.
This module simulates the behavior of the real SDK, allowing the system
to be developed and tested without a live connection to the Claude API.
"""
import json
import re

class Agent:
    """
    A mock Agent class that simulates the behavior of the Claude Agent SDK.
    It takes a user task and, based on simple keyword matching, returns a
    structured "tool call" that the orchestrator can execute.
    """
    def __init__(self, model: str, system_prompt: str = ""):
        """
        Initializes the mock agent.
        Args:
            model (str): The name of the model to be used (e.g., "claude-sonnet-4-5").
                         This is stored but not used in the mock logic.
            system_prompt (str): A system prompt to guide the agent's behavior.
        """
        self.model = model
        self.system_prompt = system_prompt
        print(f"🤖 Mock Agent initialized with model: {self.model}")
        print(f"   System Prompt: {self.system_prompt}")

    def _extract_paths(self, task_str):
        """Extracts file paths from a task string using regex."""
        paths = re.findall(r"\'(.*?)\'", task_str)
        if len(paths) == 2:
            return paths[0], paths[1]
        return None, None

    def run(self, task: str, conversation: list = None) -> dict:
        """
        Simulates running the agent to get the next action for a given task.
        The logic is a simplified simulation of an LLM's tool-use reasoning.

        Args:
            task (str): The user's high-level objective.
            conversation (list, optional): A history of previous tool calls and
                                           observations. Defaults to None.

        Returns:
            dict: A dictionary representing the tool to be called, including
                  the tool name and its parameters. Returns a 'task_complete'
                  signal if no more actions are needed.
        """
        print(f"\n🧠 Agent thinking... Task: '{task}'")

        # More robust logic that simulates state awareness
        if "read" in task.lower() and "write" in task.lower():
            input_path, output_path = self._extract_paths(task)
            if not input_path or not output_path:
                return {"tool_name": "task_complete", "parameters": {"error": "Could not parse file paths."}}

            # Check conversation history to decide the next step
            if not conversation:
                # 1. If conversation is empty, the first step is to read.
                return {"tool_name": "file_io_read", "parameters": {"filepath": input_path}}

            last_action = conversation[-1].get("action", {}).get("tool_name")
            last_observation = conversation[-1].get("observation", "")

            if last_action == "file_io_read" and "error" not in last_observation.lower():
                # 2. If reading succeeded, the next step is to write.
                return {
                    "tool_name": "file_io_write",
                    "parameters": {"filepath": output_path, "content": last_observation}
                }

            if last_action == "file_io_write" and "error" not in last_observation.lower():
                # 3. If writing succeeded, the task is complete.
                return {"tool_name": "task_complete", "parameters": {"reason": "File successfully written."}}

            # Handle cases where a previous step failed
            if "error" in last_observation.lower():
                return {"tool_name": "task_complete", "parameters": {"error": f"Previous step failed: {last_observation}"}}

        elif "list files" in task.lower() or "ls" in task.lower():
            return {"tool_name": "shell_run", "parameters": {"command": "ls -l"}}

        # Default fallback to complete the task if logic doesn't know what to do.
        return {"tool_name": "task_complete", "parameters": {"reason": "Finished sequence."}}