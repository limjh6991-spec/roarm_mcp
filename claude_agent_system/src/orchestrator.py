"""
The Orchestrator is the core component that manages the agent's lifecycle.
It connects the agent's "brain" (the SDK) with its "hands" (the tools),
running the feedback loop until a task is completed.
"""
from src.mock_claude_sdk import Agent
from tools import file_io, shell

class Orchestrator:
    """
    Manages the agent feedback loop (ReAct loop).
    1. Takes a high-level task.
    2. Gets the next action from the Agent.
    3. Executes that action using the available tools.
    4. Feeds the result (observation) back to the Agent.
    5. Repeats until the task is marked as complete.
    """
    def __init__(self, model: str = "claude-sonnet-4-5"):
        self.system_prompt = (
            "You are a helpful and autonomous AI agent. Your goal is to complete "
            "the user's task by using the available tools. Think step-by-step "
            "and decide which tool to use to make progress. When the task is fully "
            "complete, use the 'task_complete' tool."
        )
        self.agent = Agent(model=model, system_prompt=self.system_prompt)

        # The tool manager maps tool names from the LLM to actual Python functions.
        self.tools = {
            "file_io_read": file_io.read,
            "file_io_write": file_io.write,
            "shell_run": shell.run
        }
        print("🛠️ Orchestrator initialized with the following tools:", ", ".join(self.tools.keys()))

    def run_task(self, task: str):
        """
        Runs the main agent loop to accomplish a given task.
        Args:
            task (str): The high-level objective for the agent.
        """
        print(f"\n🚀 Starting task: {task}")
        conversation = []
        max_turns = 5 # Safety break to prevent infinite loops

        for turn in range(max_turns):
            # 1. Get the next action from the agent
            action = self.agent.run(task, conversation)
            tool_name = action.get("tool_name")
            parameters = action.get("parameters", {})

            # 2. Check for task completion
            if tool_name == "task_complete":
                print("✅ Task marked as complete by the agent.")
                break

            if tool_name not in self.tools:
                print(f"Error: Agent requested unknown tool '{tool_name}'.")
                observation = f"Error: Unknown tool '{tool_name}'"
            else:
                # 3. Execute the chosen tool
                print(f"▶️ Executing tool: {tool_name} with params: {parameters}")
                tool_function = self.tools[tool_name]
                try:
                    observation = tool_function(**parameters)
                    print(f"↪️ Observation: {observation}")
                except Exception as e:
                    observation = f"Error executing tool {tool_name}: {e}"
                    print(f"   Error: {observation}")

            # 4. Record the turn and feed the observation back
            conversation.append({
                "action": action,
                "observation": observation
            })
        else:
            print("⚠️ Reached max turns without completing the task.")

        print("\n🏁 Task execution finished.")