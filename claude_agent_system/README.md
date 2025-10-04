# Claude Agent System (A Simulated Implementation)

This project is a demonstration of a production-level automation system architecture using a powerful Large Language Model (LLM) like Claude Sonnet 4.5, orchestrated by an agentic framework similar to the `claude-agent-sdk`.

Since this is a simulated environment, this implementation provides a **mock SDK** that faithfully reproduces the core architectural concepts of an autonomous agent system. This allows for a clear, functional, and well-documented starting point that can be adapted to the real SDK.

## 1. System Architecture Overview

The system is designed around a central **Orchestrator** that acts as the "nervous system" for the AI agent. It connects the agent's "brain" (the LLM) to its "hands" (a set of tools it can use to interact with the environment).

### Core Components

1.  **Agent (The "Brain")**
    -   **File:** `src/mock_claude_sdk.py`
    -   **Role:** Simulates the Claude Sonnet 4.5 model. Given a high-level task and a history of past actions, it decides the next best action to take (e.g., "read a file," "run a shell command"). In this mock, the logic is simplified to keyword matching, but in a real system, this would be a powerful LLM.

2.  **Orchestrator (The "Nervous System")**
    -   **File:** `src/orchestrator.py`
    -   **Role:** Manages the main feedback loop (often called a ReAct loop: Reason + Act).
        1.  It sends the user's task to the Agent.
        2.  It receives an "action" (a tool call) from the Agent.
        3.  It executes the action using the appropriate tool.
        4.  It takes the result of that action (an "observation") and sends it back to the Agent.
        5.  This loop continues until the Agent determines the task is complete.

3.  **Tools (The "Hands")**
    -   **Directory:** `tools/`
    -   **Role:** These are modular Python scripts that give the agent its abilities. The Orchestrator calls these tools based on the Agent's requests.
    -   **Implemented Tools:**
        -   `file_io.py`: Provides functions to read and write files.
        -   `shell.py`: Provides a function to execute shell commands.

## 2. How to Run the Demonstration

This project includes a simple demonstration where the agent is tasked to **read the contents of `input.txt` and write them into a new file, `output.txt`**.

### Prerequisites

- Python 3.x

### Running the Script

1.  Navigate to the `claude_agent_system` directory in your terminal.
2.  Run the main executable:
    ```bash
    python main.py
    ```
3.  **Observe the Output:** The script will print a step-by-step log of the agent's "thought process," the tools it executes, and the observations it makes.
4.  **Verify the Result:** After the script finishes, a new file named `output.txt` will be created in the `claude_agent_system` directory. Its contents should be identical to `input.txt`.

    ```bash
    # You can verify this with a command like:
    cat output.txt
    ```

## 3. Project Structure

```
claude_agent_system/
├── docs/
│   └── SYSTEM_ARCHITECTURE.md  # Detailed documentation of the architecture
├── src/
│   ├── mock_claude_sdk.py      # The mock "Agent" (the brain)
│   └── orchestrator.py         # The "Orchestrator" (the nervous system)
├── tools/
│   ├── file_io.py              # Tool for file operations
│   └── shell.py                # Tool for running shell commands
├── input.txt                   # Sample input file for the demo
├── main.py                     # Main executable to run the demonstration
└── README.md                   # This file
```

This structure separates concerns, making the system modular and extensible. To add new abilities to the agent, you simply need to create a new tool in the `tools/` directory and register it in the `orchestrator.py`.