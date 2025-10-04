# System Architecture: Autonomous Agent Framework

This document provides a detailed description of the architecture used in this simulated agentic system. The design is based on principles that power modern autonomous AI agents, focusing on modularity, extensibility, and a clear separation of concerns.

## 1. Core Philosophy: The ReAct Loop

The entire system is built upon the **ReAct (Reason + Act)** paradigm. This is a powerful framework where a Large Language Model (LLM) is prompted not just to produce an answer, but to **reason** about a problem, choose an **action** to take, and then learn from the **observation** of that action's outcome.

Our implementation simulates this loop within the `Orchestrator`:

1.  **Reason:** The `Agent` (our mock LLM) receives the high-level task and the conversation history. It "reasons" about what step to take next to make progress. Its output is a structured tool call.
2.  **Act:** The `Orchestrator` receives this tool call and executes it by invoking the corresponding function from the `tools/` directory. This is the "Act" phase, where the agent interacts with its environment (e.g., the file system or the shell).
3.  **Observe:** The `Orchestrator` captures the return value of the tool (e.g., the contents of a file, the output of a command). This is the "Observation."
4.  **Feedback:** The Observation is appended to the conversation history and fed back to the `Agent` on the next turn. This allows the agent to adjust its plan based on the results of its previous actions.

This cycle repeats until the agent concludes that the task is complete.

## 2. Component Deep Dive

### The Agent (`src/mock_claude_sdk.py`)

-   **Purpose:** To act as the decision-making "brain" of the system.
-   **Functionality:** In a real-world scenario, this component would make a call to a powerful LLM like Claude Sonnet 4.5. The LLM would be given a carefully crafted system prompt, the user's task, the list of available tools, and the conversation history. Its job is to return a JSON object specifying the tool to use and the parameters to use with it.
-   **Our Mock Implementation:** Our `Agent.run()` method uses simple string matching to simulate this decision process. For the task "Read 'input.txt' and write to 'output.txt'", it first sees the word "read" and issues a `file_io_read` call. After that action is observed, on the next turn, it sees that the read is complete and issues the `file_io_write` call. This mimics the step-by-step reasoning of a real agent.

### The Orchestrator (`src/orchestrator.py`)

-   **Purpose:** To be the central controller that runs the ReAct loop and connects all other components.
-   **Key Responsibilities:**
    -   **Tool Management:** It maintains a dictionary (a "tool registry") that maps the string names of tools (e.g., `"file_io_read"`) to the actual, callable Python functions. This makes the system easily extensible—adding a new tool is as simple as adding a new entry to this dictionary.
    -   **State Management:** It manages the `conversation` history, which is the list of all previous actions and their corresponding observations. This history is crucial as it provides the agent with the context it needs to make informed decisions.
    -   **Execution Engine:** It is responsible for safely calling the tool functions with the parameters provided by the agent. It also handles basic error trapping, so if a tool fails, the error message can be passed back to the agent as an observation.

### The Tools (`tools/`)

-   **Purpose:** To provide the agent with a concrete set of capabilities, or "affordances," for interacting with its environment.
-   **Design Principles:**
    -   **Modular and Atomic:** Each tool should do one thing well. For example, `file_io.py` handles file operations, and `shell.py` handles command execution. This makes them easy to test and maintain.
    -   **Clear I/O:** Each tool function has a clearly defined set of parameters and a predictable return type (usually a string). This consistency is vital, as the agent's output must map cleanly to the function's signature, and the function's output must be a useful observation for the agent.
    -   **Extensibility:** The power of the agent is directly proportional to the quality and breadth of its tools. The system is designed so that a developer can easily add a new file (e.g., `tools/api_caller.py`) with new functions to grant the agent new skills, such as interacting with a database or a web API.

## 3. How to Extend the System

To add a new capability to the agent (e.g., the ability to search the web):

1.  **Create the Tool:** Create a new file, `tools/web_search.py`. Inside, define a function, e.g., `def search(query: str) -> str:`. This function would contain the logic to call a search engine's API.
2.  **Register the Tool:** In `src/orchestrator.py`, import the new function and add it to the `self.tools` dictionary:
    ```python
    from tools import file_io, shell, web_search

    # ... inside Orchestrator.__init__ ...
    self.tools = {
        "file_io_read": file_io.read,
        "file_io_write": file_io.write,
        "shell_run": shell.run,
        "web_search": web_search.search  # Add the new tool
    }
    ```
3.  **Update the Agent's Logic (or Prompt):** If using a real LLM, you would update the system prompt to inform the agent about its new `web_search` tool and how to use it. In our mock SDK, you would add a new `elif` condition to the `run` method to handle tasks involving web searches.

This modular design ensures that the system can grow in complexity and capability without requiring a rewrite of the core logic.