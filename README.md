# RoArm MCP: Robot Arm Model Context Protocol

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**RoArm MCP** is a robust framework for robot arm reinforcement learning, built on **NVIDIA Isaac Sim 5.0** and the **Model Context Protocol (MCP)**. It provides a client-server architecture that decouples the reinforcement learning agent from the physics-based simulation environment, enabling flexible and scalable research.

## Core Concepts

This project is built around a few key ideas:

*   **NVIDIA Isaac Sim**: Provides the state-of-the-art, physics-based simulation environment for training and testing the robot arms.
*   **Model Context Protocol (MCP)**: A WebSocket-based protocol that standardizes communication between the learning agent (client) and the simulation environment (server).
*   **Client-Server Architecture**: The simulation runs on a server, managed by `MCPServer`. RL agents, or other controllers, run as clients (`MCPClient`) that can connect to the server from the same machine or remotely.
*   **Gymnasium Interface**: The `mcp.client.MCPClient` class implements the standard `gymnasium.Env` interface, making it easy to integrate with popular RL libraries like Stable-Baselines3.

## Key Features

*   **Full Isaac Sim 5.0 Support**: Utilizes the latest PhysX 5 tensors API for high-performance simulation.
*   **Flexible Robot Support**: Comes with pre-configured support for the UR10 and Franka Emika Panda robot arms.
*   **Standardized Communication**: Implements the Model Context Protocol for clear and robust client-server interaction.
*   **Advanced Sensor Simulation**: Includes high-performance modules for simulating RGB, Depth, and synchronized RGB-D cameras using Isaac Sim's Replicator framework.
*   **Gymnasium Compatible**: The client acts as a standard `gymnasium.Env`, ready for use with modern RL algorithms.

## Project Structure

The repository is organized into several key directories:

```
roarm_mcp/
├── mcp/                  # Core MCP server, client, and protocol definitions.
├── envs/                 # Gymnasium-style environments for different control tasks.
├── robot/                # Robot-specific logic, including arm models and controllers.
├── sensors/              # Modules for simulating RGB, Depth, and RGB-D cameras.
├── isaac_sim/            # Low-level integration with the Isaac Sim simulator.
├── isaac_sim_integration/ # Solutions and tests for Isaac Sim, including the PhysX Tensors fix.
├── examples/             # Example scripts for running servers and clients.
├── scripts/              # Helper shell scripts for setup and execution.
└── config/               # Configuration files (currently unused, for future expansion).
```

## Prerequisites

*   **Python 3.10+**
*   **NVIDIA Isaac Sim 5.0** or later. ([Installation Guide](https://developer.nvidia.com/isaac-sim))
*   An **NVIDIA GPU** with CUDA support (RTX series recommended).
*   All other Python dependencies are listed in `requirements.txt`.

## Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/omni-mcp/isaac-sim-mcp.git
    cd isaac-sim-mcp
    ```

2.  **Run the environment setup script:**
    This script will create a Python virtual environment (`.venv`), activate it, and install all required dependencies from `requirements.txt`.
    ```bash
    bash scripts/setup_env.sh
    ```

3.  **Activate the virtual environment for your current session:**
    You will need to do this every time you open a new terminal.
    ```bash
    source .venv/bin/activate
    ```

## Usage

This project requires two main components to be running: the **Server** (within Isaac Sim) and the **Client** (the learning agent).

### Step 1: Run the Isaac Sim MCP Server

The server must be launched using the Python environment bundled with Isaac Sim.

1.  **Navigate to your Isaac Sim installation directory.**

2.  **Run the `run_isaac_sim_server.py` script using `python.sh` (Linux) or `python.bat` (Windows).**
    You must provide the absolute path to the script.

    **On Linux:**
    ```bash
    ./python.sh /path/to/your/roarm_mcp/examples/run_isaac_sim_server.py --robot-type ur10 --headless
    ```

    **On Windows:**
    ```bat
    .\python.bat C:\path\to\your\roarm_mcp\examples\run_isaac_sim_server.py --robot-type ur10 --headless
    ```

    **Server Arguments:**
    *   `--host`: Host to bind to (default: `localhost`).
    *   `--port`: Port to listen on (default: `8765`).
    *   `--robot-type`: The robot to load (`ur10` or `franka`).
    *   `--headless`: Run Isaac Sim without a GUI. Recommended for performance.

The server will start and wait for a client connection.

### Step 2: Run a Client

In a **new terminal**, activate the project's virtual environment and run a client.

```bash
source .venv/bin/activate
```

You can use one of the provided example clients:

*   **`isaac_sim_client.py`**: A great starting point that demonstrates basic interaction by sending random actions.
    ```bash
    python -m examples.isaac_sim_client --server-url ws://localhost:8765
    ```

*   **`sample_client.py`**: A lower-level client that manually constructs JSON messages. Useful for debugging the MCP protocol.
    ```bash
    python -m examples.sample_client --host localhost --port 8765
    ```

## Technical Note: Isaac Sim 5.0 PhysX Tensors Solution

This repository includes a robust solution for a common issue in Isaac Sim 5.0 where loading multiple robots can create nested `ArticulationRoot` prims, which is not allowed by the PhysX Tensors API.

Our solution, demonstrated in `isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py`, involves:
1.  **Cleaning the USD Stage**: Programmatically traversing the stage to remove nested `ArticulationRootAPI` schemas, ensuring only the top-level prim for each robot has one.
2.  **Correct Initialization Order**: Resetting the world *after* cleaning the stage but *before* creating the `ArticulationView` to ensure the physics engine registers the hierarchy correctly.
3.  **Using `create_simulation_view`**: Leveraging the modern `omni.physics.tensors.create_simulation_view()` to create articulation views, which is the correct approach for the PhysX Tensors pipeline.

This ensures that multi-robot simulations load reliably and can be controlled via the high-performance tensor-based API.

## API Documentation

The codebase is thoroughly documented with Google-style Python docstrings. Key modules to review include:

*   `roarm_mcp.mcp.server.MCPServer`: The main server implementation.
*   `roarm_mcp.mcp.client.MCPClient`: The `gymnasium.Env` client for RL agents.
*   `roarm_mcp.mcp.protocol`: Defines all message types used for communication.
*   `roarm_mcp.envs.robot_env`: The base classes for the RL environments.
*   `roarm_mcp.sensors`: Contains the high-performance camera sensor modules.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.