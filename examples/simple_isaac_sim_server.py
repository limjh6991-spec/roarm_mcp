#!/usr/bin/env python3
"""A simplified mock MCP Server for basic client testing.

!!! WARNING: This is a basic, non-functional mock server. !!!

This script starts a simple WebSocket server that accepts connections but does
not connect to an actual Isaac Sim environment or implement the true Model
Context Protocol (MCP) logic. Its primary purpose is to act as a simple
endpoint for testing client connectivity and message echoing.

It will accept any JSON message and return a generic success response. It does
not process actions or manage an environment state.

For a fully functional server, please use `examples/run_server.py`.
"""

import asyncio
import websockets
import json
import logging
import argparse

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class SimpleIsaacSimMCPServer:
    """A simple, non-functional mock server to echo client messages.

    This class manages client connections and responds to any incoming message
    with a generic success status.

    Attributes:
        clients (set): A set of currently connected WebSocket clients.
    """
    def __init__(self):
        """Initializes the simple mock server."""
        self.clients = set()
        
    async def register_client(self, websocket):
        """Adds a new client to the set of connected clients.

        Args:
            websocket: The WebSocket connection object for the new client.
        """
        self.clients.add(websocket)
        logger.info(f"✅ Client connected: {websocket.remote_address} (Total: {len(self.clients)})")
        
    async def unregister_client(self, websocket):
        """Removes a client from the set of connected clients.

        Args:
            websocket: The WebSocket connection object for the disconnected client.
        """
        self.clients.discard(websocket)
        logger.info(f"❌ Client disconnected: {websocket.remote_address} (Total: {len(self.clients)})")
        
    async def handle_message(self, websocket, message: str):
        """Processes a message from a client by sending a mock response.

        Args:
            websocket: The WebSocket connection object of the sender.
            message (str): The incoming message string from the client.
        """
        try:
            data = json.loads(message)
            action = data.get('type', 'unknown_action')
            logger.info(f"📨 Received message of type '{action}' from {websocket.remote_address}")
            
            # Create a generic, mock response
            response = {
                "status": "success",
                "message": f"Mock server received action: {action}",
                "observation": [0.0, 0.0, 0.0], # Mock observation
                "reward": 0.1, # Mock reward
                "terminated": False,
                "truncated": False,
                "info": {"mock": True}
            }
            
            await websocket.send(json.dumps(response))
            logger.info(f"📤 Sent mock response to {websocket.remote_address}")
            
        except json.JSONDecodeError:
            error_response = {"status": "error", "message": "Invalid JSON received."}
            await websocket.send(json.dumps(error_response))
            
    async def client_handler(self, websocket, path: str):
        """Manages the lifecycle of a single client connection.

        Args:
            websocket: The WebSocket connection object.
            path (str): The requested connection path.
        """
        await self.register_client(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Connection with {websocket.remote_address} closed normally.")
        except Exception as e:
            logger.error(f"An error occurred with client {websocket.remote_address}: {e}")
        finally:
            await self.unregister_client(websocket)
            
    async def start(self, host: str, port: int):
        """Starts the WebSocket server and runs it indefinitely.

        Args:
            host (str): The hostname or IP address to bind to.
            port (int): The port number to listen on.
        """
        logger.info(f"🚀 Starting simple mock MCP server on ws://{host}:{port}")
        
        async with websockets.serve(self.client_handler, host, port):
            logger.info("📡 Mock server is running. Press Ctrl+C to stop.")
            await asyncio.Future()  # Run forever

async def main():
    """Parses arguments and starts the server."""
    parser = argparse.ArgumentParser(description="Run a simple mock MCP server.")
    parser.add_argument("--host", type=str, default="localhost", help="Host to bind to.")
    parser.add_argument("--port", type=int, default=8765, help="Port to bind to.")
    args = parser.parse_args()

    server = SimpleIsaacSimMCPServer()
    try:
        await server.start(args.host, args.port)
    except KeyboardInterrupt:
        logger.info("⚡ Server shutting down.")

if __name__ == "__main__":
    asyncio.run(main())