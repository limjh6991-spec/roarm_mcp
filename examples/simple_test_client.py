#!/usr/bin/env python3
"""A minimal test client for the Isaac Sim MCP Server.

This script provides a very basic example of a WebSocket client that connects
to the server, sends a predefined sequence of test messages, and prints the
responses.

Its primary purpose is to serve as a quick sanity check to ensure that the
server is running, accepting connections, and responding to simple requests.

It is less comprehensive than `stability_test_client.py` and does not perform
any complex interaction or environment looping like `isaac_sim_client.py`.

Usage:
    python -m examples.simple_test_client --server-url <ws_url>
"""

import asyncio
import websockets
import json
import logging
import argparse
import traceback

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SimpleTestClient:
    """A client for performing a basic connection and message sending test.

    Attributes:
        server_url (str): The WebSocket URL of the server to connect to.
    """
    def __init__(self, server_url: str):
        """Initializes the simple test client.

        Args:
            server_url (str): The WebSocket URL of the server.
        """
        self.server_url = server_url
        
    async def test_connection(self):
        """Connects to the server and sends a sequence of test messages."""
        try:
            async with websockets.connect(self.server_url) as websocket:
                logger.info(f"✅ Successfully connected to server: {self.server_url}")
                
                # A predefined list of simple test messages to send,
                # reflecting the actual MCP protocol.
                test_messages = [
                    {"type": "action_space"},
                    {"type": "observation_space"},
                    {"type": "reset"},
                ]
                
                for i, message in enumerate(test_messages, 1):
                    action_type = message.get('type', 'unknown')
                    logger.info(f"📤 Sending message {i}/{len(test_messages)}: type='{action_type}'")
                    await websocket.send(json.dumps(message))
                    
                    # Wait for and log the response.
                    # Note: A "reset" action will be followed by 5 messages from a full
                    # server, but this simple client only waits for the first one.
                    response = await websocket.recv()
                    data = json.loads(response)
                    response_type = data.get('type', 'unknown')
                    logger.info(f"📨 Received response {i}/{len(test_messages)} with type: '{response_type}'")
                    
                    await asyncio.sleep(1)
                    
                logger.info("🎉 All test messages were sent and initial responses were received.")
                
        except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError) as e:
            logger.error(f"❌ Connection failed: {e}. Is the server running?")
        except Exception as e:
            logger.error(f"❌ An unexpected error occurred during the test: {e}", exc_info=True)
            
async def main():
    """Parses arguments and runs the simple client test."""
    parser = argparse.ArgumentParser(
        description="Run a simple test client for the MCP server.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--server-url",
        type=str,
        default="ws://localhost:8765",
        help="The WebSocket URL of the server to test."
    )
    args = parser.parse_args()

    client = SimpleTestClient(args.server_url)
    await client.test_connection()
    
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Client test interrupted by user.")