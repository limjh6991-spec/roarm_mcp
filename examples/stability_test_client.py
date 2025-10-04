#!/usr/bin/env python3
"""An advanced client for server stability and error handling tests.

This script provides a suite of tests designed to verify the stability,
robustness, and error-handling capabilities of a WebSocket server, such as the
`simple_isaac_sim_server.py` or a full `run_server.py` instance.

It includes the following test scenarios:
  1.  **Connection/Disconnection:** Repeatedly connects and disconnects to
      ensure the server handles client churn gracefully.
  2.  **Invalid Messages:** Sends various forms of malformed or invalid JSON
      messages to test the server's parsing and error-handling logic.
  3.  **Rapid Messages:** Sends a burst of messages in quick succession to
      test the server's ability to handle high traffic without dropping
      connections or blocking.
  4.  **Long Connection:** Maintains a connection over an extended period,
      sending periodic heartbeat messages to check for connection stability
      and timeouts.

Usage:
    python -m examples.stability_test_client --server-url <ws_url>
"""

import asyncio
import websockets
import json
import logging
import time
import argparse
import traceback

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class StabilityTestClient:
    """A client to perform stability tests on a WebSocket server.

    Attributes:
        server_url (str): The WebSocket URL of the server to test.
    """
    def __init__(self, server_url: str):
        """Initializes the stability test client.

        Args:
            server_url (str): The WebSocket URL of the server.
        """
        self.server_url = server_url
        
    async def test_connection_disconnect(self):
        """Tests the server's ability to handle rapid connections and disconnections."""
        logger.info("🔌 Starting connection/disconnection test...")
        
        for i in range(3):
            try:
                async with websockets.connect(self.server_url) as websocket:
                    logger.info(f"✅ Connection {i+1}/3 successful.")
                    message = {"action": "ping", "test": f"connection_{i+1}"}
                    await websocket.send(json.dumps(message))
                    response = await websocket.recv()
                    logger.info(f"📨 Received response with status: {json.loads(response).get('status')}")
                    await asyncio.sleep(0.5)
                logger.info(f"🔌 Connection {i+1} closed normally.")
            except Exception as e:
                logger.error(f"❌ An error occurred during connection test {i+1}: {e}")
                
    async def test_invalid_messages(self):
        """Tests the server's handling of malformed and invalid messages."""
        logger.info("⚠️ Starting invalid message test...")
        
        invalid_messages = [
            "this is not json",          # Invalid JSON
            '{"action": "test", "bad',   # Incomplete JSON
            '{}',                        # Empty JSON object
            '{"action": "unknown_cmd"}', # Unknown command
            '{"action": null}',          # Null action
        ]
        
        try:
            async with websockets.connect(self.server_url) as websocket:
                logger.info("✅ Connected to server for invalid message test.")
                for i, invalid_msg in enumerate(invalid_messages, 1):
                    logger.info(f"📤 Sending invalid message {i}/{len(invalid_messages)}: {invalid_msg[:40]}...")
                    try:
                        await websocket.send(invalid_msg)
                        response = await websocket.recv()
                        data = json.loads(response)
                        logger.info(f"📨 Server responded with status: '{data.get('status', 'unknown')}'")
                    except Exception as e:
                        logger.warning(f"⚠️ An error occurred while processing message {i}: {e}")
                    await asyncio.sleep(0.5)
        except Exception as e:
            logger.error(f"❌ Failed to run invalid message test: {e}")
            
    async def test_rapid_messages(self, count: int = 50):
        """Tests the server's performance under a high message load.

        Args:
            count (int): The number of messages to send in a burst.
        """
        logger.info(f"🚀 Starting rapid message test ({count} messages)...")
        
        try:
            async with websockets.connect(self.server_url) as websocket:
                logger.info("✅ Connected to server for rapid message test.")
                
                # Send a burst of messages
                for i in range(count):
                    message = {"action": "rapid_test", "sequence": i + 1, "timestamp": time.time()}
                    await websocket.send(json.dumps(message))
                logger.info(f"📤 All {count} messages sent.")

                # Try to receive all responses
                responses_received = 0
                try:
                    for _ in range(count):
                        await asyncio.wait_for(websocket.recv(), timeout=2.0)
                        responses_received += 1
                    logger.info(f"✅ Received all {responses_received}/{count} responses.")
                except asyncio.TimeoutError:
                    logger.warning(f"⚠️ Timed out after receiving {responses_received}/{count} responses.")
                        
        except Exception as e:
            logger.error(f"❌ An error occurred during rapid message test: {e}", exc_info=True)
            
    async def test_long_connection(self, duration_sec: int = 20):
        """Tests the server's ability to maintain a long-lived connection.

        Args:
            duration_sec (int): The duration of the test in seconds.
        """
        logger.info(f"⏱️ Starting long connection test ({duration_sec} seconds)...")
        
        try:
            async with websockets.connect(self.server_url, ping_interval=5, ping_timeout=5) as websocket:
                logger.info("✅ Connection established for long duration test.")
                start_time = time.time()
                message_count = 0
                
                while time.time() - start_time < duration_sec:
                    message = {"action": "heartbeat", "timestamp": time.time()}
                    await websocket.send(json.dumps(message))
                    await websocket.recv()
                    message_count += 1
                    
                    if message_count % 5 == 0:
                        logger.info(f"💗 Heartbeat #{message_count} successful.")
                    await asyncio.sleep(1)
                    
                logger.info(f"✅ Long connection test complete. Sent {message_count} messages over {time.time() - start_time:.1f}s.")
                
        except Exception as e:
            logger.error(f"❌ An error occurred during the long connection test: {e}", exc_info=True)

    async def run_all_tests(self):
        """Runs the complete suite of stability tests."""
        logger.info("🧪 === Starting Full Stability Test Suite ===")
        
        tests = [
            ("Connection/Disconnection", self.test_connection_disconnect),
            ("Invalid Messages", self.test_invalid_messages),
            ("Rapid Messages", self.test_rapid_messages),
            ("Long Connection", self.test_long_connection),
        ]
        
        for name, test_func in tests:
            logger.info(f"\n🔬 --- Running: {name} ---")
            try:
                await test_func()
                logger.info(f"✅ --- Finished: {name} ---")
            except Exception as e:
                logger.error(f"❌ --- Test Failed: {name} ---")
                logger.error(traceback.format_exc())
            
            await asyncio.sleep(2)
            
        logger.info("\n🎉 === All Stability Tests Complete ===")

async def main():
    """Parses arguments and runs the test suite."""
    parser = argparse.ArgumentParser(
        description="Run stability tests against an MCP server.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--server-url",
        type=str,
        default="ws://localhost:8765",
        help="The WebSocket URL of the server to test."
    )
    args = parser.parse_args()

    client = StabilityTestClient(args.server_url)
    await client.run_all_tests()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Test suite interrupted by user.")