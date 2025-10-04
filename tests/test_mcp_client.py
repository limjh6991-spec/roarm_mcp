import asyncio
import json
import random
import websockets
from unittest.mock import MagicMock, AsyncMock

from mcp.client import MCPClient
from mcp.protocol import (
    MCPActionSpaceMessage, MCPObservationSpaceMessage, MCPStepMessage,
    MCPObservationMessage, MCPRewardMessage, MCPTerminatedMessage,
    MCPTruncatedMessage, MCPInfoMessage, MCPErrorMessage, MCPSpace, SpaceType
)


# Mock server to test the client
async def mock_server(websocket, path):
    """
    A mock server that sends step responses in a random order.
    """
    try:
        async for message in websocket:
            data = json.loads(message)
            msg_type = data["type"]

            if msg_type == "action_space":
                space = MCPSpace(type=SpaceType.BOX, shape=[1], low=[-1], high=[1])
                await websocket.send(json.dumps({"type": "action_space", "space": space.to_dict()}))
            elif msg_type == "observation_space":
                space = MCPSpace(type=SpaceType.BOX, shape=[1], low=[-1], high=[1])
                await websocket.send(json.dumps({"type": "observation_space", "space": space.to_dict()}))
            elif msg_type == "step":
                # The core of the test: send responses in a random order
                responses = [
                    MCPObservationMessage([0.5]).to_json(),
                    MCPRewardMessage(1.0).to_json(),
                    MCPTerminatedMessage(False).to_json(),
                    MCPTruncatedMessage(False).to_json(),
                    MCPInfoMessage({"detail": "some_info"}).to_json()
                ]
                random.shuffle(responses)
                for res in responses:
                    await websocket.send(res)
            elif msg_type == "reset":
                await websocket.send(MCPObservationMessage([0.0]).to_json())

    except websockets.exceptions.ConnectionClosed:
        pass


async def run_test():
    """
    Tests if the client correctly handles step responses arriving in a random order.
    This test should fail with the original implementation and pass with the fix.
    """
    server = await websockets.serve(mock_server, "localhost", 8765)
    client = MCPClient(host="localhost", port=8765)

    try:
        # Connect to the server and get action/observation spaces
        await client.connect()

        # Perform a step and get the results
        observation, reward, terminated, truncated, info = await client.step(action=[0.1])

        print("Test received:")
        print(f"  observation: {observation}")
        print(f"  reward: {reward}")
        print(f"  terminated: {terminated}")
        print(f"  truncated: {truncated}")
        print(f"  info: {info}")

        assert observation == [0.5], f"Expected [0.5], got {observation}"
        assert reward == 1.0, f"Expected 1.0, got {reward}"
        assert terminated is False, f"Expected False, got {terminated}"
        assert truncated is False, f"Expected False, got {truncated}"
        assert info == {"detail": "some_info"}, f"Expected {{'detail': 'some_info'}}, got {info}"

        print("\\n*** Test PASSED ***")

    except Exception as e:
        print(f"\\n*** Test FAILED: {e} ***")
        import traceback
        traceback.print_exc()
    finally:
        if client.connected:
            await client.close()
        server.close()
        await server.wait_closed()


if __name__ == "__main__":
    asyncio.run(run_test())