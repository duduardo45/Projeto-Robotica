import asyncio
import time

import cv2
import numpy as np
import websockets

WS_PORT = 8000
WS_PATH = "/ws/robot"


async def handle_client(connection: websockets.ServerConnection) -> None:
    """Handle a single websocket client connection."""
    try:
        async for message in connection:
            if isinstance(message, bytes):
                recv_time = time.perf_counter_ns()
                frame_size = len(message)

                # Decode JPEG frame
                np_arr = np.frombuffer(message, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if img is not None:
                    decode_time = time.perf_counter_ns()
                    decode_duration_ns = decode_time - recv_time
                    print(
                        f"[RX] Frame size: {frame_size} bytes, "
                        f"Decode time: {decode_duration_ns / 1e6:.3f} ms"
                    )
                    cv2.imshow("Robot Camera", img)
                    cv2.waitKey(1)
            else:
                print(f"[RX Text]: {message}")

    except websockets.exceptions.ConnectionClosed:
        print("[Disconnected] Client disconnected")
    except Exception as e:
        print(f"[Error]: {e}")


async def main():
    """Start the websocket server."""
    print(f"Starting WebSocket server on ws://0.0.0.0:{WS_PORT}{WS_PATH}")
    print("Waiting for ESP32 to connect...")

    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT):  # type: ignore
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down...")
        cv2.destroyAllWindows()
