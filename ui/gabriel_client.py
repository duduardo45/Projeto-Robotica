import json
import threading

import cv2
import numpy as np
import websocket

# Configuration
websocket_url = "ws://192.168.4.1:8000"

# Shared variables
latest_frame_bytes = None
keep_running = True
ws_app = None


def on_message(ws: websocket.WebSocketApp, message: str | bytes) -> None:
    """
    Handles incoming messages.
    Crucial optimization: We do NO processing here. We just dump the raw data
    into a variable and return immediately to keep the socket buffer empty.
    """
    global latest_frame_bytes

    if isinstance(message, bytes):
        # Update the variable with the absolute latest frame
        latest_frame_bytes = message
    else:
        print(f"[RX Text]: {message}")


def on_error(ws: websocket.WebSocketApp, error: Exception) -> None:
    print(f"[Error]: {error}")


def on_close(
    ws: websocket.WebSocketApp, close_status_code: int, close_msg: str
) -> None:
    print("[Closed]: Connection closed.")
    global keep_running
    keep_running = False


def on_open(ws: websocket.WebSocketApp) -> None:
    print("[Open]: Connection established!")


def send_command(cmd_char: int) -> None:
    """Sends JSON commands based on key presses."""
    data = None
    if cmd_char == ord("e"):
        data = {"type": "fork", "fork": 1}
    elif cmd_char == ord("q"):
        data = {"type": "fork", "fork": 0}
    elif cmd_char == ord("w"):
        data = {"type": "mov", "dir": 500, "esq": 500}
    elif cmd_char == ord("a"):
        data = {"type": "mov", "dir": 500, "esq": -500}
    elif cmd_char == ord("s"):
        data = {"type": "mov", "dir": -500, "esq": -500}
    elif cmd_char == ord("d"):
        data = {"type": "mov", "dir": -500, "esq": 500}
    elif cmd_char == ord("f"):  # Stop
        data = {"type": "mov", "dir": 0, "esq": 0}

    if data and ws_app:
        try:
            ws_app.send(json.dumps(data))
        except Exception as e:
            print(f"Send error: {e}")


def run_websocket():
    """Runs the websocket connection in a background thread."""
    global ws_app
    # enable_trace=False improves performance slightly by removing debug prints
    websocket.enableTrace(False)
    ws_app = websocket.WebSocketApp(
        websocket_url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )
    ws_app.run_forever()


if __name__ == "__main__":
    print(f"Connecting to {websocket_url}...")
    print("-------------------------------------------------")
    print("CLICK THE VIDEO WINDOW TO CONTROL")
    print("Controls: W, A, S, D | Q, E (Fork) | F (Stop) | ESC (Exit)")
    print("-------------------------------------------------")

    # 1. Start WebSocket in a separate thread
    ws_thread = threading.Thread(target=run_websocket)
    ws_thread.daemon = True
    ws_thread.start()

    # 2. Main Loop: Handle Image Decoding and Display
    # This loop runs as fast as the CPU allows, independent of network speed
    while keep_running:
        if latest_frame_bytes is not None:
            try:
                # A. Copy bytes to local scope to unblock the receiver immediately
                frame_data = latest_frame_bytes

                # B. Decode raw bytes to image (CPU Intensive)
                # converting to numpy array
                np_arr = np.frombuffer(frame_data, np.uint8)
                # decoding to image
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if img is not None:
                    # C. Display
                    cv2.imshow("Fast Stream", img)

            except Exception as e:
                print(f"Frame processing error: {e}")

        # D. Handle User Input via OpenCV Window
        # waitKey(1) waits 1ms for a key press. This dictates the loop speed.
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC key
            print("Exiting...")
            keep_running = False
            if ws_app:
                ws_app.close()
            break
        elif key != 255:  # If a key was actually pressed
            send_command(key)
            # Optional: Clear key after sending to prevent auto-repeat spam
            # if your hardware prefers single-shot commands

    cv2.destroyAllWindows()
