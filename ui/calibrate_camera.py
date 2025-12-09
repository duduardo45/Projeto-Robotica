import glob

import cv2
import numpy as np

# --- CONFIGURATION ---
CHECKERBOARD_SIZE = (8, 5)  # Internal corners (width-1, height-1) of your pattern
SQUARE_SIZE_METERS = 0.03  # Size of one square side in meters (measure your print!)
IMAGE_DIR = "esp32_calibration_images/*.jpg"  # Path to your images


def calibrate():
    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[
        0 : CHECKERBOARD_SIZE[0], 0 : CHECKERBOARD_SIZE[1]
    ].T.reshape(-1, 2)
    objp = objp * SQUARE_SIZE_METERS

    images = glob.glob(IMAGE_DIR)

    if not images:
        print("No images found! Check your path.")
        return

    print(f"Found {len(images)} images. Processing...")

    shape = None

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD_SIZE, cv2.CALIB_CB_ACCURACY
        )

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            # Increase accuracy of corner location
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            imgpoints.append(corners2)
            print(f" - Used {fname}")
        else:
            print(f" - Could not find corners in {fname}")

    print("\nCalibrating...")
    # The magic function
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, shape, None, None
    )

    print("\n--------- RESULTS for your code ---------")
    print(f"FX = {mtx[0, 0]:.3f}")
    print(f"FY = {mtx[1, 1]:.3f}")
    print(f"CX = {mtx[0, 2]:.3f}")
    print(f"CY = {mtx[1, 2]:.3f}")
    print("-----------------------------------------")
    print("\nDistortion Coefficients (k1, k2, p1, p2, k3):")
    print(dist.ravel())


if __name__ == "__main__":
    calibrate()


# import asyncio
# import os
# from datetime import datetime

# import cv2
# import numpy as np
# import websockets

# # CONFIGURATION
# ROBOT_WS_URL = "ws://192.168.4.1:8000"  # Or whatever IP your ESP32 has
# OUTPUT_DIR = "esp32_calibration_images"


# async def run_capture():
#     if not os.path.exists(OUTPUT_DIR):
#         os.makedirs(OUTPUT_DIR)
#         print(f"Created directory: {OUTPUT_DIR}")

#     print(f"Connecting to {ROBOT_WS_URL}...")

#     async with websockets.connect(ROBOT_WS_URL) as websocket:
#         print("Connected! Press 's' to save a photo, 'q' to quit.")

#         while True:
#             try:
#                 message = await websocket.recv()

#                 # Ignore text telemetry, we only want binary video frames
#                 if isinstance(message, str):
#                     continue

#                 # Decode the image
#                 np_arr = np.frombuffer(message, dtype=np.uint8)
#                 img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#                 if img is None:
#                     continue

#                 # Show the live feed
#                 cv2.imshow("Calibration Stream", img)

#                 key = cv2.waitKey(1) & 0xFF

#                 # Press 's' to save
#                 if key == ord("s"):
#                     timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#                     filename = f"{OUTPUT_DIR}/calib_{timestamp}.jpg"
#                     cv2.imwrite(filename, img)
#                     print(f"Saved {filename} ({img.shape[1]}x{img.shape[0]})")

#                     # Flash the screen white briefly to confirm save
#                     white_frame = np.full_like(img, 255)
#                     cv2.imshow("Calibration Stream", white_frame)
#                     cv2.waitKey(50)

#                 # Press 'q' to quit
#                 elif key == ord("q"):
#                     break

#             except websockets.exceptions.ConnectionClosed:
#                 print("Connection closed.")
#                 break
#             except Exception as e:
#                 print(f"Error: {e}")
#                 break

#     cv2.destroyAllWindows()


# if __name__ == "__main__":
#     try:
#         asyncio.run(run_capture())
#     except KeyboardInterrupt:
#         pass
