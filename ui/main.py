from __future__ import annotations

import asyncio
import logging
import math
import time
from asyncio import TaskGroup
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any, AsyncIterable, Iterable, Literal

import numpy as np
import uvicorn
import websockets
from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, status
from fastapi.middleware.cors import CORSMiddleware
from pupil_apriltags import (  # pyright: ignore [reportMissingTypeStubs]
    Detection,
    Detector,
)
from pydantic import BaseModel, Field
from turbojpeg import TJPF_GRAY, TurboJPEG  # pyright: ignore [reportMissingTypeStubs]
from websockets.asyncio.client import ClientConnection

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())

# --- Constants ---

# define FX 509.932 // fx (in pixel)
# define FY 509.932 // fy (in pixel)
# define CX 390.456 // cx (in pixel)
# define CY 176.282 // cy (in pixel)

WHEEL_BASE_METERS = 0.17  # distance between the two wheels
WHEEL_RADIUS_METERS = 0.025
TICKS_PER_ROTATION = 64.0
TICKS_PER_METER = TICKS_PER_ROTATION * (1.0 / 2 * math.pi * WHEEL_RADIUS_METERS)
APRILTAG_SIZE_METERS = 0.043

CAMERA_PARAMS = (509.932, 509.932, 390.456, 176.282)
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
FRAME_FORMAT: Literal["jpeg", "grayscale"] = "jpeg"

ARRIVAL_TOLERANCE_METERS = 0.05
HEADING_TOLERANCE_RADIANS = 0.1
ANGULAR_SPEED_LIMIT_RADIANS = 2.0
ANGULAR_SPEED_GAIN = 2.0
HEADING_GAIN = 1.0

# Robot WebSocket Configuration
ROBOT_WS_URL = "ws://192.168.4.1:8000"
HEARTBEAT_INTERVAL_SECONDS = 0.2  # Send heartbeat every 200ms
STATE_BROADCAST_INTERVAL_SECONDS = 0.04  # Broadcast robot state every 40ms

TAG_MAP = {
    0: (0.0, 0.0, 0.0),  # Origin
    1: (2.0, 0.0, 0.0),  # Example goal
}

# --- Helpers ---


def monotonic_micros() -> int:
    return time.time_ns() // 1_000


def clamp(value: float, limit: float) -> float:
    """Simple symmetric clamp."""
    return max(-limit, min(limit, value))


def normalize_angle(angle: float) -> float:
    # Uses atan2 to wrap exactly to (-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation_matrix_to_yaw(R: np.ndarray[Any, Any]) -> float:
    """
    Extracts the 2D Yaw (rotation around Z-axis) from a 3x3 rotation matrix.
    Assumes standard camera coordinates where Z is forward, X is right.
    """
    # simply: atan2(R[0, 2], R[2, 2]) for yaw in camera frame usually works
    # for planar movement, but standard conversion is safer:
    return math.atan2(R[1, 0], R[0, 0])


def rigid_transform_3d_to_2d(
    t_id: int, t_dist: float, t_yaw_cam: float, t_angle_offset: float
) -> Pose | None:
    # 1. Validate Tag ID
    if t_id not in TAG_MAP:
        return None

    tag_x, tag_y, tag_theta = TAG_MAP[t_id]

    # 2. Calculate Robot Heading
    # Tag Global Angle - Relative Yaw + 180° flip (facing opposite directions)
    r_theta = normalize_angle(tag_theta - t_yaw_cam + math.pi)

    # 3. Calculate Robot Position
    # We subtract the vector pointing from Robot->Tag from the Tag's location
    # Vector Angle = Robot Heading + Angle of tag in image
    bearing_global = r_theta + t_angle_offset

    rx = tag_x - (t_dist * math.cos(bearing_global))
    ry = tag_y - (t_dist * math.sin(bearing_global))

    return Pose(x=rx, y=ry, theta=r_theta)


class RobotWebSocketNotConnected(Exception):
    pass


# --- Data Models (Simplified) ---


@dataclass(frozen=True, kw_only=True, slots=True)
class Pose:
    x: float
    y: float
    theta: float


@dataclass(kw_only=True, slots=True, frozen=True)
class FramePacket:
    width: int
    height: int
    data: bytes
    format: Literal["jpeg", "grayscale"]


class WheelTelemetry(BaseModel):
    encoder: int
    raw_speed: float
    filtered_speed: float
    target_speed: float
    debug_f: float
    debug_p: float
    debug_i: float
    debug_out: float


class RobotState(BaseModel):
    """Now includes the calculated Pose."""

    message_type: Literal["robot_state"] = "robot_state"
    pose: Pose
    left: WheelTelemetry
    right: WheelTelemetry
    last_update_micros: int


class TelemetryIncoming(BaseModel):
    message_type: Literal["telemetry"] = "telemetry"
    timestamp: int
    left: WheelTelemetry
    right: WheelTelemetry


class WheelsCommand(BaseModel):
    message_type: Literal["command"] = "command"
    left: float
    right: float


class Heartbeat(BaseModel):
    message_type: Literal["heartbeat"] = "heartbeat"


class CommandRequest(BaseModel):
    left: float
    right: float


class CommandResponse(BaseModel):
    status: str


class TagDetection(BaseModel):
    tag_id: int
    distance_m: float
    center: tuple[float, float]
    yaw_rel: float
    angle_offset: float


class VisionSnapshot(BaseModel):
    message_type: Literal["vision_snapshot"] = "vision_snapshot"
    detections: list[TagDetection]


class MissionRequest(BaseModel):
    """Barebones mission request."""

    x: float
    y: float
    speed: float


class MissionStatus(BaseModel):
    state: Literal["idle", "running", "completed"] = "idle"


# --- Internal State ---


@dataclass(kw_only=True, slots=True)
class Mission:
    x: float
    y: float
    speed: float


class DashboardMessage(BaseModel):
    data: RobotState | VisionSnapshot = Field(discriminator="message_type")


class DashboardHub:
    def __init__(self) -> None:
        self._connections: set[WebSocket] = set()

    async def register(self, websocket: WebSocket) -> None:
        self._connections.add(websocket)

    async def unregister(self, websocket: WebSocket) -> None:
        self._connections.discard(websocket)

    async def broadcast_dashboard_message(self, message: DashboardMessage) -> None:
        for ws in list(self._connections):
            try:
                await ws.send_text(message.model_dump_json())
            except Exception:
                await self.unregister(ws)

    async def broadcast_frame_packet(self, data: bytes) -> None:
        for ws in list(self._connections):
            try:
                await ws.send_bytes(data)
            except Exception:
                await self.unregister(ws)


class Robot:
    def __init__(self) -> None:
        self._state = RobotState(
            pose=Pose(x=0.0, y=0.0, theta=0.0),
            left=WheelTelemetry(
                encoder=0,
                raw_speed=0.0,
                filtered_speed=0.0,
                target_speed=0.0,
                debug_f=0.0,
                debug_p=0.0,
                debug_i=0.0,
                debug_out=0.0,
            ),
            right=WheelTelemetry(
                encoder=0,
                raw_speed=0.0,
                filtered_speed=0.0,
                target_speed=0.0,
                debug_f=0.0,
                debug_p=0.0,
                debug_i=0.0,
                debug_out=0.0,
            ),
            last_update_micros=0,
        )
        self._ws_client: ClientConnection | None = None
        self._mission: Mission | None = None
        self._last_command_left: float = 0.0
        self._last_command_right: float = 0.0

        # We need a reference to the supervisor to cancel it on shutdown
        self._supervisor_task: asyncio.Task[None] | None = None
        self._running = False  # Flag to control the supervisor loop

        # Reduced detector settings for speed
        self._detector = Detector(families="tag36h11", quad_decimate=1.0, nthreads=4)
        self._latest_detections: list[TagDetection] = []
        self._jpeg = TurboJPEG()

    async def _run_lifecycle(self) -> None:
        """
        Uses TaskGroup to manage the four dependent loops.
        If connection dies, everything cancels, and supervisor restarts them.
        """
        async with TaskGroup() as tg:
            tg.create_task(self._robot_ws_client_loop())
            tg.create_task(self._heartbeat_loop())
            tg.create_task(self._broadcast_state_loop())

    async def start(self) -> None:
        self._running = True
        self._supervisor_task = asyncio.create_task(self._run_supervisor())

    async def _run_supervisor(self) -> None:
        """
        The Resilient Supervisor:
        If the logic crashes, we wait 1 second and restart the TaskGroup.
        We do NOT kill the process.
        """
        logger.info("Robot Supervisor started.")
        while self._running:
            try:
                # This blocks until one of the sub-tasks raises an exception
                # or the connection closes unexpectedly.
                await self._run_lifecycle()
            except asyncio.CancelledError:
                # App is shutting down
                break
            except Exception:
                logger.exception("Robot subsystem crashed. Restarting in 1s...")
                # Reset critical state if needed
                self._ws_client = None
                await asyncio.sleep(1.0)

    async def shutdown(self) -> None:
        self._running = False
        if self._supervisor_task:
            self._supervisor_task.cancel()
            try:
                await self._supervisor_task
            except asyncio.CancelledError:
                pass

        await self.stop()

    def mission_status(self) -> MissionStatus:
        state = "running" if self._mission else "idle"
        return MissionStatus(state=state)

    async def set_mission(self, request: MissionRequest) -> MissionStatus:
        if self._mission:
            await self.cancel_mission()
        self._mission = Mission(x=request.x, y=request.y, speed=request.speed)
        return self.mission_status()

    async def cancel_mission(self) -> MissionStatus:
        self._mission = None
        await self.stop()
        return self.mission_status()

    async def _send_command_raw(self, left: float, right: float) -> None:
        msg = WheelsCommand(left=left, right=right)
        await self._send_ws(msg.model_dump_json())

    async def send_command(self, request: CommandRequest) -> None:
        if self._mission:
            await self.cancel_mission()
        await self._send_command_raw(request.left, request.right)

    async def stop(self) -> None:
        if self._mission:
            # Don't call cancel_mission here to avoid circular logic in shutdown
            self._mission = None
        await self._send_command_raw(0.0, 0.0)

    # --- Simplified Odometry Here ---
    async def _apply_telemetry(self, msg: TelemetryIncoming) -> None:
        """Calculates Pose based on exact Encoder Deltas (Safe & Accurate)."""
        # 1. Initialize previous state on first run
        if self._state.last_update_micros == 0:
            self._state.left = msg.left
            self._state.right = msg.right
            self._state.last_update_micros = msg.timestamp
            return

        # 2. Calculate the CHANGE in ticks (Delta)
        # Note: You usually need to handle integer overflow (wrapping) here if
        # the robot runs for days, but for simple missions, subtraction is fine.
        delta_ticks_left = msg.left.encoder - self._state.left.encoder
        delta_ticks_right = msg.right.encoder - self._state.right.encoder

        # 3. Convert Ticks to Meters
        d_left = delta_ticks_left / TICKS_PER_METER
        d_right = delta_ticks_right / TICKS_PER_METER

        # 4. Calculate Distance and Rotation (Differential Drive Kinematics)
        # The average distance both wheels traveled is the center distance
        d_center = (d_left + d_right) / 2.0

        # The difference in distance creates rotation
        d_theta = (d_right - d_left) / WHEEL_BASE_METERS

        # 5. Calculate new Pose (trigonometry)
        # We use the average theta during the movement for better accuracy (Runge-Kutta 2nd order approximation)
        current_pose = self._state.pose
        avg_theta = current_pose.theta + (d_theta / 2.0)

        new_x = current_pose.x + d_center * math.cos(avg_theta)
        new_y = current_pose.y + d_center * math.sin(avg_theta)
        new_theta = normalize_angle(current_pose.theta + d_theta)
        new_pose = Pose(x=new_x, y=new_y, theta=new_theta)

        # 6. Save wheel telemetry state for next loop
        self._state.left = msg.left
        self._state.right = msg.right
        self._state.last_update_micros = msg.timestamp

        # 7. Update pose and trigger control loop
        await self._update_pose_and_react(new_pose, "telemetry")

    async def _process_frame(self, payload: bytes) -> None:
        await dashboard_hub.broadcast_frame_packet(payload)

        loop = asyncio.get_running_loop()
        # Don't block the event loop with vision processing
        # For vision processing, we need grayscale, so convert JPEG if needed
        tag_detections: list[TagDetection] = []
        if FRAME_FORMAT == "jpeg":
            tag_detections = await loop.run_in_executor(
                None, self._detect_tags_from_jpeg, payload
            )
        else:
            tag_detections = await loop.run_in_executor(
                None, self._detect_tags_grayscale, payload
            )
        if tag_detections:
            logger.debug(f"Detected {len(tag_detections)} tags")
            closest_tag = min(tag_detections, key=lambda t: t.distance_m)
            vision_pose = rigid_transform_3d_to_2d(
                closest_tag.tag_id,
                closest_tag.distance_m,
                closest_tag.yaw_rel,
                closest_tag.angle_offset,
            )
            if vision_pose:
                await self._update_pose_and_react(vision_pose, "vision")
            else:
                logger.warning("Tag ID not found in TAG_MAP")

        await dashboard_hub.broadcast_dashboard_message(
            DashboardMessage(
                data=VisionSnapshot(detections=tag_detections),
            )
        )

    async def _send_ws(
        self,
        message: websockets.Data
        | Iterable[websockets.Data]
        | AsyncIterable[websockets.Data],
    ):
        if self._ws_client:
            await self._ws_client.send(message)
        else:
            raise RobotWebSocketNotConnected

    async def _robot_ws_client_loop(self) -> None:
        """Connects to the robot websocket server and handles incoming messages."""
        while True:
            logger.info(f"Connecting to robot at {ROBOT_WS_URL}...")
            async for ws in websockets.connect(ROBOT_WS_URL):
                self._ws_client = ws
                logger.info("Connected to robot")
                try:
                    async for message in ws:
                        if isinstance(message, str):
                            telemetry = TelemetryIncoming.model_validate_json(message)
                            await self._apply_telemetry(telemetry)
                        else:  # Binary frame data
                            await self._process_frame(message)
                except websockets.exceptions.ConnectionClosed:
                    logger.info("Robot connection closed")
                finally:
                    self._ws_client = None

    async def _heartbeat_loop(self) -> None:
        """Sends periodic heartbeat messages to keep the robot alive."""
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL_SECONDS)
            msg = Heartbeat()
            try:
                await self._send_ws(msg.model_dump_json())
            except RobotWebSocketNotConnected:
                pass

    async def _broadcast_state_loop(self) -> None:
        """Periodically broadcasts robot state to dashboard clients."""
        while True:
            await asyncio.sleep(STATE_BROADCAST_INTERVAL_SECONDS)
            await dashboard_hub.broadcast_dashboard_message(
                DashboardMessage(data=self._state)
            )

    async def _update_pose_and_react(
        self, new_pose: Pose, source: Literal["telemetry", "vision"]
    ):
        """
        The Single Source of Truth for position updates.
        1. Updates the internal state.
        2. Updates the dashboard.
        3. Triggers the mission step logic if there is a mission.
        """

        self._state.pose = new_pose
        self._state.last_update_micros = monotonic_micros()

        await dashboard_hub.broadcast_dashboard_message(
            DashboardMessage(data=self._state)
        )

        # Optional: Log jumps if vision corrected us significantly
        if source == "vision":
            logger.info(f"Vision correction applied: {new_pose}")

        # The explicit trigger
        if self._mission:
            pose = self._state.pose
            dx = self._mission.x - pose.x
            dy = self._mission.y - pose.y
            dist = math.hypot(dx, dy)

            if dist < ARRIVAL_TOLERANCE_METERS:
                await self.stop()
                self._mission = None
                logger.info("Mission Completed")
                return

            target_angle = math.atan2(dy, dx)
            angle_error = normalize_angle(target_angle - pose.theta)

            lin_cmd = 0.0
            ang_cmd = 0.0

            if abs(angle_error) > HEADING_TOLERANCE_RADIANS:
                ang_cmd = ANGULAR_SPEED_GAIN * angle_error
                lin_cmd = 0.0
            else:
                lin_cmd = self._mission.speed
                ang_cmd = HEADING_GAIN * angle_error

            lin_cmd = clamp(lin_cmd, self._mission.speed)
            ang_cmd = clamp(ang_cmd, ANGULAR_SPEED_LIMIT_RADIANS)

            half_base = WHEEL_BASE_METERS / 2.0
            left_speed = lin_cmd - (ang_cmd * half_base)
            right_speed = lin_cmd + (ang_cmd * half_base)

            await self._send_command_raw(left_speed, right_speed)

    def _detect_tags_grayscale(self, packet: bytes) -> list[TagDetection]:
        np_buffer = np.frombuffer(packet, dtype=np.uint8)
        gray = np_buffer.reshape((CAMERA_HEIGHT, CAMERA_WIDTH))

        return self._detect_tags_ndarray(gray)

    def _detect_tags_from_jpeg(self, packet: bytes) -> list[TagDetection]:
        """Convert JPEG to grayscale for tag detection."""
        img = self._jpeg.decode(packet, pixel_format=TJPF_GRAY)  # type: ignore

        if len(img.shape) == 3:  # type: ignore
            img = img[:, :, 0]  # type: ignore

        if img.dtype != np.uint8:  # type: ignore
            img = img.astype(np.uint8)  # type: ignore

        return self._detect_tags_ndarray(img)  # type: ignore

    def _detect_tags_ndarray(self, img: np.ndarray[Any, Any]) -> list[TagDetection]:
        detections: list[Detection] = self._detector.detect(  # type: ignore
            img,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=APRILTAG_SIZE_METERS,
        )

        results: list[TagDetection] = []
        for d in detections:
            # Extract Translation (x, y, z)
            # z = forward distance, x = horizontal offset
            t_vec = d.pose_t.flatten()  # type: ignore
            x = t_vec[0]  # type: ignore
            z = t_vec[2]  # type: ignore

            dist = float(np.linalg.norm(t_vec))  # type: ignore
            angle_offset = math.atan2(
                x,  # type: ignore
                z,  # type: ignore
            )  # Angle of tag in camera frame

            # Extract Rotation (Yaw)
            yaw = rotation_matrix_to_yaw(d.pose_R)  # type: ignore

            results.append(
                TagDetection(
                    tag_id=d.tag_id,
                    distance_m=dist,
                    yaw_rel=yaw,
                    angle_offset=angle_offset,
                    center=tuple(d.center.tolist()),  # type: ignore
                )
            )
        return results


robot = Robot()
dashboard_hub = DashboardHub()


@asynccontextmanager
async def lifespan(app: FastAPI):
    await robot.start()
    yield
    await robot.shutdown()


app = FastAPI(lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


async def handle_robot_websocket_not_connected(_: Request, __: Exception) -> Response:
    raise HTTPException(
        status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
        detail="Not connected to robot",
    )


app.add_exception_handler(
    RobotWebSocketNotConnected,
    handle_robot_websocket_not_connected,
)


@app.post("/control/speed")
async def set_speed(cmd: CommandRequest):
    await robot.send_command(cmd)
    return {"status": "ok"}


@app.post("/mission")
async def set_mission(req: MissionRequest):
    return await robot.set_mission(req)


@app.delete("/mission")
async def cancel_mission():
    return await robot.cancel_mission()


@app.websocket("/ws/dashboard")
async def dashboard_ws(ws: WebSocket):
    await ws.accept()
    await dashboard_hub.register(ws)
    try:
        while True:
            await ws.receive_text()
    except Exception as e:
        logger.warning(e)
        await dashboard_hub.unregister(ws)


if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
