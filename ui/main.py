from __future__ import annotations

import asyncio
import logging
import math
import time
from asyncio import TaskGroup
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Literal

import numpy as np
import uvicorn
import websockets
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from pupil_apriltags import Detection, Detector
from pydantic import BaseModel, Field
from turbojpeg import TJPF_GRAY, TurboJPEG
from websockets.asyncio.client import ClientConnection

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())

# --- Constants ---

# define FX 509.932 // fx (in pixel)
# define FY 509.932 // fy (in pixel)
# define CX 390.456 // cx (in pixel)
# define CY 176.282 // cy (in pixel)

WHEEL_BASE_METERS = 0.28
APRILTAG_SIZE_METERS = 0.043
CAMERA_PARAMS = (509.932, 509.932, 390.456, 176.282)
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
# Change this to "jpeg" or "grayscale" to switch frame format
FRAME_FORMAT: Literal["jpeg", "grayscale"] = "jpeg"
# valores errados:
WHEEL_RADIUS_METERS = 0.04967
TICKS_PER_METER = 64.0 * (1.0 / 2 * math.pi * WHEEL_RADIUS_METERS)

ARRIVAL_TOLERANCE_METERS = 0.05
HEADING_TOLERANCE_RADIANS = 0.1
ANGULAR_SPEED_LIMIT_RADIANS = 2.0
ANGULAR_SPEED_GAIN = 2.0
HEADING_GAIN = 1.0

MISSION_LOOP_INTERVAL_SECONDS = 0.2

# Robot WebSocket Configuration
ROBOT_WS_URL = "ws://192.168.4.1:8000"
ROBOT_WS_RECONNECT_DELAY = 5.0
HEARTBEAT_INTERVAL_SECONDS = 0.2  # Send heartbeat every 200ms

# --- Helpers ---


def monotonic_micros() -> int:
    return time.time_ns() // 1_000


def clamp(value: float, limit: float) -> float:
    """Simple symmetric clamp."""
    return max(-limit, min(limit, value))


def normalize_angle(angle: float) -> float:
    # Uses atan2 to wrap exactly to (-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))


# --- Data Models (Simplified) ---


@dataclass
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

    pose: Pose
    left: WheelTelemetry
    right: WheelTelemetry
    last_update_micros: int


class TelemetryIncoming(BaseModel):
    message_type: Literal["telemetry"]
    timestamp: int
    left: WheelTelemetry
    right: WheelTelemetry


class WheelsCommand(BaseModel):
    message_type: Literal["command"]
    left: float
    right: float


class Heartbeat(BaseModel):
    message_type: Literal["heartbeat"]


class CommandRequest(BaseModel):
    left: float
    right: float


class CommandResponse(BaseModel):
    status: str


class TagDetection(BaseModel):
    tag_id: int
    distance_m: float
    center: tuple[float, float]


class VisionSnapshot(BaseModel):
    message_type: Literal["vision_snapshot"]
    detections: list[TagDetection]


class MissionRequest(BaseModel):
    """Barebones mission request."""

    x: float
    y: float
    speed: float


class MissionStatus(BaseModel):
    state: Literal["idle", "running", "completed"] = "idle"


class RobotOfflineError(RuntimeError):
    pass


# --- Internal State ---


@dataclass(kw_only=True, slots=True)
class RobotStateData:
    pose: Pose
    left: WheelTelemetry
    right: WheelTelemetry
    last_update_micros: int


@dataclass(kw_only=True, slots=True)
class Mission:
    x: float
    y: float
    speed: float


class DashboardMessage(BaseModel):
    data: TelemetryIncoming | VisionSnapshot = Field(discriminator="message_type")


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
        # Initialize at 0,0,0
        self._state = RobotStateData(
            pose=Pose(0.0, 0.0, 0.0),
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
        self._detector = Detector(families="tag36h11")
        self._latest_detections: list[TagDetection] = []
        self._jpeg = TurboJPEG()

    async def _run_lifecycle(self) -> None:
        """
        Uses TaskGroup to manage the three dependent loops.
        If connection dies, everything cancels, and supervisor restarts them.
        """
        async with TaskGroup() as tg:
            tg.create_task(self._robot_ws_client_loop())
            tg.create_task(self._mission_control_loop())
            tg.create_task(self._heartbeat_loop())

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

    def state_snapshot(self) -> RobotState:
        return RobotState(
            pose=self._state.pose,
            left=self._state.left,
            right=self._state.right,
            last_update_micros=self._state.last_update_micros,
        )

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
        if self._ws_client:
            msg = WheelsCommand(message_type="command", left=left, right=right)
            await self._ws_client.send(msg.model_dump_json())

    async def send_command(self, request: CommandRequest) -> None:
        if self._ws_client:
            if self._mission:
                await self.cancel_mission()
            await self._send_command_raw(request.left, request.right)

    async def stop(self) -> None:
        if self._ws_client:
            if self._mission:
                # Don't call cancel_mission here to avoid circular logic in shutdown
                self._mission = None
            await self._send_command_raw(0.0, 0.0)

    # --- Simplified Odometry Here ---
    def apply_telemetry(self, msg: TelemetryIncoming) -> None:
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

        # 5. Update Pose (trigonometry)
        # We use the average theta during the movement for better accuracy (Runge-Kutta 2nd order approximation)
        avg_theta = self._state.pose.theta + (d_theta / 2.0)

        self._state.pose.x += d_center * math.cos(avg_theta)
        self._state.pose.y += d_center * math.sin(avg_theta)
        self._state.pose.theta = normalize_angle(self._state.pose.theta + d_theta)

        # 6. Save state for next loop
        self._state.left = msg.left
        self._state.right = msg.right
        self._state.last_update_micros = msg.timestamp

    async def process_frame(self, payload: bytes) -> None:
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

        await dashboard_hub.broadcast_dashboard_message(
            DashboardMessage(
                data=VisionSnapshot(
                    message_type="vision_snapshot", detections=tag_detections
                ),
            )
        )

    async def _robot_ws_client_loop(self) -> None:
        """Connects to the robot websocket server and handles incoming messages."""
        while True:
            logger.info(f"Connecting to robot at {ROBOT_WS_URL}...")
            try:
                async with websockets.connect(ROBOT_WS_URL) as ws:
                    self._ws_client = ws
                    logger.info("Connected to robot")
                    try:
                        async for message in ws:
                            if isinstance(message, str):
                                # JSON telemetry message
                                try:
                                    telemetry = TelemetryIncoming.model_validate_json(
                                        message
                                    )
                                except Exception as e:
                                    logger.warning(f"Failed to parse telemetry: {e}")
                                else:
                                    self.apply_telemetry(telemetry)
                                    await dashboard_hub.broadcast_dashboard_message(
                                        DashboardMessage(data=telemetry)
                                    )
                            elif isinstance(message, bytes):
                                # Binary frame data
                                await self.process_frame(message)
                    except websockets.exceptions.ConnectionClosed:
                        logger.info("Robot connection closed")
                    finally:
                        self._ws_client = None
            except TimeoutError:
                logger.error("Robot connection timed out")
                self._ws_client = None
                await asyncio.sleep(ROBOT_WS_RECONNECT_DELAY)

    async def _heartbeat_loop(self) -> None:
        """Sends periodic heartbeat messages to keep the robot alive."""
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL_SECONDS)
            if self._ws_client:
                msg = Heartbeat(message_type="heartbeat")
                await self._ws_client.send(msg.model_dump_json())

    # --- Simplified Mission Logic Here ---
    async def _mission_control_loop(self) -> None:
        while True:
            if not self._mission:
                await asyncio.sleep(0.1)
                continue
            # We check self._mission again because of the await above
            if self._mission:
                pose = self._state.pose
                dx = self._mission.x - pose.x
                dy = self._mission.y - pose.y
                dist = math.hypot(dx, dy)

                if dist < ARRIVAL_TOLERANCE_METERS:
                    await self.stop()
                    self._mission = None
                    logger.info("Mission Completed")
                    continue

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

                try:
                    await self._send_command_raw(left_speed, right_speed)
                except RobotOfflineError:
                    self._mission = None

            await asyncio.sleep(MISSION_LOOP_INTERVAL_SECONDS)

    def _detect_tags_grayscale(self, packet: bytes) -> list[TagDetection]:
        np_buffer = np.frombuffer(packet, dtype=np.uint8)
        try:
            gray = np_buffer.reshape((CAMERA_HEIGHT, CAMERA_WIDTH))
        except ValueError:
            return []

        detections: list[Detection] = self._detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=APRILTAG_SIZE_METERS,
        )

        return [
            TagDetection(
                tag_id=d.tag_id,
                distance_m=float(np.linalg.norm(d.pose_t)),
                center=tuple(d.center.tolist()),
            )
            for d in detections
        ]

    def _detect_tags_from_jpeg(self, packet: bytes) -> list[TagDetection]:
        """Convert JPEG to grayscale for tag detection."""
        img = self._jpeg.decode(packet, pixel_format=TJPF_GRAY)

        if len(img.shape) == 3:
            img = img[:, :, 0]

        if img.dtype != np.uint8:
            img = img.astype(np.uint8)
        detections: list[Detection] = self._detector.detect(
            img,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=APRILTAG_SIZE_METERS,
        )
        return [
            TagDetection(
                tag_id=d.tag_id,
                distance_m=float(np.linalg.norm(d.pose_t)),
                center=tuple(d.center.tolist()),
            )
            for d in detections
        ]


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


@app.get("/state", response_model=RobotState)
async def get_state() -> RobotState:
    return robot.state_snapshot()


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
