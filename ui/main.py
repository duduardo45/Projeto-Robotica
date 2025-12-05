from __future__ import annotations

import asyncio
import logging
import math
import struct
import time
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Literal

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pupil_apriltags import Detection, Detector
from pydantic import BaseModel, Field

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())

# --- Constants ---

WHEEL_BASE_METERS = 0.28
FRAME_PACKET_MAGIC = 0x46524D31
FRAME_PACKET_VERSION = 1
FRAME_HEADER_STRUCT = struct.Struct("<I H H I I H H I")
APRILTAG_SIZE_METERS = 0.06
CAMERA_PARAMS = (620.0, 620.0, 320.0, 240.0)
# valores errados:
WHEEL_RADIUS_METERS = 0.04967
TICKS_PER_METER = 64.0 * (1.0 / 2 * math.pi * WHEEL_RADIUS_METERS)

ARRIVAL_TOLERANCE_METERS = 0.05
HEADING_TOLERANCE_RADIANS = 0.1
ANGULAR_SPEED_LIMIT_RADIANS = 2.0
ANGULAR_SPEED_GAIN = 2.0
HEADING_GAIN = 1.0

MISSION_LOOP_INTERVAL_SECONDS = 0.2

# --- Helpers ---


def monotonic_micros() -> int:
    return time.time_ns() // 1_000


def clamp(value: float, limit: float) -> float:
    """Simple symmetric clamp."""
    return max(-limit, min(limit, value))


def normalize_angle(angle: float) -> float:
    """Keeps angle between -pi and pi."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# --- Data Models (Simplified) ---


@dataclass
class Pose:
    x: float
    y: float
    theta: float


@dataclass(kw_only=True, slots=True, frozen=True)
class FramePacket:
    frame_id: int
    timestamp_micros: int
    width: int
    height: int
    data: bytes


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
    left_target_speed: float
    right_target_speed: float


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


def parse_frame_packet(payload: bytes) -> FramePacket | None:
    if len(payload) <= FRAME_HEADER_STRUCT.size:
        return None
    (magic, version, _, frame_id, ts, w, h, p_len) = FRAME_HEADER_STRUCT.unpack_from(
        payload, 0
    )
    if (
        magic != FRAME_PACKET_MAGIC
        or version != FRAME_PACKET_VERSION
        or p_len != len(payload) - FRAME_HEADER_STRUCT.size
    ):
        return None
    return FramePacket(
        frame_id=frame_id,
        timestamp_micros=ts,
        width=w,
        height=h,
        data=payload[FRAME_HEADER_STRUCT.size :],
    )


# --- Internal State ---


@dataclass(kw_only=True, slots=True)
class RobotStateData:
    pose: Pose
    left: WheelTelemetry
    right: WheelTelemetry
    last_update_micros: int


@dataclass(kw_only=True, slots=True)
class MissionPlan:
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
                await ws.send_json(message.model_dump(mode="json"))
            except Exception:
                await self.unregister(ws)

    async def broadcast_frame_packet(self, data: FramePacket) -> None:
        for ws in list(self._connections):
            try:
                await ws.send_bytes(data.data)
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
        self._ws: WebSocket | None = None
        self._mission: MissionPlan | None = None
        self._mission_task: asyncio.Task[None] | None = None

        # Reduced detector settings for speed
        self._detector = Detector(families="tag36h11")
        self._latest_detections: list[TagDetection] = []
        self._last_frame_timestamp_micros = 0

    async def start(self) -> None:
        self._ensure_mission_task()

    async def shutdown(self) -> None:
        if self._mission_task:
            self._mission_task.cancel()
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
        self._mission = MissionPlan(
            x=request.x,
            y=request.y,
            speed=request.speed,
        )
        self._ensure_mission_task()
        return self.mission_status()

    async def cancel_mission(self) -> MissionStatus:
        self._mission = None
        await self.stop()
        return self.mission_status()

    async def attach_websocket(self, websocket: WebSocket) -> None:
        self._ws = websocket

    async def detach_websocket(self, websocket: WebSocket | None = None) -> None:
        self._ws = None

    async def send_command(self, request: CommandRequest) -> None:
        if self._ws:
            if self._mission:
                await self.cancel_mission()
            msg = WheelsCommand(
                message_type="command",
                left_target_speed=request.left,
                right_target_speed=request.right,
            )
            await self._ws.send_json(msg.model_dump(mode="json"))

    async def stop(self) -> None:
        if self._ws:
            if self._mission:
                await self.cancel_mission()
            msg = WheelsCommand(
                message_type="command",
                left_target_speed=0.0,
                right_target_speed=0.0,
            )
            await self._ws.send_json(msg.model_dump(mode="json"))

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
        packet = parse_frame_packet(payload)
        if not packet:
            return

        await dashboard_hub.broadcast_frame_packet(packet)

        loop = asyncio.get_running_loop()
        # Don't block the event loop with vision processing
        detections: list[TagDetection] = await loop.run_in_executor(
            None, self._detect_tags, packet
        )
        if detections:
            logger.debug(f"Detected {len(detections)} tags")

        self._last_frame_timestamp_micros = packet.timestamp_micros

        await dashboard_hub.broadcast_dashboard_message(
            DashboardMessage(
                data=VisionSnapshot(
                    message_type="vision_snapshot", detections=detections
                ),
            )
        )

    def _ensure_mission_task(self) -> None:
        if self._mission_task is None or self._mission_task.done():
            self._mission_task = asyncio.create_task(self._mission_loop())

    # --- Simplified Mission Logic Here ---
    async def _mission_loop(self) -> None:
        while True:
            if not self._mission:
                await asyncio.sleep(0.1)
                continue

            # Simple Proportional Control Logic
            pose = self._state.pose
            dx = self._mission.x - pose.x
            dy = self._mission.y - pose.y
            dist = math.hypot(dx, dy)

            # 1. Check if we arrived
            if dist < ARRIVAL_TOLERANCE_METERS:
                await self.stop()
                self._mission = None
                logger.info("Mission Completed")
                continue

            # 2. Calculate Heading Error
            target_angle = math.atan2(dy, dx)
            angle_error = normalize_angle(target_angle - pose.theta)

            # 3. Determine Wheel Speeds
            # Strategy: Rotate until aligned, then move forward
            lin_cmd = 0.0
            ang_cmd = 0.0

            if abs(angle_error) > HEADING_TOLERANCE_RADIANS:
                # Rotate in place
                ang_cmd = ANGULAR_SPEED_GAIN * angle_error
                lin_cmd = 0.0
            else:
                # Move forward towards target
                lin_cmd = self._mission.speed
                # Maintain heading
                ang_cmd = HEADING_GAIN * angle_error

            # 4. Clamp output
            lin_cmd = clamp(lin_cmd, self._mission.speed)
            ang_cmd = clamp(ang_cmd, ANGULAR_SPEED_LIMIT_RADIANS)

            # 5. Convert Unicycle (v, w) to Differential (vl, vr)
            # left = v - (w * base / 2)
            # right = v + (w * base / 2)
            half_base = WHEEL_BASE_METERS / 2.0
            left_speed = lin_cmd - (ang_cmd * half_base)
            right_speed = lin_cmd + (ang_cmd * half_base)

            try:
                await self.send_command(
                    CommandRequest(left=left_speed, right=right_speed)
                )
            except RobotOfflineError:
                self._mission = None  # Abort

            await asyncio.sleep(MISSION_LOOP_INTERVAL_SECONDS)

    def _detect_tags(self, packet: FramePacket) -> list[TagDetection]:
        np_buffer = np.frombuffer(packet.data, dtype=np.uint8)
        try:
            gray = np_buffer.reshape((packet.height, packet.width))
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


@app.websocket("/ws/robot")
async def robot_ws(ws: WebSocket):
    await ws.accept()
    await robot.attach_websocket(ws)
    try:
        while True:
            msg = await ws.receive()
            if "text" in msg:
                telemetry = TelemetryIncoming.model_validate_json(msg["text"])
                robot.apply_telemetry(telemetry)
                await dashboard_hub.broadcast_dashboard_message(
                    DashboardMessage(data=telemetry)
                )
            elif "bytes" in msg:
                await robot.process_frame(msg["bytes"])
            elif msg.get("type") == "websocket.disconnect":
                break
            else:
                logger.warning(f"Unknown message type: {msg}")
    except WebSocketDisconnect:
        pass
    finally:
        await robot.detach_websocket(ws)


@app.websocket("/ws/dashboard")
async def dashboard_ws(ws: WebSocket):
    await ws.accept()
    await dashboard_hub.register(ws)
    try:
        while True:
            await ws.receive_text()
    except Exception:
        await dashboard_hub.unregister(ws)


if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
