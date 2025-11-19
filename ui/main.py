from __future__ import annotations

import asyncio
import math
import struct
import time
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from typing import Literal, NamedTuple

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from pupil_apriltags import Detector
from pydantic import BaseModel, ConfigDict, Field, ValidationError

WHEEL_BASE_METERS = 0.28
FRAME_PACKET_MAGIC = 0x46524D31
FRAME_PACKET_VERSION = 1
FRAME_HEADER_STRUCT = struct.Struct("<I H H I I H H I")
APRILTAG_SIZE_METERS = 0.06
CAMERA_PARAMS = (620.0, 620.0, 320.0, 240.0)


def monotonic_ms() -> int:
    return time.time_ns() // 1_000_000


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class FramePacket(NamedTuple):
    frame_id: int
    timestamp_ms: int
    width: int
    height: int
    data: bytes


class WheelSample(BaseModel):
    rotation_count: float = Field(default=0.0, alias="rotation")
    frequency_rpm: float = 0.0
    speed_m_s: float = 0.0
    delta_count: int = 0
    interval_ms: int = 0

    model_config = ConfigDict(populate_by_name=True)


class Pose(BaseModel):
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class RobotState(BaseModel):
    pose: Pose = Field(default_factory=Pose)
    left: WheelSample = Field(default_factory=WheelSample)
    right: WheelSample = Field(default_factory=WheelSample)
    last_update_ms: int = 0


class TelemetryIncoming(BaseModel):
    type: Literal["telemetry"]
    timestamp: int = Field(default_factory=monotonic_ms)
    left: WheelSample = Field(default_factory=WheelSample)
    right: WheelSample = Field(default_factory=WheelSample)


class WheelsCommand(BaseModel):
    type: Literal["command"] = "command"
    left: float
    right: float


class CommandRequest(BaseModel):
    left: float
    right: float


class CommandResponse(BaseModel):
    status: str
    target: CommandRequest
    timestamp_ms: int = Field(default_factory=monotonic_ms)


class TagDetection(BaseModel):
    tag_id: int
    decision_margin: float
    distance_m: float
    frame_id: int
    timestamp_ms: int
    pose: Pose | None = None


class VisionSnapshot(BaseModel):
    last_frame_timestamp_ms: int
    detections: list[TagDetection]


class MissionRequest(BaseModel):
    pose: Pose
    position_tolerance: float = Field(default=0.03, gt=0)
    heading_tolerance_rad: float = Field(default=math.radians(8.0), gt=0)
    linear_speed: float = Field(default=0.35, gt=0)
    angular_speed: float = Field(default=1.2, gt=0)


class MissionStatus(BaseModel):
    state: Literal["idle", "running", "completed", "error"] = "idle"
    target: Pose | None = None
    started_ms: int | None = None
    last_error: str | None = None


class RobotOfflineError(RuntimeError):
    pass


def parse_frame_packet(payload: bytes) -> FramePacket | None:
    if len(payload) <= FRAME_HEADER_STRUCT.size:
        return None
    (
        magic,
        version,
        _reserved,
        frame_id,
        timestamp_ms,
        width,
        height,
        payload_len,
    ) = FRAME_HEADER_STRUCT.unpack_from(payload, 0)
    if magic != FRAME_PACKET_MAGIC or version != FRAME_PACKET_VERSION:
        return None
    if payload_len != len(payload) - FRAME_HEADER_STRUCT.size:
        return None
    data = payload[FRAME_HEADER_STRUCT.size :]
    return FramePacket(
        frame_id=frame_id,
        timestamp_ms=timestamp_ms,
        width=width,
        height=height,
        data=data,
    )


@dataclass
class WheelSampleState:
    rotation_count: float = 0.0
    frequency_rpm: float = 0.0
    speed_m_s: float = 0.0
    delta_count: int = 0
    interval_ms: int = 0


@dataclass
class PoseState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class RobotStateData:
    pose: PoseState = field(default_factory=PoseState)
    left: WheelSampleState = field(default_factory=WheelSampleState)
    right: WheelSampleState = field(default_factory=WheelSampleState)
    last_update_ms: int = 0


@dataclass
class TagDetectionState:
    tag_id: int
    decision_margin: float
    distance_m: float
    frame_id: int
    timestamp_ms: int
    pose: PoseState | None = None


@dataclass
class MissionPlan:
    pose: PoseState
    position_tolerance: float
    heading_tolerance_rad: float
    linear_speed: float
    angular_speed: float


@dataclass
class MissionStatusState:
    state: Literal["idle", "running", "completed", "error"] = "idle"
    target: PoseState | None = None
    started_ms: int | None = None
    last_error: str | None = None


APRILTAG_ANCHORS: dict[int, PoseState] = {
    1: PoseState(x=0.0, y=0.0, theta=0.0),
}


class Robot:
    def __init__(self) -> None:
        self._state = RobotStateData()
        self._ws: WebSocket | None = None
        self._mission: MissionPlan | None = None
        self._mission_status = MissionStatusState()
        self._mission_task: asyncio.Task[None] | None = None
        self._detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
        )
        self._latest_detections: list[TagDetectionState] = []
        self._last_frame_timestamp = 0

    async def start(self) -> None:
        self._ensure_mission_task()

    async def shutdown(self) -> None:
        if self._mission_task is not None:
            self._mission_task.cancel()
            try:
                await self._mission_task
            except asyncio.CancelledError:
                pass
            self._mission_task = None
        try:
            await self.stop()
        except RobotOfflineError:
            pass

    def state_snapshot(self) -> RobotState:
        return RobotState(
            pose=self._pose_model(self._state.pose),
            left=self._wheel_model(self._state.left),
            right=self._wheel_model(self._state.right),
            last_update_ms=self._state.last_update_ms,
        )

    def vision_snapshot(self) -> VisionSnapshot:
        return VisionSnapshot(
            last_frame_timestamp_ms=self._last_frame_timestamp,
            detections=[self._detection_model(d) for d in self._latest_detections],
        )

    def mission_status(self) -> MissionStatus:
        return MissionStatus(
            state=self._mission_status.state,
            target=self._pose_model(self._mission_status.target)
            if self._mission_status.target is not None
            else None,
            started_ms=self._mission_status.started_ms,
            last_error=self._mission_status.last_error,
        )

    async def set_mission(self, request: MissionRequest) -> MissionStatus:
        self._mission = MissionPlan(
            pose=self._pose_state_from_model(request.pose),
            position_tolerance=request.position_tolerance,
            heading_tolerance_rad=request.heading_tolerance_rad,
            linear_speed=request.linear_speed,
            angular_speed=request.angular_speed,
        )
        self._mission_status = MissionStatusState(
            state="running",
            target=self._pose_clone(self._mission.pose),
            started_ms=monotonic_ms(),
        )
        self._ensure_mission_task()
        return self.mission_status()

    async def cancel_mission(self) -> MissionStatus:
        self._mission = None
        self._mission_status = MissionStatusState(state="idle")
        await self.stop()
        return self.mission_status()

    async def attach_websocket(self, websocket: WebSocket) -> None:
        if self._ws is not None and self._ws is not websocket:
            await self._ws.close()
        self._ws = websocket

    async def detach_websocket(self, websocket: WebSocket | None = None) -> None:
        if websocket is not None and websocket is not self._ws:
            return
        self._ws = None

    async def send_command(self, request: CommandRequest) -> None:
        websocket = self._ws
        if websocket is None:
            raise RobotOfflineError("Robot is not connected")
        message = WheelsCommand(left=request.left, right=request.right)
        await websocket.send_text(message.model_dump_json())

    async def stop(self) -> None:
        await self.send_command(CommandRequest(left=0.0, right=0.0))

    async def handle_text_message(self, raw_message: str) -> None:
        try:
            message = TelemetryIncoming.model_validate_json(raw_message)
        except ValidationError:
            return
        self._apply_telemetry(message)

    async def process_frame(self, payload: bytes) -> None:
        packet = parse_frame_packet(payload)
        if packet is None:
            return
        loop = asyncio.get_running_loop()
        detections = await loop.run_in_executor(None, self._detect_tags, packet)
        self._last_frame_timestamp = packet.timestamp_ms
        self._latest_detections = detections
        self._apply_pose_correction(detections)

    def _ensure_mission_task(self) -> None:
        if self._mission_task is None or self._mission_task.done():
            self._mission_task = asyncio.create_task(self._mission_loop())

    async def _mission_loop(self) -> None:
        try:
            while True:
                plan = self._mission
                if plan is None:
                    await asyncio.sleep(0.1)
                    continue
                await self._advance_mission(plan)
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            raise

    async def _advance_mission(self, plan: MissionPlan) -> None:
        pose = self._state.pose
        dx = plan.pose.x - pose.x
        dy = plan.pose.y - pose.y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_heading - pose.theta),
            math.cos(target_heading - pose.theta),
        )
        if (
            distance <= plan.position_tolerance
            and abs(heading_error) <= plan.heading_tolerance_rad
        ):
            await self.stop()
            self._mission = None
            self._mission_status = MissionStatusState(
                state="completed",
                target=self._pose_clone(plan.pose),
                started_ms=self._mission_status.started_ms,
            )
            return
        linear = clamp(
            plan.linear_speed * distance,
            -plan.linear_speed,
            plan.linear_speed,
        )
        angular = clamp(
            plan.angular_speed * heading_error,
            -plan.angular_speed,
            plan.angular_speed,
        )
        half_base = WHEEL_BASE_METERS / 2.0
        left_speed = clamp(
            linear - angular * half_base,
            -plan.linear_speed,
            plan.linear_speed,
        )
        right_speed = clamp(
            linear + angular * half_base,
            -plan.linear_speed,
            plan.linear_speed,
        )
        try:
            await self.send_command(CommandRequest(left=left_speed, right=right_speed))
        except RobotOfflineError:
            self._mission = None
            self._mission_status = MissionStatusState(
                state="error",
                target=self._pose_clone(plan.pose),
                started_ms=self._mission_status.started_ms,
                last_error="Robot disconnected",
            )

    def _apply_telemetry(self, msg: TelemetryIncoming) -> None:
        current_ms = msg.timestamp
        dt_ms = 0
        if self._state.last_update_ms > 0:
            dt_ms = max(0, current_ms - self._state.last_update_ms)
        self._state.left = self._wheel_state_from_model(msg.left)
        self._state.right = self._wheel_state_from_model(msg.right)
        self._state.last_update_ms = current_ms
        if dt_ms > 0:
            self._integrate_pose(dt_ms, msg.left.speed_m_s, msg.right.speed_m_s)

    def _wheel_state_from_model(self, sample: WheelSample) -> WheelSampleState:
        return WheelSampleState(
            rotation_count=sample.rotation_count,
            frequency_rpm=sample.frequency_rpm,
            speed_m_s=sample.speed_m_s,
            delta_count=sample.delta_count,
            interval_ms=sample.interval_ms,
        )

    def _integrate_pose(
        self, delta_ms: int, left_speed: float, right_speed: float
    ) -> None:
        dt = delta_ms / 1000.0
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (right_speed - left_speed) / WHEEL_BASE_METERS
        pose = self._state.pose
        pose.theta += angular_velocity * dt
        pose.x += linear_velocity * math.cos(pose.theta) * dt
        pose.y += linear_velocity * math.sin(pose.theta) * dt

    def _detect_tags(self, packet: FramePacket) -> list[TagDetectionState]:
        np_buffer = np.frombuffer(packet.data, dtype=np.uint8)
        frame = cv2.imdecode(np_buffer, cv2.IMREAD_COLOR)
        if frame is None:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self._detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=APRILTAG_SIZE_METERS,
        )
        results: list[TagDetectionState] = []
        for det in detections:
            pose_state = self._world_pose_from_detection(det.tag_id, det.pose_t)
            results.append(
                TagDetectionState(
                    tag_id=det.tag_id,
                    decision_margin=float(det.decision_margin),
                    distance_m=float(np.linalg.norm(det.pose_t)),
                    frame_id=packet.frame_id,
                    timestamp_ms=packet.timestamp_ms,
                    pose=pose_state,
                )
            )
        return results

    def _world_pose_from_detection(
        self, tag_id: int, translation: np.ndarray
    ) -> PoseState | None:
        anchor = APRILTAG_ANCHORS.get(tag_id)
        if anchor is None:
            return None
        dx = float(translation[0])
        dz = float(translation[2])
        heading = anchor.theta
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        x = anchor.x - (dz * cos_h - dx * sin_h)
        y = anchor.y - (dz * sin_h + dx * cos_h)
        return PoseState(x=x, y=y, theta=heading)

    def _apply_pose_correction(self, detections: list[TagDetectionState]) -> None:
        for detection in detections:
            if detection.pose is None:
                continue
            self._state.pose = self._pose_clone(detection.pose)
            self._state.last_update_ms = monotonic_ms()
            break

    def _pose_clone(self, pose: PoseState) -> PoseState:
        return PoseState(x=pose.x, y=pose.y, theta=pose.theta)

    def _pose_model(self, pose: PoseState) -> Pose:
        return Pose(x=pose.x, y=pose.y, theta=pose.theta)

    def _wheel_model(self, sample: WheelSampleState) -> WheelSample:
        return WheelSample(
            rotation_count=sample.rotation_count,
            frequency_rpm=sample.frequency_rpm,
            speed_m_s=sample.speed_m_s,
            delta_count=sample.delta_count,
            interval_ms=sample.interval_ms,
        )

    def _detection_model(self, detection: TagDetectionState) -> TagDetection:
        return TagDetection(
            tag_id=detection.tag_id,
            decision_margin=detection.decision_margin,
            distance_m=detection.distance_m,
            frame_id=detection.frame_id,
            timestamp_ms=detection.timestamp_ms,
            pose=self._pose_model(detection.pose)
            if detection.pose is not None
            else None,
        )

    def _pose_state_from_model(self, pose: Pose) -> PoseState:
        return PoseState(x=pose.x, y=pose.y, theta=pose.theta)


robot = Robot()


@asynccontextmanager
async def lifespan(app: FastAPI):
    await robot.start()
    try:
        yield
    finally:
        await robot.shutdown()


app = FastAPI(lifespan=lifespan)


@app.get("/state", response_model=RobotState)
async def get_state() -> RobotState:
    return robot.state_snapshot()


@app.post("/control/speed", response_model=CommandResponse)
async def set_speed(command: CommandRequest) -> CommandResponse:
    try:
        await robot.send_command(command)
    except RobotOfflineError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return CommandResponse(status="sent", target=command)


@app.post("/control/stop")
async def stop_robot() -> dict[str, str]:
    try:
        await robot.stop()
    except RobotOfflineError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"status": "stopped"}


@app.get("/vision/tags", response_model=VisionSnapshot)
async def get_tags() -> VisionSnapshot:
    return robot.vision_snapshot()


@app.get("/mission", response_model=MissionStatus)
async def get_mission() -> MissionStatus:
    return robot.mission_status()


@app.post("/mission", response_model=MissionStatus)
async def create_mission(request: MissionRequest) -> MissionStatus:
    return await robot.set_mission(request)


@app.delete("/mission", response_model=MissionStatus)
async def cancel_mission() -> MissionStatus:
    return await robot.cancel_mission()


@app.websocket("/ws/robot")
async def robot_websocket(websocket: WebSocket) -> None:
    await websocket.accept()
    await robot.attach_websocket(websocket)
    try:
        while True:
            message = await websocket.receive()
            if "text" in message and message["text"] is not None:
                await robot.handle_text_message(message["text"])
            elif "bytes" in message and message["bytes"] is not None:
                await robot.process_frame(message["bytes"])
            elif message.get("type") == "websocket.disconnect":
                break
    except WebSocketDisconnect:
        pass
    finally:
        await robot.detach_websocket(websocket)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
