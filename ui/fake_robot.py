from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import math
import time
from dataclasses import dataclass

import websockets

WS_URL = "ws://localhost:8000/ws/robot"

# Rough copies of the firmware constants, simplified.
WHEEL_RADIUS_M = 0.04967  # from src/main.cpp (49.67 mm)
CONTROL_PERIOD_MS = 40
DEBUG_PERIOD_MS = 50
TELEMETRY_PERIOD_MS = 500

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())


def monotonic_ms() -> int:
    """Return a monotonic timestamp in milliseconds."""
    return time.time_ns() // 1_000_000


@dataclass
class FakeWheel:
    """Very small wheel model that mimics the firmware PID outputs."""

    ks: float
    kf: float
    kp: float
    ki: float

    target_speed: float = 0.0  # m/s
    speed: float = 0.0  # m/s (simulated measured speed)
    integrator: float = 0.0

    p_term: float = 0.0
    i_term: float = 0.0
    f_term: float = 0.0
    output: float = 0.0  # "pwm" debug value

    rotation_revs: float = 0.0  # total wheel rotations

    def step(self, dt_s: float) -> None:
        """Advance the wheel state by dt_s seconds.

        This is intentionally simple: the speed just exponentially approaches
        the target, and PID-like terms are computed for logging only.
        """
        # First-order lag towards target speed.
        alpha = 0.2
        self.speed += alpha * (self.target_speed - self.speed)

        # Compute very rough PID-like terms (for logging).
        error = abs(self.target_speed) - abs(self.speed)
        if abs(self.target_speed) < 0.01:
            self.integrator = 0.0
        else:
            self.integrator += error * dt_s

        self.f_term = self.kf * abs(self.target_speed)
        self.p_term = self.kp * error
        self.i_term = self.ki * self.integrator
        self.output = self.ks + self.f_term + self.p_term + self.i_term

        # Integrate rotations from linear speed.
        circumference = 2.0 * math.pi * WHEEL_RADIUS_M
        if circumference > 0:
            self.rotation_revs += (self.speed * dt_s) / circumference

    @property
    def rpm(self) -> float:
        """Approximate wheel RPM from linear speed."""
        circumference = 2.0 * math.pi * WHEEL_RADIUS_M
        if circumference <= 0:
            return 0.0
        rev_per_s = self.speed / circumference
        return rev_per_s * 60.0


class FakeRobotClient:
    """Fake robot that talks to the FastAPI server like the real ESP32."""

    def __init__(self) -> None:
        # Tunings roughly match src/main.cpp but exact values don't matter here.
        self.left = FakeWheel(ks=100.0, kf=45.0, kp=120.0, ki=20.0)
        self.right = FakeWheel(ks=140.0, kf=30.0, kp=100.0, ki=5.0)

    async def run_forever(self) -> None:
        """Keep trying to connect, reconnecting on errors."""
        while True:
            try:
                async with websockets.connect(WS_URL) as ws:
                    print(f"[fake_robot] Connected to {WS_URL}")
                    await self._handle_connection(ws)
            except Exception as exc:
                print(f"[fake_robot] Connection error: {exc!r}, retrying in 2s")
                await asyncio.sleep(2.0)

    async def _handle_connection(self, ws: websockets.WebSocketClientProtocol) -> None:
        """Run receiver and sender loops for a single connection."""
        receiver = asyncio.create_task(self._receiver_loop(ws))
        sender = asyncio.create_task(self._sender_loop(ws))
        try:
            await asyncio.gather(receiver, sender)
        finally:
            receiver.cancel()
            sender.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await receiver
                await sender

    async def _receiver_loop(self, ws: websockets.WebSocketClientProtocol) -> None:
        """Listen for speed commands from the server."""
        async for raw in ws:
            if isinstance(raw, bytes):
                # Real robot would send camera frames as bytes; the fake ignores this.
                continue
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if not isinstance(data, dict):
                continue
            if data.get("type") != "command":
                continue

            # Shape matches WheelsCommand in ui/main.py and src/main.cpp
            left = float(data.get("left", 0.0))
            right = float(data.get("right", 0.0))
            self.left.target_speed = left
            self.right.target_speed = right

    async def _sender_loop(self, ws: websockets.WebSocketClientProtocol) -> None:
        """Periodically update the model and send pid_log + telemetry."""
        last_tick_ms = monotonic_ms()
        last_debug_ms = last_tick_ms
        last_telemetry_ms = last_tick_ms

        last_left_rot = self.left.rotation_revs
        last_right_rot = self.right.rotation_revs

        while True:
            now_ms = monotonic_ms()
            dt_ms = max(1, now_ms - last_tick_ms)
            dt_s = dt_ms / 1000.0
            last_tick_ms = now_ms

            # Advance simple wheel model.
            self.left.step(dt_s)
            self.right.step(dt_s)

            # Send pid_log at a higher rate.
            if now_ms - last_debug_ms >= DEBUG_PERIOD_MS:
                last_debug_ms = now_ms
                await self._send_pid_log(ws, now_ms)

            # Send telemetry at a slower rate.
            if now_ms - last_telemetry_ms >= TELEMETRY_PERIOD_MS:
                tel_dt = now_ms - last_telemetry_ms
                last_telemetry_ms = now_ms

                left_delta_rot = self.left.rotation_revs - last_left_rot
                right_delta_rot = self.right.rotation_revs - last_right_rot
                last_left_rot = self.left.rotation_revs
                last_right_rot = self.right.rotation_revs

                await self._send_telemetry(
                    ws,
                    now_ms,
                    tel_dt,
                    left_delta_rot,
                    right_delta_rot,
                )

            await asyncio.sleep(CONTROL_PERIOD_MS / 1000.0)

    async def _send_pid_log(
        self,
        ws: websockets.WebSocketClientProtocol,
        timestamp_ms: int,
    ) -> None:
        """Send a pid_log JSON message that the dashboard expects."""
        msg = {
            "type": "pid_log",
            "ts": timestamp_ms,
            "left": {
                "t": self.left.target_speed,
                "m": self.left.speed,
                "pwm": self.left.output,
                "p": self.left.p_term,
                "i": self.left.i_term,
            },
            "right": {
                "t": self.right.target_speed,
                "m": self.right.speed,
                "pwm": self.right.output,
                "p": self.right.p_term,
                "i": self.right.i_term,
            },
        }
        try:
            await ws.send(json.dumps(msg))
        except Exception:
            # Let the outer loop handle disconnects.
            raise

    async def _send_telemetry(
        self,
        ws: websockets.WebSocketClientProtocol,
        timestamp_ms: int,
        interval_ms: int,
        left_delta_rot: float,
        right_delta_rot: float,
    ) -> None:
        """Send a simple telemetry message compatible with TelemetryIncoming."""
        # Approximate encoder counts from rotations (not strictly needed by server).
        PULSES_PER_ROT = 64.0
        left_delta_count = int(left_delta_rot * PULSES_PER_ROT)
        right_delta_count = int(right_delta_rot * PULSES_PER_ROT)

        payload = {
            "type": "telemetry",
            "timestamp": timestamp_ms,
            "left": {
                "rotation": self.left.rotation_revs,
                "frequency_rpm": self.left.rpm,
                "speed_m_s": self.left.speed,
                "delta_count": left_delta_count,
                "interval_ms": interval_ms,
            },
            "right": {
                "rotation": self.right.rotation_revs,
                "frequency_rpm": self.right.rpm,
                "speed_m_s": self.right.speed,
                "delta_count": right_delta_count,
                "interval_ms": interval_ms,
            },
        }
        try:
            await ws.send(json.dumps(payload))
        except Exception:
            raise


async def main() -> None:
    client = FakeRobotClient()
    await client.run_forever()


if __name__ == "__main__":
    asyncio.run(main())
