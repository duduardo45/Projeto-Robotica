import {
	Badge,
	Box,
	Button,
	ChakraProvider,
	Field,
	Flex,
	Grid,
	Heading,
	Input,
	InputAddon,
	InputGroup,
	SimpleGrid,
	Stack,
	Text,
} from "@chakra-ui/react";
import {
	Activity,
	ArrowDown,
	ArrowLeft,
	ArrowRight,
	ArrowUp,
	Square,
	Zap,
} from "lucide-react";
import { useCallback, useEffect, useId, useRef, useState } from "react";
import { CameraFeed, type CameraFeedRef } from "./components/CameraFeed";
import { system } from "./theme";

// --- TYPES ---
interface ChartPoint {
	t: number;
	// Left
	left_target: number;
	left_measured: number;
	left_pwm: number;
	left_p: number;
	left_i: number;
	// Right
	right_target: number;
	right_measured: number;
	right_pwm: number;
	right_p: number;
	right_i: number;
	[key: string]: number;
}

interface MissionConfig {
	x: number;
	y: number;
	speed: number;
}

interface MissionStatus {
	state: "idle" | "running" | "completed" | "error";
	target: { x: number; y: number };
}

// --- UPDATED SCHEMAS ---
interface WheelTelemetry {
	encoder: number;
	raw_speed: number;
	filtered_speed: number;
	target_speed: number;
	debug_f: number;
	debug_p: number;
	debug_i: number;
	debug_out: number;
}

interface TelemetryData {
	message_type: "telemetry";
	timestamp: number;
	left: WheelTelemetry;
	right: WheelTelemetry;
}

interface TagDetection {
	tag_id: number;
	distance_m: number;
	center: [number, number];
}

interface VisionSnapshotData {
	message_type: "vision_snapshot";
	detections: TagDetection[];
}

interface DashboardMessage {
	data: TelemetryData | VisionSnapshotData;
}

// --- CONFIGURATION ---
const WS_URL = "ws://localhost:8000/ws/dashboard";
const API_URL = "http://localhost:8000";
const MAX_POINTS = 600;

// --- Metric Input helper (simplified styling) ---
interface MetricInputProps {
	label: string;
	value: number;
	onChange: (val: number) => void;
	unit: string;
}

const MetricInput = ({ label, value, onChange, unit }: MetricInputProps) => {
	const inputId = useId();
	return (
		<Field.Root>
			<Text textStyle="label" mb={1}>
				{label}
			</Text>
			<InputGroup width="full" endElement={<InputAddon>{unit}</InputAddon>}>
				<Input
					id={inputId}
					type="number"
					value={Number.isNaN(value) ? "" : value}
					onChange={(e) => onChange(parseFloat(e.target.value))}
					variant="outline"
					borderRadius="sm"
					fontFamily="mono"
					borderColor="border.muted"
					h="8"
				/>
			</InputGroup>
		</Field.Root>
	);
};

// --- Chart component (styling simplified) ---
const LiveChart = ({
	dataRef,
	dataKeys,
	colors,
	title,
}: {
	dataRef: React.RefObject<ChartPoint[]>;
	dataKeys: string[];
	colors: string[];
	title: string;
}) => {
	const canvasRef = useRef<HTMLCanvasElement>(null);

	const draw = useCallback(() => {
		const canvas = canvasRef.current;
		if (!canvas) return;
		const ctx = canvas.getContext("2d");
		if (!ctx) return;

		const width = canvas.width;
		const cvsHeight = canvas.height;

		ctx.clearRect(0, 0, width, cvsHeight);

		// Draw Grid (Subtle)
		ctx.strokeStyle = "#27272a";
		ctx.lineWidth = 1;
		ctx.beginPath();
		ctx.moveTo(0, cvsHeight / 2);
		ctx.lineTo(width, cvsHeight / 2);
		ctx.stroke();

		const data = dataRef.current;
		if (!data || data.length < 2) return;

		let min = Infinity;
		let max = -Infinity;
		data.forEach((pt: ChartPoint) => {
			dataKeys.forEach((key: string) => {
				const val = pt[key];
				if (val < min) min = val;
				if (val > max) max = val;
			});
		});
		const range = max - min || 1;
		min -= range * 0.1;
		max += range * 0.1;

		const mapX = (i: number) => (i / (MAX_POINTS - 1)) * width;
		const mapY = (val: number) =>
			cvsHeight - ((val - min) / (max - min)) * cvsHeight;

		dataKeys.forEach((key: string, idx: number) => {
			ctx.beginPath();
			ctx.strokeStyle = colors[idx];
			ctx.lineWidth = 1.5;
			const startIndex = Math.max(0, data.length - MAX_POINTS);
			for (let i = startIndex; i < data.length; i++) {
				const x = mapX(i - startIndex);
				const y = mapY(data[i][key]);
				if (i === startIndex) ctx.moveTo(x, y);
				else ctx.lineTo(x, y);
			}
			ctx.stroke();
		});
	}, [dataRef, dataKeys, colors]);

	useEffect(() => {
		let animId: number;
		const loop = () => {
			draw();
			animId = requestAnimationFrame(loop);
		};
		loop();
		return () => cancelAnimationFrame(animId);
	}, [draw]);

	return (
		<Box layerStyle="panel" h="full">
			<Flex justify="space-between" mb={3}>
				<Text textStyle="label" fontWeight="bold">
					{title}
				</Text>
				<Activity size={14} style={{ opacity: 0.5 }} />
			</Flex>
			<Box layerStyle="readout" h="160px" position="relative" overflow="hidden">
				<canvas
					ref={canvasRef}
					width={800}
					height={160}
					style={{ width: "100%", height: "100%", display: "block" }}
				/>
			</Box>
		</Box>
	);
};

export default function RobotDashboard() {
	const [connected, setConnected] = useState(false);
	const [driveSpeed, setDriveSpeed] = useState(0.3);
	const [missionConfig, setMissionConfig] = useState<MissionConfig>({
		x: 0.5,
		y: 0.0,
		speed: 0.3,
	});
	const [, setMissionStatus] = useState<MissionStatus | null>(null);
	const chartDataRef = useRef<ChartPoint[]>([]);
	const cameraRef = useRef<CameraFeedRef>(null);

	useEffect(() => {
		const ws = new WebSocket(WS_URL);
		ws.binaryType = "arraybuffer";
		let startTime: number | null = null;
		ws.onopen = () => setConnected(true);
		ws.onclose = () => setConnected(false);
		ws.onmessage = (event) => {
			// 1. Binary Frame Handling
			if (event.data instanceof ArrayBuffer) {
				cameraRef.current?.handleFrame(event.data);
				return;
			}

			// 2. JSON Telemetry Handling
			const msg = JSON.parse(event.data) as DashboardMessage;

			// Dispatch based on 'message_type'
			if (msg.data.message_type === "vision_snapshot") {
				const visionData = msg.data as VisionSnapshotData;
				cameraRef.current?.handleDetections(visionData);
			} else if (msg.data.message_type === "telemetry") {
				const telMsg = msg.data as TelemetryData;

				if (startTime === null) startTime = telMsg.timestamp;

				// Map Telemetry keys to Chart keys
				chartDataRef.current.push({
					t: (telMsg.timestamp - (startTime ?? telMsg.timestamp)) / 1_000_000.0, // convert micros to sec

					// Left
					left_target: telMsg.left.target_speed,
					left_measured: telMsg.left.filtered_speed,
					left_pwm: telMsg.left.debug_out,
					left_p: telMsg.left.debug_p,
					left_i: telMsg.left.debug_i,

					// Right
					right_target: telMsg.right.target_speed,
					right_measured: telMsg.right.filtered_speed,
					right_pwm: telMsg.right.debug_out,
					right_p: telMsg.right.debug_p,
					right_i: telMsg.right.debug_i,
				});

				if (chartDataRef.current.length > MAX_POINTS)
					chartDataRef.current.shift();
			}
		};
		return () => ws.close();
	}, []);

	const sendDrive = async (left: number, right: number) => {
		try {
			await fetch(`${API_URL}/control/speed`, {
				method: "POST",
				headers: { "Content-Type": "application/json" },
				body: JSON.stringify({ left, right }),
			});
		} catch {}
	};

	const handleDrive = (dir: string) => {
		const s = Math.abs(driveSpeed);
		const cmds: Record<string, [number, number]> = {
			fwd: [s, s],
			back: [-s, -s],
			left: [-s, s],
			right: [s, -s],
			stop: [0, 0],
		};
		const [l, r] = cmds[dir];
		sendDrive(l, r);
	};

	const startMission = async () => {
		setMissionStatus({
			state: "running",
			target: { x: missionConfig.x, y: missionConfig.y },
		});
		try {
			await fetch(`${API_URL}/mission`, {
				method: "POST",
				headers: { "Content-Type": "application/json" },
				body: JSON.stringify({
					x: missionConfig.x,
					y: missionConfig.y,
					speed: missionConfig.speed,
				}),
			});
		} catch {}
	};

	return (
		<ChakraProvider value={system}>
			<Box minH="100vh" bg="bg.canvas" color="fg.default" p={6}>
				{/* HEADER */}
				<Flex justify="space-between" align="center" mb={8}>
					<Box>
						<Heading size="md" fontFamily="mono" letterSpacing="tighter">
							ROBOT
							<Text as="span" color="fg.muted">
								.CTRL
							</Text>
						</Heading>
					</Box>
					<Badge
						variant="solid"
						colorPalette={connected ? "green" : "red"}
						borderRadius="sm"
					>
						<Zap size={10} style={{ marginRight: 4 }} />
						{connected ? "ONLINE" : "OFFLINE"}
					</Badge>
				</Flex>

				<Grid templateColumns={{ base: "1fr", xl: "1fr 300px" }} gap={6}>
					{/* LEFT: CHARTS */}
					<Stack gap={6}>
						<CameraFeed ref={cameraRef} />
						<Grid templateColumns="1fr 1fr" gap={6}>
							<LiveChart
								title="Left Velocity"
								dataRef={chartDataRef}
								dataKeys={["left_target", "left_measured"]}
								colors={["red", "#00A3C4"]}
							/>
							<LiveChart
								title="Right Velocity"
								dataRef={chartDataRef}
								dataKeys={["right_target", "right_measured"]}
								colors={["red", "#00A3C4"]}
							/>
							<LiveChart
								title="Left PID"
								dataRef={chartDataRef}
								dataKeys={["left_pwm", "left_p", "left_i"]}
								colors={["white", "orange", "purple"]}
							/>
							<LiveChart
								title="Right PID"
								dataRef={chartDataRef}
								dataKeys={["right_pwm", "right_p", "right_i"]}
								colors={["white", "orange", "purple"]}
							/>
						</Grid>

						{/* MISSION PANEL */}
						<Box layerStyle="panel">
							<Flex justify="space-between" mb={4}>
								<Text textStyle="label" fontSize="sm" color="fg.default">
									Mission Config
								</Text>
							</Flex>
							<Flex gap={6}>
								<Grid templateColumns="repeat(3, 1fr)" gap={4} flex={1}>
									<MetricInput
										label="Target X"
										value={missionConfig.x}
										onChange={(v: number) =>
											setMissionConfig({ ...missionConfig, x: v })
										}
										unit="m"
									/>
									<MetricInput
										label="Target Y"
										value={missionConfig.y}
										onChange={(v: number) =>
											setMissionConfig({ ...missionConfig, y: v })
										}
										unit="m"
									/>
									<MetricInput
										label="Speed"
										value={missionConfig.speed}
										onChange={(v: number) =>
											setMissionConfig({ ...missionConfig, speed: v })
										}
										unit="m/s"
									/>
								</Grid>
								<Stack>
									<Button
										layerStyle="ctrlBtn"
										bg="fg.default"
										color="bg.canvas"
										_hover={{ opacity: 0.9 }}
										onClick={startMission}
									>
										INITIATE
									</Button>
									<Button
										layerStyle="ctrlBtn"
										borderColor="red.800"
										color="red.500"
										onClick={() => {
											setMissionStatus(null);
											fetch(`${API_URL}/mission`, { method: "DELETE" });
										}}
									>
										ABORT
									</Button>
								</Stack>
							</Flex>
						</Box>
					</Stack>

					{/* RIGHT: MANUAL CONTROL */}
					<Box layerStyle="panel">
						<Text textStyle="label" mb={6} textAlign="center">
							Manual Override
						</Text>
						<Stack align="center" gap={6}>
							<MetricInput
								label="PWM Limit"
								value={driveSpeed}
								onChange={setDriveSpeed}
								unit="m/s"
							/>
							{/* CONTROLS */}
							<SimpleGrid columns={3} gap={2}>
								<Box />
								<Button
									layerStyle="ctrlBtn"
									onMouseDown={() => handleDrive("fwd")}
									onMouseUp={() => handleDrive("stop")}
								>
									<ArrowUp />
								</Button>
								<Box />

								<Button
									layerStyle="ctrlBtn"
									onMouseDown={() => handleDrive("left")}
									onMouseUp={() => handleDrive("stop")}
								>
									<ArrowLeft />
								</Button>
								<Button
									layerStyle="ctrlBtn"
									color="red.500"
									onClick={() => handleDrive("stop")}
								>
									<Square size={12} fill="currentColor" />
								</Button>
								<Button
									layerStyle="ctrlBtn"
									onMouseDown={() => handleDrive("right")}
									onMouseUp={() => handleDrive("stop")}
								>
									<ArrowRight />
								</Button>

								<Box />
								<Button
									layerStyle="ctrlBtn"
									onMouseDown={() => handleDrive("back")}
									onMouseUp={() => handleDrive("stop")}
								>
									<ArrowDown />
								</Button>
								<Box />
							</SimpleGrid>
							{/* LOG READOUT */}
							<Box layerStyle="readout" w="full" p={3}>
								<Text textStyle="monoVal" fontSize="xs" color="green.400">
									&gt; SYSTEM READY
								</Text>
								<Text textStyle="monoVal" fontSize="xs" color="fg.muted">
									&gt; Waiting for command...
								</Text>
							</Box>
						</Stack>
					</Box>
				</Grid>
			</Box>
		</ChakraProvider>
	);
}
