import { Box, Flex, Text } from "@chakra-ui/react";
import { Camera } from "lucide-react";
import { forwardRef, useImperativeHandle, useRef } from "react";
import type { TagDetection } from "../types";

export interface CameraFeedRef {
	handleFrame: (
		data: ArrayBuffer,
		width: number,
		height: number,
		format: "jpeg" | "grayscale",
	) => void;
	handleDetections: (data: { detections: TagDetection[] }) => void;
}

export const CameraFeed = forwardRef<CameraFeedRef, object>((_, ref) => {
	const canvasRef = useRef<HTMLCanvasElement>(null);
	const detectionsRef = useRef<TagDetection[]>([]);

	useImperativeHandle(ref, () => ({
		handleDetections: (data: { detections: TagDetection[] }) => {
			detectionsRef.current = data.detections || [];
		},

		handleFrame: (
			buffer: ArrayBuffer,
			width: number,
			height: number,
			format: "jpeg" | "grayscale",
		) => {
			const canvas = canvasRef.current;
			if (!canvas) return;
			const ctx = canvas.getContext("2d", { alpha: false });
			if (!ctx) return;

			// Setup canvas dimensions
			if (canvas.width !== width || canvas.height !== height) {
				canvas.width = width;
				canvas.height = height;
			}

			if (format === "jpeg") {
				// Handle JPEG format
				const blob = new Blob([buffer], { type: "image/jpeg" });
				const url = URL.createObjectURL(blob);
				const img = new Image();
				img.onload = () => {
					ctx.clearRect(0, 0, width, height);
					ctx.drawImage(img, 0, 0, width, height);
					URL.revokeObjectURL(url);
					drawDetections(ctx);
				};
				img.onerror = () => {
					URL.revokeObjectURL(url);
					console.error("Failed to decode JPEG frame");
				};
				img.src = url;
			} else {
				// Handle grayscale format
				const rawData = new Uint8Array(buffer);
				const imageData = ctx.createImageData(width, height);
				const rgbaData = imageData.data;

				for (let i = 0; i < rawData.length; i++) {
					const pixelValue = rawData[i];
					const offset = i * 4;
					rgbaData[offset] = pixelValue;
					rgbaData[offset + 1] = pixelValue;
					rgbaData[offset + 2] = pixelValue;
					rgbaData[offset + 3] = 255;
				}

				ctx.putImageData(imageData, 0, 0);
				drawDetections(ctx);
			}
		},
	}));

	const drawDetections = (ctx: CanvasRenderingContext2D) => {
		const dets = detectionsRef.current;
		if (dets && dets.length > 0) {
			ctx.lineWidth = 2;
			ctx.font = "bold 14px monospace";
			ctx.strokeStyle = "#00ff00";
			ctx.fillStyle = "#00ff00";

			dets.forEach((det) => {
				const [cX, cY] = det.center;
				const id = det.tag_id;
				const dist = det.distance_m;
				const yaw = det.yaw_rel;
				const angleOffset = det.angle_offset;

				ctx.beginPath();
				ctx.arc(cX, cY, 4, 0, 2 * Math.PI);
				ctx.fill();

				ctx.fillText(`ID: ${id}`, cX + 10, cY - 10);
				if (dist !== undefined) {
					ctx.fillText(`${dist.toFixed(2)}m`, cX + 10, cY + 10);
				}
				if (yaw !== undefined) {
					ctx.fillText(`Yaw: ${yaw.toFixed(2)}°`, cX + 10, cY + 30);
				}
				if (angleOffset !== undefined) {
					ctx.fillText(`Offset: ${angleOffset.toFixed(2)}°`, cX + 10, cY + 50);
				}
			});
		}
	};

	return (
		<Box
			layerStyle="panel"
			h="full"
			minH="300px"
			display="flex"
			flexDirection="column"
		>
			<Flex justify="space-between" mb={3}>
				<Text textStyle="label" fontWeight="bold">
					ROBOT EYE
				</Text>
				<Camera size={14} style={{ opacity: 0.5 }} />
			</Flex>
			<Box
				flex={1}
				bg="black"
				borderRadius="xs"
				overflow="hidden"
				position="relative"
				display="flex"
				alignItems="center"
				justifyContent="center"
			>
				{/* Pixelated rendering helps debug raw sensor data. 
                  Remove 'imageRendering' if you want it smoothed.
                */}
				<canvas
					ref={canvasRef}
					style={{
						maxWidth: "100%",
						maxHeight: "100%",
						objectFit: "contain",
						imageRendering: "pixelated",
					}}
				/>
			</Box>
		</Box>
	);
});
