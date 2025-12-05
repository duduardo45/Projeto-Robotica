import { Box, Flex, Text } from "@chakra-ui/react";
import { Camera } from "lucide-react";
import { forwardRef, useImperativeHandle, useRef } from "react";

interface TagDetection {
    tag_id: number;
    distance_m: number;
    center: [number, number];
}

export interface CameraFeedRef {
    handleFrame: (data: ArrayBuffer) => void;
    handleDetections: (data: { detections: TagDetection[] }) => void;
}

export const CameraFeed = forwardRef<CameraFeedRef, object>((_, ref) => {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const detectionsRef = useRef<TagDetection[]>([]);

    useImperativeHandle(ref, () => ({
        handleDetections: (data: { detections: TagDetection[] }) => {
            detectionsRef.current = data.detections || [];
        },
        handleFrame: (buffer: ArrayBuffer) => {
            const canvas = canvasRef.current;
            if (!canvas) return;
            const ctx = canvas.getContext("2d");
            if (!ctx) return;

            // 2. RENDER JPEG
            // The backend sends raw JPEG bytes
            const blob = new Blob([buffer], { type: "image/jpeg" });
            const img = new Image();
            
            img.onload = () => {
                // Clear and draw image
                if (canvas.width !== img.width || canvas.height !== img.height) {
                    canvas.width = img.width;
                    canvas.height = img.height;
                }
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                ctx.drawImage(img, 0, 0);
                URL.revokeObjectURL(img.src); // Cleanup memory

                // 3. OVERLAY DETECTIONS (Draw AFTER image loads)
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

                        // Draw center
                        ctx.beginPath();
                        ctx.arc(cX, cY, 4, 0, 2 * Math.PI);
                        ctx.fill();

                        // Draw Info
                        ctx.fillText(`ID: ${id}`, cX + 8, cY - 8);
                        if (dist !== undefined) {
                            ctx.fillText(`${dist.toFixed(2)}m`, cX + 8, cY + 8);
                        }
                    });
                }
            };
            
            img.src = URL.createObjectURL(blob);
        },
    }));

    return (
        <Box layerStyle="panel" h="full" minH="300px" display="flex" flexDirection="column">
            <Flex justify="space-between" mb={3}>
                <Text textStyle="label" fontWeight="bold">
                    CAMERA FEED
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
                <canvas
                    ref={canvasRef}
                    style={{ maxWidth: "100%", maxHeight: "100%", objectFit: "contain" }}
                />
            </Box>
        </Box>
    );
});