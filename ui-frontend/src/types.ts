export interface TagDetection {
	tag_id: number;
	distance_m: number;
	center: [number, number];
	yaw_rel: number;
	angle_offset: number;
}
