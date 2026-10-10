"""Native-disparity smoothing for display, without replacing measurement depth."""

import pyrealsense2 as rs


class DepthPreviewFilter:
    def __init__(self, mode="spatial"):
        if mode not in ("off", "spatial", "temporal"):
            raise ValueError("Unknown depth preview filter")
        self.mode = mode
        self.blocks = []
        if mode != "off":
            spatial = rs.spatial_filter()
            spatial.set_option(rs.option.filter_magnitude, 2)
            spatial.set_option(rs.option.filter_smooth_alpha, 0.6)
            spatial.set_option(rs.option.filter_smooth_delta, 8)
            spatial.set_option(rs.option.holes_fill, 0)
            self.blocks = [rs.disparity_transform(True), spatial]
            if mode == "temporal":
                temporal = rs.temporal_filter()
                temporal.set_option(rs.option.filter_smooth_alpha, 0.6)
                temporal.set_option(rs.option.filter_smooth_delta, 8)
                temporal.set_option(rs.option.holes_fill, 0)
                self.blocks.append(temporal)
            self.blocks.append(rs.disparity_transform(False))
        self.align = rs.align(rs.stream.color)

    def process(self, frames):
        processed = frames
        for block in self.blocks:
            processed = block.process(processed)
        return self.align.process(processed.as_frameset()).get_depth_frame()

    def describe(self):
        return {
            "mode": self.mode, "purpose": "display only; XYZ uses unfiltered aligned depth",
            "order": "native depth -> disparity -> spatial -> optional temporal -> depth -> align to color",
            "spatial_iterations": 2, "smooth_alpha": 0.6, "smooth_delta": 8,
            "hole_filling": False, "temporal_persistence": False, "decimation": False,
        }
