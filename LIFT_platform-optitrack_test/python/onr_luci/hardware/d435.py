''' interface to realsense d435 camera '''
from typing import Tuple
from dataclasses import dataclass, field

import numpy as np
import pyrealsense2 as rs
try:
    rs.pipeline
except AttributeError:
    # import fix for source builds
    import pyrealsense2.pyrealsense2 as rs


from onr_luci.pytypes import PythonMsg


@dataclass
class CameraIntrinsics(PythonMsg):
    ''' camera intrinsics '''
    fx: float = field(default=385)
    fy: float = field(default=385)
    ppx: float = field(default = 318)
    ppy: float = field(default = 240)
    width: int = field(default = 640)
    height: int = field(default = 480)


class D435:
    ''' d435 camera interface '''
    depth_intrinsics: CameraIntrinsics
    _depth_scale: float

    def __init__(self):
        pipe = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 960, 540, rs.format.rgb8, 60)
        config.enable_stream(rs.stream.accel,rs.format.motion_xyz32f, 250)
        config.enable_stream(rs.stream.gyro,rs.format.motion_xyz32f, 200)

        cfg = pipe.start(config)

        profile = cfg.get_stream(rs.stream.depth)
        depth_intrinsics = profile.as_video_stream_profile().get_intrinsics()
        self.depth_intrinsics = CameraIntrinsics(
            width = depth_intrinsics.width,
            height = depth_intrinsics.height,
            ppx = depth_intrinsics.ppx,
            ppy = depth_intrinsics.ppy,
            fx = depth_intrinsics.fx,
            fy = depth_intrinsics.fy,
        )
        self._depth_scale = cfg.get_device().first_depth_sensor().get_depth_scale()

        self._camera = pipe
        self._filters = [
            rs.decimation_filter(),
            rs.threshold_filter(),
            rs.disparity_transform(True),
            rs.spatial_filter(),
            rs.temporal_filter(),
            rs.disparity_transform(False),
            rs.hole_filling_filter(),
        ]

    def step(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        ''' await and return camera frame '''
        frames = self._camera.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if depth_frame:
            for depth_filter in self._filters:
                depth_frame = depth_filter.process(depth_frame)
            depth_image = np.asanyarray(depth_frame.get_data()) * self._depth_scale
        else:
            depth_image = None

        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
        else:
            color_image = None

        accel_frame = frames[2]
        gyro_frame  = frames[3]

        accel_data = accel_frame.as_motion_frame().get_motion_data()
        gyro_data  = gyro_frame.as_motion_frame().get_motion_data()

        accel = np.asanyarray([accel_data.z, -accel_data.x, -accel_data.y])
        gyro  = np.asanyarray([gyro_data.z,  -gyro_data.x,  -gyro_data.y])

        return depth_image, color_image, accel, gyro
