"""相机畸变校正：加载 calib_solve.py 的输出, 提供高效的 remap 接口。

性能：cv2.initUndistortRectifyMap 一次预计算两张映射表 (mapx, mapy),
之后每帧 cv2.remap 在 Pi 5 上 ≈ 1-2ms, 不影响 120fps。

用法：
    undist = Undistorter.load("calib_params.npz", frame_size=(640, 480))
    if undist is not None:
        frame = undist.apply(frame)
"""
from pathlib import Path
from typing import Any, Optional, Tuple, Union


class Undistorter:
    """加载相机内参 + 畸变, 把每帧 BGR 图像映射成 undistorted 版本。"""

    def __init__(self, camera_matrix, dist_coeffs, frame_size: Tuple[int, int]) -> None:
        import cv2  # type: ignore
        import numpy as np  # type: ignore

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.frame_size = frame_size  # (W, H)

        # alpha=0: 裁掉黑边, undistorted 图所有像素都来自原图有效区
        # alpha=1: 保留全部像素, 边缘可能有黑边 (我们不需要)
        new_K, _ = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, frame_size, alpha=0.0, newImgSize=frame_size
        )
        self.new_camera_matrix = new_K

        # 预计算映射表 (mapx, mapy 都是 H×W float32 / int16, 约 2MB 内存)
        self._mapx, self._mapy = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, new_K, frame_size, cv2.CV_16SC2
        )

    def apply(self, frame: Any) -> Any:
        """对 BGR 帧做畸变校正, 返回新图。原图不变。"""
        import cv2  # type: ignore
        return cv2.remap(frame, self._mapx, self._mapy, interpolation=cv2.INTER_LINEAR)

    @property
    def fx(self) -> float:
        return float(self.new_camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        return float(self.new_camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        return float(self.new_camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        return float(self.new_camera_matrix[1, 2])

    @property
    def focal_px(self) -> float:
        """K2 ECG 模式像素→角度换算用的等效焦距, 取 fx/fy 平均。"""
        return 0.5 * (self.fx + self.fy)

    @classmethod
    def load(
        cls, path: Union[str, Path], frame_size: Tuple[int, int]
    ) -> Optional["Undistorter"]:
        """从 calib_params.npz 加载。文件不存在或损坏时返回 None。"""
        import numpy as np  # type: ignore

        p = Path(path)
        if not p.exists():
            return None
        try:
            data = np.load(str(p))
            K = data["camera_matrix"]
            dist = data["dist_coeffs"]
        except Exception as exc:
            print(f"[undistort] load failed: {exc}", flush=True)
            return None

        # 标定时记录的图像尺寸, 不一致就告警 (运行时会按 frame_size 重新算 newK)
        try:
            calib_size = tuple(int(v) for v in data["image_size"])
            if tuple(calib_size) != tuple(frame_size):
                print(
                    f"[undistort] WARN: calib image_size {calib_size} != runtime {frame_size}, "
                    "畸变校正质量可能下降, 建议用同样分辨率重新标定。",
                    flush=True,
                )
        except KeyError:
            pass

        return cls(K, dist, frame_size)
