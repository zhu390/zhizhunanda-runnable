"""Build no-mole baseline face models from captured frames.

The model stores a per-view median image and a variability/noise image.  Mole
detection can then prefer pixels that became darker than the stable no-mole
face instead of relying only on absolute darkness.
"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


DEFAULT_RAW_DIR = "/home/jetson/jixiebi/calibration/baseline_face/raw"
DEFAULT_MODEL_DIR = "/home/jetson/jixiebi/calibration/baseline_face/model"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成无黑痣人脸基准模型")
    parser.add_argument("--raw-dir", default=DEFAULT_RAW_DIR)
    parser.add_argument("--model-dir", default=DEFAULT_MODEL_DIR)
    parser.add_argument(
        "--views",
        nargs="+",
        default=["front", "left", "right"],
        choices=["front", "left", "right", "scan", "other"],
    )
    parser.add_argument("--prefix", default="baseline")
    parser.add_argument("--min-count", type=int, default=8)
    return parser.parse_args()


def _load_view_images(raw_dir: Path, view: str, prefix: str):
    files = sorted((raw_dir / view).glob("{}_{}_*.png".format(prefix, view)))
    frames = []
    shape = None
    for path in files:
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        if shape is None:
            shape = frame.shape
        if frame.shape != shape:
            raise ValueError(
                "{} shape {} does not match first shape {}".format(
                    path,
                    frame.shape,
                    shape,
                )
            )
        frames.append(frame)
    return files, frames


def _build_view_model(frames):
    stack = np.stack(frames, axis=0).astype(np.float32)
    median = np.median(stack, axis=0)
    abs_dev = np.abs(stack - median)
    noise = np.percentile(abs_dev, 90, axis=0)
    return (
        np.clip(np.rint(median), 0, 255).astype(np.uint8),
        np.clip(np.rint(noise), 0, 255).astype(np.uint8),
    )


def main() -> int:
    args = parse_args()
    raw_dir = Path(args.raw_dir)
    model_dir = Path(args.model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "raw_dir": str(raw_dir),
        "views": {},
    }
    for view in args.views:
        files, frames = _load_view_images(raw_dir, view, args.prefix)
        if len(frames) < int(args.min_count):
            print(
                "{}: 跳过，只有 {} 张，少于 min-count={}".format(
                    view,
                    len(frames),
                    int(args.min_count),
                ),
                flush=True,
            )
            continue
        median, noise = _build_view_model(frames)
        median_path = model_dir / "{}_median.png".format(view)
        noise_path = model_dir / "{}_noise.png".format(view)
        cv2.imwrite(str(median_path), median)
        cv2.imwrite(str(noise_path), noise)
        manifest["views"][view] = {
            "count": len(frames),
            "resolution": [int(median.shape[1]), int(median.shape[0])],
            "median": str(median_path),
            "noise": str(noise_path),
            "first": str(files[0]),
            "last": str(files[-1]),
        }
        print(
            "{}: 已生成 {} / {}，样本 {} 张".format(
                view,
                median_path,
                noise_path,
                len(frames),
            ),
            flush=True,
        )

    manifest_path = model_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("manifest: {}".format(manifest_path), flush=True)
    return 0 if manifest["views"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
