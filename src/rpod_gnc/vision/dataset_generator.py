"""Generate a minimal OpenGL SPEED-style CubeSat pose dataset."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from .camera import PinholeCamera
from .geometry import CubeSatModel, project_points, quaternion_to_rvec, random_quaternion
from .opengl_renderer import OpenGLCubeSatRenderer


def sample_pose(rng: np.random.Generator, camera: PinholeCamera) -> Dict[str, Any]:
    z = float(rng.uniform(1.5, 15.0))
    x_limit = z * np.tan(np.deg2rad(camera.horizontal_fov_deg) / 2.0) * 0.55
    y_limit = x_limit * camera.height / camera.width
    tvec = np.array([rng.uniform(-x_limit, x_limit), rng.uniform(-y_limit, y_limit), z], dtype=np.float64)
    q = random_quaternion(rng)
    return {"translation_m": tvec, "quaternion_wxyz": q}


def generate_dataset(output_dir: str | Path, num_images: int = 100, seed: int = 7) -> Path:
    output_dir = Path(output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(seed)
    camera = PinholeCamera()
    model = CubeSatModel()
    renderer = OpenGLCubeSatRenderer(camera=camera, model=model)
    points_3d = model.keypoints_3d()

    samples: List[Dict[str, Any]] = []
    for idx in range(num_images):
        pose = sample_pose(rng, camera)
        tvec = pose["translation_m"].reshape(3, 1)
        q = pose["quaternion_wxyz"]
        rvec = quaternion_to_rvec(q)
        keypoints_px = project_points(points_3d, rvec, tvec, camera.matrix)

        result = renderer.render(q, tvec, keypoints_px)
        filename = f"{idx:06d}.png"
        renderer.save_image(image_dir / filename, result.rgb)

        samples.append(
            {
                "image": f"images/{filename}",
                "translation_m": pose["translation_m"].round(8).tolist(),
                "quaternion_wxyz": pose["quaternion_wxyz"].round(10).tolist(),
                "keypoints_2d_px": keypoints_px.round(3).tolist(),
                "keypoints_3d_m": points_3d.round(6).tolist(),
            }
        )

    metadata = {
        "name": "opengl_speed_cube_mini",
        "description": "Minimal OpenGL synthetic CubeSat image dataset for 6-DOF pose estimation.",
        "num_images": num_images,
        "seed": seed,
        "camera": camera.as_dict(),
        "target_model": model.as_dict(),
        "samples": samples,
    }
    metadata_path = output_dir / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return metadata_path
