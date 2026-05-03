"""OpenGL/Pyrender synthetic renderer for a minimal SPEED-style CubeSat dataset."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from .camera import PinholeCamera
from .geometry import CubeSatModel


@dataclass
class RenderResult:
    rgb: np.ndarray
    visible_keypoints_px: np.ndarray


class OpenGLCubeSatRenderer:
    """Small offscreen OpenGL renderer.

    This is intentionally lightweight. It gives us a SPEED-UE-Cube-like data
    contract now: image + camera intrinsics + exact target pose label. Later, the
    same pose generator and metadata can drive Unreal Engine rendering.
    """

    def __init__(self, camera: PinholeCamera, model: CubeSatModel | None = None) -> None:
        self.camera = camera
        self.model = model or CubeSatModel()

    def _make_scene(self, q_wxyz: np.ndarray, tvec: np.ndarray):
        import pyrender
        import trimesh

        scene = pyrender.Scene(bg_color=[0, 0, 0, 255], ambient_light=[0.015, 0.015, 0.015])

        body = trimesh.creation.box(extents=[self.model.body_x, self.model.body_y, self.model.body_z])
        panel = trimesh.creation.box(extents=[self.model.panel_span, 0.012, self.model.panel_height])
        panel.apply_translation([0.0, -self.model.body_y / 2 - 0.011, 0.0])

        # Antennae as thin cylinders for asymmetry.
        antenna_meshes = []
        for x in [-self.model.body_x / 2, self.model.body_x / 2]:
            ant = trimesh.creation.cylinder(radius=0.002, height=0.18, sections=8)
            ant.apply_translation([x, 0.0, -self.model.body_z / 2 - 0.09])
            antenna_meshes.append(ant)
        top_ant = trimesh.creation.cylinder(radius=0.002, height=0.18, sections=8)
        top_ant.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
        top_ant.apply_translation([0.0, self.model.body_y / 2 + 0.09, self.model.body_z / 2])
        antenna_meshes.append(top_ant)

        target_mesh = trimesh.util.concatenate([body, panel, *antenna_meshes])
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=[0.78, 0.78, 0.72, 1.0], metallicFactor=0.0, roughnessFactor=0.5
        )
        mesh = pyrender.Mesh.from_trimesh(target_mesh, material=material, smooth=False)

        q = np.asarray(q_wxyz, dtype=np.float64)
        rot = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
        scene.add(mesh, pose=pose)

        camera = pyrender.IntrinsicsCamera(
            fx=self.camera.fx,
            fy=self.camera.fy,
            cx=self.camera.cx,
            cy=self.camera.cy,
            znear=0.05,
            zfar=100.0,
        )
        scene.add(camera, pose=np.eye(4))

        light_pose = np.eye(4)
        light_pose[:3, 3] = np.array([-2.0, -3.0, 4.0])
        scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=3.0), pose=light_pose)
        return scene

    def render(self, q_wxyz: np.ndarray, tvec: np.ndarray, keypoints_px: np.ndarray) -> RenderResult:
        import pyrender

        scene = self._make_scene(q_wxyz, tvec)
        renderer = pyrender.OffscreenRenderer(viewport_width=self.camera.width, viewport_height=self.camera.height)
        color, _ = renderer.render(scene)
        renderer.delete()

        rgb = color[:, :, :3].copy()
        rgb = self._add_background_and_noise(rgb)
        rgb = self._draw_keypoint_overlay(rgb, keypoints_px)
        return RenderResult(rgb=rgb, visible_keypoints_px=keypoints_px)

    def _add_background_and_noise(self, rgb: np.ndarray) -> np.ndarray:
        rng = np.random.default_rng()
        image = rgb.astype(np.float32)
        star_mask = rng.random((self.camera.height, self.camera.width)) > 0.9994
        image[star_mask] = rng.uniform(180, 255, size=(star_mask.sum(), 1))
        noise = rng.normal(0, 2.5, size=image.shape)
        image = np.clip(image + noise, 0, 255)
        return image.astype(np.uint8)

    def _draw_keypoint_overlay(self, rgb: np.ndarray, keypoints_px: np.ndarray) -> np.ndarray:
        # Tiny green points make debugging easy. Training images can disable this later.
        image = Image.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        for x, y in keypoints_px:
            if 0 <= x < self.camera.width and 0 <= y < self.camera.height:
                draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(0, 255, 0))
        return np.asarray(image)

    @staticmethod
    def save_image(path: str | Path, rgb: np.ndarray) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb).save(path)
