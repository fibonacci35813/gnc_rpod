# OpenGL Vision Pose Dataset MVP

This branch adds a minimal SPEED-UE-Cube-style pipeline for vision-based 6-DOF pose estimation in RPOD.

The goal is not to replicate the full Unreal Engine dataset immediately. The goal is to create the same core data contract first:

```text
relative target pose -> rendered camera image -> pose label -> PnP estimate -> pose error report
```

## What is included

- A pinhole camera model with SPEED-like horizontal field-of-view.
- A simplified 3U CubeSat geometry model with body, solar panel, and antenna keypoints.
- An OpenGL/Pyrender offscreen renderer for synthetic target images.
- Dataset generation with image files and `metadata.json` labels.
- A PnP baseline using EPnP plus iterative refinement.
- Evaluation metrics:
  - translation error in meters
  - rotation error in degrees
  - reprojection error in pixels

## Install

```bash
pip install -r requirements-vision.txt
```

On some Linux systems, offscreen OpenGL may need EGL or OSMesa configuration. For headless runs, try:

```bash
export PYOPENGL_PLATFORM=egl
```

If EGL is unavailable, use:

```bash
export PYOPENGL_PLATFORM=osmesa
```

## Generate a small dataset

```bash
python scripts/generate_opengl_pose_dataset.py \
  --output datasets/opengl_speed_cube_mini \
  --num-images 100 \
  --seed 7
```

This writes:

```text
datasets/opengl_speed_cube_mini/
  images/
    000000.png
    000001.png
    ...
  metadata.json
```

Each sample in `metadata.json` contains:

- image path
- relative translation `[x, y, z]` in meters
- target quaternion `[qw, qx, qy, qz]`
- 2D keypoints in pixels
- 3D keypoints in target body frame
- camera intrinsics

## Evaluate with PnP

```bash
python scripts/evaluate_opengl_pose_dataset.py \
  datasets/opengl_speed_cube_mini/metadata.json
```

To simulate imperfect keypoint detection:

```bash
python scripts/evaluate_opengl_pose_dataset.py \
  datasets/opengl_speed_cube_mini/metadata.json \
  --pixel-noise-std 2.0
```

## Why this matters

This gives the repo the first open-loop perception validation primitive:

```text
synthetic image + ground-truth pose + estimated pose + error metric
```

Later this can be extended into:

1. trajectory image sequences,
2. ArUco or learned keypoint detection,
3. navigation filtering,
4. closed-loop guidance and control,
5. Unreal Engine rendering with Earth/Sun/space lighting realism.

## Next steps

- Add trajectory generation from the existing RPOD relative dynamics.
- Replace perfect projected keypoints with detected visual keypoints.
- Add plots for error vs distance and error vs time.
- Add an Unreal renderer behind the same metadata interface.
