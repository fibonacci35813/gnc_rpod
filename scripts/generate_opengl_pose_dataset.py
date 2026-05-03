#!/usr/bin/env python3
"""Generate a minimal OpenGL CubeSat pose dataset."""

from __future__ import annotations

import argparse

from rpod_gnc.vision.dataset_generator import generate_dataset


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="datasets/opengl_speed_cube_mini", help="Output dataset directory")
    parser.add_argument("--num-images", type=int, default=100, help="Number of images to render")
    parser.add_argument("--seed", type=int, default=7, help="Random seed")
    args = parser.parse_args()

    metadata_path = generate_dataset(args.output, num_images=args.num_images, seed=args.seed)
    print(f"Wrote dataset metadata: {metadata_path}")


if __name__ == "__main__":
    main()
