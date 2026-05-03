#!/usr/bin/env python3
"""Evaluate a generated OpenGL CubeSat pose dataset with EPnP."""

from __future__ import annotations

import argparse
import json

from rpod_gnc.vision.evaluate_dataset import evaluate_metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metadata", help="Path to metadata.json")
    parser.add_argument("--pixel-noise-std", type=float, default=0.0, help="Optional synthetic keypoint noise in pixels")
    args = parser.parse_args()

    report = evaluate_metadata(args.metadata, pixel_noise_std=args.pixel_noise_std)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
