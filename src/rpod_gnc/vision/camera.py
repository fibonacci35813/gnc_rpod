"""Camera geometry utilities for synthetic RPOD vision datasets."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict

import numpy as np


@dataclass(frozen=True)
class PinholeCamera:
    """Simple pinhole camera model.

    The default values roughly follow the SPEED camera resolution and horizontal
    field-of-view, while keeping the implementation renderer-agnostic.
    """

    width: int = 960
    height: int = 600
    horizontal_fov_deg: float = 35.6

    @property
    def fx(self) -> float:
        fov_rad = np.deg2rad(self.horizontal_fov_deg)
        return float((self.width / 2.0) / np.tan(fov_rad / 2.0))

    @property
    def fy(self) -> float:
        return self.fx

    @property
    def cx(self) -> float:
        return self.width / 2.0

    @property
    def cy(self) -> float:
        return self.height / 2.0

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def as_dict(self) -> Dict[str, float]:
        data = asdict(self)
        data.update({"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy})
        return data
