"""Small helpers for reading either lossless or JPEG-backed episode RGB frames."""
from __future__ import annotations

import cv2
import h5py
import numpy as np


def read_rgb(dataset: h5py.Dataset, index: int) -> np.ndarray:
    """Return an HxWx3 RGB uint8 frame independent of its HDF5 storage mode."""
    if dataset.attrs.get("storage", "raw_lzf") == "jpeg":
        bgr = cv2.imdecode(np.asarray(dataset[index], dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f"failed to decode JPEG frame {index} from {dataset.name}")
        return np.ascontiguousarray(bgr[..., ::-1])
    return np.asarray(dataset[index], dtype=np.uint8)
