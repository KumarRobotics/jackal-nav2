"""Ultralytics SAM3 backend for the semantic terrain processor.

Heavy inference dependencies are imported only when this backend is instantiated so
the processing core and its unit tests do not require SAM3, Torch, or model weights.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from jackal_nav2.semantic_terrain import InstanceMask

import numpy as np


class UltralyticsSam3Backend:
    """Text-prompted SAM3 instance segmentation using Ultralytics."""

    def __init__(
        self,
        *,
        model_path: str,
        device: str = "cuda",
        image_size: int = 640,
        model_confidence: float = 0.25,
        half_precision: bool = True,
    ):
        checkpoint = Path(model_path).expanduser()
        if not checkpoint.is_file():
            raise ValueError(f"SAM3 model_path is not a file: {checkpoint}")
        if image_size <= 0:
            raise ValueError("SAM3 image_size must be positive")
        if not 0.0 <= model_confidence <= 1.0:
            raise ValueError("SAM3 model_confidence must be in [0.0, 1.0]")

        # This is intentionally a private import boundary. Importing the module itself
        # stays cheap and allows fake backends to exercise the complete processing path.
        try:
            from ultralytics.models.sam import SAM3SemanticPredictor
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics with SAM3 support is required for the production backend"
            ) from exc

        device_name = str(device).strip() or "cpu"
        overrides = {
            "conf": float(model_confidence),
            "task": "segment",
            "mode": "predict",
            "model": str(checkpoint),
            "device": device_name,
            "imgsz": int(image_size),
            "half": bool(half_precision and device_name.startswith("cuda")),
            "retina_masks": True,
            "save": False,
            "verbose": False,
        }
        self._predictor = SAM3SemanticPredictor(overrides=overrides)

    def infer(
        self, bgr_image: np.ndarray, prompts: Sequence[str]
    ) -> list[InstanceMask]:
        """Return original-resolution masks and prompt-relative class indices."""
        image = np.asarray(bgr_image)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("SAM3 input must be a uint8 BGR image")
        if not prompts:
            raise ValueError("SAM3 requires at least one text prompt")

        self._predictor.set_image(image)
        results = self._predictor(text=list(prompts))
        instances: list[InstanceMask] = []
        for result in results:
            if result.boxes is None or result.masks is None or len(result.boxes) == 0:
                continue
            class_indices = result.boxes.cls.detach().cpu().numpy().astype(np.int64)
            scores = result.boxes.conf.detach().cpu().numpy().astype(np.float32)
            masks = result.masks.data.detach().cpu().numpy() > 0.5
            if len(class_indices) != len(masks):
                raise RuntimeError(
                    "SAM3 returned different numbers of boxes and segmentation masks"
                )

            for class_index, score, mask in zip(class_indices, scores, masks):
                mask_array = np.asarray(mask, dtype=bool)
                if mask_array.shape != image.shape[:2]:
                    # Ultralytics normally emits retina masks at source resolution. Keep
                    # a defensive nearest-neighbor path for API/version differences.
                    import cv2

                    mask_array = cv2.resize(
                        mask_array.astype(np.uint8),
                        (image.shape[1], image.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    ).astype(bool)
                instances.append(
                    InstanceMask(
                        class_index=int(class_index),
                        score=float(score),
                        mask=mask_array,
                    )
                )
        return instances

    def close(self) -> None:
        """Release predictor references and return unused CUDA cache to Torch."""
        self._predictor = None
        try:
            import torch

            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except ImportError:
            pass
