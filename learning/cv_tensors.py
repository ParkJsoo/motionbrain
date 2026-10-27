"""Optional CPU tensor adapter for approved CV manifests; no model execution."""
from __future__ import annotations

import hashlib
import io

from learning.cv_dataset import CVDataset, require


class TorchCVDataset(CVDataset):
    """Reuse manifest validation; decode with Pillow, preserve source coordinates.

    Torch, NumPy and Pillow are imported only when an item is requested. Images
    are RGB float32 CHW on CPU, with no resize, augmentation or EXIF transpose.
    Only 8-bit RGB, RGBA and grayscale images are accepted. Alpha is discarded.
    """

    def __getitem__(self, index):
        import numpy as np
        import torch
        from PIL import Image

        frame = self.frames[index]
        target = frame.target()
        require(0 < frame.image_id <= torch.iinfo(torch.int64).max, "imageId exceeds int64")
        raw = frame.path.read_bytes()
        require(hashlib.sha256(raw).hexdigest() == frame.sha256,
                f"image hash mismatch: {frame.id}")
        with Image.open(io.BytesIO(raw)) as source:
            require(source.size == (frame.width, frame.height), f"image size mismatch: {frame.id}")
            require(source.mode in ("RGB", "RGBA", "L"), f"unsupported image mode: {source.mode}")
            require(getattr(source, "n_frames", 1) == 1, "animated images are unsupported")
            pixels = np.array(source.convert("RGB"), dtype=np.uint8, copy=True)
        image = torch.from_numpy(pixels).permute(2, 0, 1).contiguous().to(torch.float32).div_(255)
        boxes = torch.tensor(target["boxes"], dtype=torch.float32, device="cpu").reshape(-1, 4)
        require(bool(torch.isfinite(boxes).all()), "bbox cannot be represented as finite float32")
        require(bool((boxes[:, 2:] > boxes[:, :2]).all()), "bbox collapses at float32 precision")
        area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        require(bool(torch.isfinite(area).all()) and bool((area > 0).all()),
                "bbox area cannot be represented as positive finite float32")
        return image, {
            "boxes": boxes,
            "labels": torch.tensor(target["labels"], dtype=torch.int64, device="cpu"),
            "image_id": torch.tensor([frame.image_id], dtype=torch.int64, device="cpu"),
            "area": area,
            "iscrowd": torch.tensor(target["iscrowd"], dtype=torch.int64, device="cpu"),
        }


def detection_collate(batch):
    """Keep variable image sizes and per-image box counts as lists, not stacks."""
    require(bool(batch), "cannot collate an empty batch")
    images, targets = zip(*batch)
    return list(images), list(targets)
