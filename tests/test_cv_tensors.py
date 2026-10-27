import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from learning.cv_dataset import CLASSES, COORDINATES
from learning.cv_tensors import TorchCVDataset, detection_collate


class OptionalTensorDependencyTest(unittest.TestCase):
    def test_module_and_collate_do_not_import_torch(self):
        result = subprocess.run([sys.executable, "-c", """
import sys
sys.modules['torch'] = None
sys.modules['PIL'] = None
from learning.cv_tensors import TorchCVDataset, detection_collate
assert detection_collate([(1, 2)]) == ([1], [2])
"""], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


@unittest.skipUnless(importlib.util.find_spec("torch") is not None, "optional torch runtime not installed")
class CVTensorTest(unittest.TestCase):
    def setUp(self):
        import torch
        self.torch = torch
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.path = self.root / "manifest.json"
        self.doc = {"schema": "motionbrain.cv_dataset.v1",
                    "classIds": {"background_reserved_not_annotation": 0, "lip_balm": 1, "gripper": 2},
                    "records": [], "splits": {"train": []}}
        self.add_frame("pair", [[1, 1, 4, 3], [4, 0, 7, 4]])

    def add_frame(self, name, boxes, size=(8, 4), mode="RGB", exif=None):
        from PIL import Image
        image = Image.new(mode, size, (255, 64 + len(self.doc["records"]), 0) if mode == "RGB" else 120)
        if mode == "RGB":
            image.putpixel((0, 0), (0, 128, 255))
        path = self.root / (name + ".png")
        image.save(path, **({"exif": exif} if exif else {}))
        labels = [{"class": cls, "presence": "present" if b else "absent", "bbox_xyxy": b,
                   "bbox_valid": b is not None, "occlusion": "none" if b else "unknown",
                   "truncated": False if b else "unknown", "humanApproved": True,
                   "reviewStatus": "human_approved"} for cls, b in zip(CLASSES, boxes)]
        self.doc["records"].append({"id": name, "imageId": len(self.doc["records"]) + 1,
                                    "image": path.name, "sourceSha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                    "width": size[0], "height": size[1], "sessionId": "s", "leakageGroup": "g",
                                    "split": "train", "coordinates": COORDINATES, "labels": labels})
        self.doc["splits"]["train"].append(name)

    def dataset(self, **kwargs):
        self.path.write_text(json.dumps(self.doc))
        return TorchCVDataset(self.path, **kwargs)

    def test_rgb_layout_dtype_scale_and_original_coordinates(self):
        t = self.torch
        image, target = self.dataset()[0]
        self.assertEqual(tuple(image.shape), (3, 4, 8))
        self.assertEqual(image.dtype, t.float32)
        self.assertEqual(image.device.type, "cpu")
        self.assertTrue(image.is_contiguous())
        t.testing.assert_close(image[:, 0, 0], t.tensor([0, 128 / 255, 1]))
        t.testing.assert_close(image[:, 2, 2], t.tensor([1, 64 / 255, 0]))
        self.assertEqual(target["boxes"].tolist(), [[1, 1, 4, 3], [4, 0, 7, 4]])
        self.assertEqual(target["labels"].tolist(), [1, 2])
        self.assertEqual(target["area"].tolist(), [6, 12])
        self.assertEqual(target["image_id"].tolist(), [1])
        for key in ("labels", "image_id", "iscrowd"):
            self.assertEqual(target[key].dtype, t.int64)
        for key in ("boxes", "area"):
            self.assertEqual(target[key].dtype, t.float32)
        self.assertEqual(target["iscrowd"].tolist(), [0, 0])

    def test_absent_and_empty_targets_keep_exact_tensor_shapes(self):
        self.add_frame("absent", [None, [1, 0, 7, 4]])
        self.add_frame("empty", [None, None])
        ds = self.dataset()
        self.assertEqual(ds[1][1]["labels"].tolist(), [2])
        self.assertEqual(tuple(ds[1][1]["boxes"].shape), (1, 4))
        target = ds[2][1]
        self.assertEqual(tuple(target["boxes"].shape), (0, 4))
        for key in ("labels", "area", "iscrowd"):
            self.assertEqual(tuple(target[key].shape), (0,))

    def test_cpu_output_does_not_inherit_callers_default_device(self):
        self.add_frame("empty", [None, None])
        ds = self.dataset()
        with self.torch.device("meta"):
            for i in range(len(ds)):
                image, target = ds[i]
                self.assertEqual(image.device.type, "cpu")
                self.assertTrue(all(v.device.type == "cpu" for v in target.values()))
                self.assertEqual(target["image_id"].tolist(), [i + 1])

    def test_dataloader_keeps_variable_sizes_and_box_counts(self):
        from torch.utils.data import DataLoader
        self.add_frame("other", [None, None], size=(5, 7))
        images, targets = next(iter(DataLoader(self.dataset(), batch_size=2, collate_fn=detection_collate)))
        self.assertIsInstance(images, list)
        self.assertEqual([tuple(i.shape) for i in images], [(3, 4, 8), (3, 7, 5)])
        self.assertEqual([len(t["boxes"]) for t in targets], [2, 0])
        with self.assertRaisesRegex(ValueError, "empty batch"):
            detection_collate([])

    def test_returned_tensors_do_not_mutate_later_reads(self):
        ds = self.dataset()
        image, target = ds[0]
        image.zero_(); target["boxes"].zero_(); target["labels"].zero_()
        image, target = ds[0]
        self.assertEqual(image[2, 0, 0].item(), 1)
        self.assertEqual(target["boxes"][0].tolist(), [1, 1, 4, 3])
        self.assertEqual(target["labels"].tolist(), [1, 2])

    def test_image_bytes_are_rehashed_on_every_access(self):
        ds = self.dataset(); ds[0]
        (self.root / "pair.png").write_bytes(b"tampered")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            ds[0]

    def test_decode_size_and_mode_failures_are_not_background(self):
        original = copy.deepcopy(self.doc)
        raw = (self.root / "pair.png").read_bytes()
        for failure in ("size", "decode", "mode"):
            with self.subTest(failure=failure):
                self.doc = copy.deepcopy(original)
                path = self.root / "pair.png"; path.write_bytes(raw)
                if failure == "size":
                    self.doc["records"][0]["width"] = 9
                elif failure == "decode":
                    path.write_bytes(b"not a png")
                else:
                    from PIL import Image
                    Image.new("I;16", (8, 4)).save(path)
                self.doc["records"][0]["sourceSha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
                with self.assertRaises((ValueError, OSError)):
                    self.dataset()[0]

    def test_exif_orientation_is_not_applied(self):
        from PIL import Image
        exif = Image.Exif(); exif[274] = 6
        self.add_frame("oriented", [None, None], exif=exif)
        image, _ = self.dataset()[1]
        self.assertEqual(tuple(image.shape), (3, 4, 8))
        self.torch.testing.assert_close(image[:, 0, 0], self.torch.tensor([0, 128 / 255, 1]))

    def test_animated_image_is_rejected(self):
        from PIL import Image
        path = self.root / "pair.png"
        Image.new("RGB", (8, 4), (1, 2, 3)).save(
            path, save_all=True, append_images=[Image.new("RGB", (8, 4), (4, 5, 6))])
        self.doc["records"][0]["sourceSha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, "animated"):
            self.dataset()[0]

    def test_grayscale_and_rgba_conversion(self):
        from PIL import Image
        self.add_frame("gray", [None, None], mode="L")
        self.add_frame("alpha", [None, None])
        path = self.root / "alpha.png"
        Image.new("RGBA", (8, 4), (255, 64, 0, 0)).save(path)
        self.doc["records"][-1]["sourceSha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        ds = self.dataset()
        self.torch.testing.assert_close(ds[1][0][:, 0, 0], self.torch.full((3,), 120 / 255))
        self.torch.testing.assert_close(ds[2][0][:, 0, 0], self.torch.tensor([1, 64 / 255, 0]))

    def test_float32_box_collapse_and_int64_id_overflow_fail(self):
        original = copy.deepcopy(self.doc)
        self.doc["records"][0]["labels"][0]["bbox_xyxy"] = [1, 1, 1 + 1e-9, 2]
        with self.assertRaisesRegex(ValueError, "float32 precision"):
            self.dataset()[0]
        self.doc = original
        self.doc["records"][0]["imageId"] = 2 ** 63
        with self.assertRaisesRegex(ValueError, "int64"):
            self.dataset()[0]

    def test_approval_and_split_leakage_checks_are_inherited(self):
        original = copy.deepcopy(self.doc)
        self.doc["records"][0]["labels"][1]["humanApproved"] = False
        with self.assertRaisesRegex(ValueError, "approval"):
            self.dataset()
        self.doc = original
        self.add_frame("test", [None, None])
        self.doc["records"][1]["split"] = "test"
        self.doc["splits"] = {"train": ["pair"], "test": ["test"]}
        with self.assertRaisesRegex(ValueError, "crosses splits"):
            self.dataset(split="train")


if __name__ == "__main__":
    unittest.main()
