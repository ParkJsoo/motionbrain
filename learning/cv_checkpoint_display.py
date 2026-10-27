"""Saved evaluation overlays, without importing or running a model."""
import io
from pathlib import Path

from learning.cv_training import verified_bytes


def render_comparisons(frames, rows, output):
    from PIL import Image, ImageDraw

    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    inventory = []
    for index, (frame, row) in enumerate(zip(frames, rows), 1):
        canvas = Image.new("RGB", (960, 620), "#171f29")
        draw = ImageDraw.Draw(canvas)
        draw.text((12, 8), "GT: green | selected model boxes: orange | IMAGE COORDINATES ONLY", fill="white")
        draw.text((12, 28), "No physical success or generalization claim", fill="white")
        error = None
        try:
            with Image.open(io.BytesIO(verified_bytes(frame.path, frame.sha256))) as source:
                if source.size != (frame.width, frame.height):
                    raise ValueError("image size changed")
                image = source.convert("RGB")
            scale = min(960 / frame.width, 540 / frame.height)
            size = (round(frame.width * scale), round(frame.height * scale))
            if min(size) < 1:
                raise ValueError("image too thin for comparison")
            ox, oy = (960 - size[0]) // 2, 60 + (540 - size[1]) // 2
            canvas.paste(image.resize(size), (ox, oy))
            for boxes, color, prefix in ((frame.boxes, "#55ff80", "GT"),
                                          (row["selectedBoxes"], "#ffab44", "PRED")):
                centers = []
                for name, box in zip(("lip", "grip"), boxes):
                    if box is None:
                        centers.append(None)
                        continue
                    x0, y0, x1, y1 = box
                    coords = (ox+x0*scale, oy+y0*scale, ox+x1*scale, oy+y1*scale)
                    draw.rectangle(coords, outline=color, width=2)
                    draw.text((coords[0], max(60, coords[1]-12)), f"{prefix} {name}", fill=color)
                    centers.append((ox+(x0+x1)*scale/2, oy+(y0+y1)*scale/2))
                if all(c is not None for c in centers):
                    draw.line(centers, fill=color, width=2)
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            draw.text((12, 90), "IMAGE UNAVAILABLE", fill="white")
        message = "PROCESSING FAILED / NOT EVALUATED" if row["processingFailure"] else (
            f"pair={row['pairMatched']} position={row['positionSuccess']} direction={row['directionSuccess']}")
        draw.text((12, 604), message, fill="white")
        filename = f"{index:04d}.png"
        canvas.save(output / filename)
        inventory.append({"id": frame.id, "preview": filename, "renderError": error,
                          "processingFailure": row["processingFailure"]})
    return {"frames": inventory, "failures": sum(r["renderError"] is not None for r in inventory)}
