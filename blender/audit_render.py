"""Validate a complete canonical PNG sequence before telemetry encoding."""
import csv
import hashlib
import json
from pathlib import Path

from PIL import Image

root = Path(__file__).resolve().parents[1]
out = root / "out/revised"
rows = list(csv.DictReader((out / "trajectory.csv").open(newline="")))
expected = {f"frame_{i:04d}.png" for i in range(1,len(rows)+1)}
found = {p.name for p in (out / "frames").glob("*.png")}
errors, dimensions = [], set()
for name in sorted(expected & found):
    try:
        with Image.open(out / "frames" / name) as frame:
            dimensions.add(frame.size)
            frame.load()
    except Exception as exc:
        errors.append({"file": name, "error": str(exc)})
report = {"expected_frame_count": len(rows), "actual_frame_count": len(found),
    "missing_frames": sorted(expected-found), "unexpected_frames": sorted(found-expected),
    "decode_errors": errors, "dimensions": sorted(dimensions),
    "fps": json.loads((out / "scene.json").read_text())["fps"],
    "source_sha256": {name: hashlib.sha256((out/name).read_bytes()).hexdigest()
                      for name in ("trajectory.csv", "scene.json", "mission_s.blend")}}
report["pass"] = not (report["missing_frames"] or report["unexpected_frames"] or errors) and dimensions == {(1344,756)}
(root / "out/revised/render_delivery_audit.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
raise SystemExit(0 if report["pass"] else 1)
