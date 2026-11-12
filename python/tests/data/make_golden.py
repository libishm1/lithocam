# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Record quiddity's recognition document for every fixture STEP file, so the mapping tests run without quiddity.

    python -B tests/data/make_golden.py            (needs quiddity: pip install quiddity, Python 3.11+)

step/*.step        (build123d)  -> quiddity/<name>.json
step_rhino/*.step  (Rhino 8)    -> quiddity/rhino_<name>.json, read after lithocam.step_canon, as recognise() does
Only the versioned `document` is kept. Re-run after a quiddity upgrade and review the diff.
"""
import json
import os
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src"))
from lithocam.features import run_quiddity  # noqa: E402
from lithocam.step_canon import canonical_lines  # noqa: E402

out = HERE / "quiddity"
out.mkdir(exist_ok=True)


def record(step, name, canon):
    path, tmp, n = str(step), None, 0
    if canon:
        text, n = canonical_lines(step.read_text(encoding="utf-8", errors="replace"))
        fd, tmp = tempfile.mkstemp(suffix=".step")
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        path = tmp
    try:
        doc = run_quiddity(path)["document"]
    finally:
        if tmp:
            os.remove(tmp)
    (out / (name + ".json")).write_text(json.dumps({"document": doc}, indent=1) + "\n", encoding="utf-8")
    print(name, doc["package"]["version"], len(doc["records"]), "records,", n, "edges canonicalised")


for step in sorted((HERE / "step").glob("*.step")):
    record(step, step.stem, canon=False)
for step in sorted((HERE / "step_rhino").glob("*.step")):
    record(step, "rhino_" + step.stem, canon=True)
