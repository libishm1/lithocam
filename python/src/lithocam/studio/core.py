# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Locate the compiled C++ core (_lithocam). Order: an installed or bundled module, the LITHOCAM_CORE directory,
then the repo's build_py/Release and build_py."""
from __future__ import annotations

import os
import sys
from pathlib import Path


def _load():
    try:
        import _lithocam
        return _lithocam
    except ImportError:
        pass
    repo = Path(__file__).resolve().parents[4]
    for d in [os.environ.get("LITHOCAM_CORE", ""), repo / "build_py" / "Release", repo / "build_py"]:
        if d and Path(d).is_dir() and str(d) not in sys.path:
            sys.path.insert(0, str(d))
            try:
                import _lithocam
                return _lithocam
            except ImportError:
                sys.path.remove(str(d))
    raise ImportError("_lithocam not found: build bindings/ (see bindings/README.md) or set LITHOCAM_CORE")


lc = _load()
