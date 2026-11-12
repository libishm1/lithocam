# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Build the lithocam studio executable with PyInstaller (one folder, windowed).

    python tools/build_studio_exe.py [dist dir]

Needs, in the running Python: PySide6, numpy, pyinstaller, and the compiled core (_lithocam) from bindings/
(found through LITHOCAM_CORE or build_py/Release). Bundles cells/ (URDFs, visual and collision meshes).
Licences of what gets bundled: PySide6/Qt LGPL-3.0 (dynamically linked, unmodified), numpy BSD-3, robot meshes
BSD-3 (UR) and Apache-2.0 (KUKA, ROS-Industrial); see cells/README.md.
"""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))


def main():
    dist = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "dist"
    work = dist.parent / "build_studio"
    from lithocam.studio.core import lc
    core = Path(lc.__file__)
    entry = work / "lithocam_studio.py"
    work.mkdir(parents=True, exist_ok=True)
    entry.write_text("import sys\nfrom lithocam.studio.app import main\nsys.exit(main())\n")
    sep = os.pathsep
    args = [str(entry), "--name", "lithocam-studio", "--windowed", "--noconfirm", "--onedir",
            "--distpath", str(dist), "--workpath", str(work), "--specpath", str(work),
            "--paths", str(ROOT / "python" / "src"),
            "--add-binary", "{}{}.".format(core, sep),
            "--add-data", "{}{}cells".format(ROOT / "cells", sep),
            "--hidden-import", "_lithocam",
            "--collect-submodules", "lithocam.studio",
            "--exclude-module", "tesseract_robotics", "--exclude-module", "OCP", "--exclude-module", "matplotlib"]
    import PyInstaller.__main__
    PyInstaller.__main__.run(args)
    print("built", dist / "lithocam-studio")


if __name__ == "__main__":
    main()
