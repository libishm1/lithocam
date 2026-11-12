# Differential tests against OpenCAMLib (test-only)

lithocam's milling kernel is written from first principles and the papers. OpenCAMLib (LGPL-2.1) is used here **only as a test oracle**. It is never imported or linked by lithocam, and none of its code is copied.

Steps:

1. `python make_case.py out/` writes `tris.txt` and `points.txt`: a terrain mesh plus random triangles, and query points.
2. `dropcutter_cli out/tris.txt out/points.txt ball 3 > out/lithocam.txt`. `dropcutter_cli` is built by CMake.
3. In a separate environment with `opencamlib` (wheels up to CPython 3.11; on Windows use WSL with Python 3.10):

   ```sh
   python ocl_dropcutter.py out/tris.txt out/points.txt ball 3 > out/ocl.txt
   ```

4. `python compare.py out/lithocam.txt out/ocl.txt` prints the differences.
