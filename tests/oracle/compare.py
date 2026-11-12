# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Compare two height lists (one value or nan per line). Exit 1 when any pair differs by more than tol (default 1e-6)."""
import math
import sys

a = [float(x) for x in open(sys.argv[1]).read().split()]
b = [float(x) for x in open(sys.argv[2]).read().split()]
tol = float(sys.argv[3]) if len(sys.argv) > 3 else 1e-6
assert len(a) == len(b), (len(a), len(b))
both_nan = sum(1 for x, y in zip(a, b) if math.isnan(x) and math.isnan(y))
one_nan = [i for i, (x, y) in enumerate(zip(a, b)) if math.isnan(x) != math.isnan(y)]
diffs = [(abs(x - y), i) for i, (x, y) in enumerate(zip(a, b)) if not (math.isnan(x) or math.isnan(y))]
worst = max(diffs) if diffs else (0.0, -1)
bad = [d for d in diffs if d[0] > tol]
print("points {}  both-nan {}  nan-mismatch {}  max |diff| {:.3e} at {}  over tol {}".format(
    len(a), both_nan, len(one_nan), worst[0], worst[1], len(bad)))
for d, i in sorted(bad, reverse=True)[:5]:
    print("  point {}: lithocam {:.9f}  other {:.9f}".format(i, a[i], b[i]))
sys.exit(1 if bad or one_nan else 0)
