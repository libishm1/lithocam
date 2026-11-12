// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// dropcutter_cli <triangles.txt> <points.txt> <ball|flat> <radius>  ->  one tip height per point on stdout
//   triangles.txt: 9 numbers per line (ax ay az bx by bz cx cy cz)
//   points.txt:    2 numbers per line (x y)
// Used for differential tests against other drop-cutter implementations (tests/oracle/).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "lithocam/dropcutter.hpp"

int main(int argc, char** argv) {
    if (argc != 5) {
        std::fprintf(stderr, "usage: dropcutter_cli triangles.txt points.txt ball|flat radius\n");
        return 2;
    }
    std::vector<lithocam::Tri> tris;
    {
        std::ifstream f(argv[1]);
        lithocam::Tri t{};
        while (f >> t.a.x >> t.a.y >> t.a.z >> t.b.x >> t.b.y >> t.b.z >> t.c.x >> t.c.y >> t.c.z) tris.push_back(t);
    }
    lithocam::Cutter c{std::strcmp(argv[3], "ball") == 0 ? lithocam::CutterKind::Ball : lithocam::CutterKind::Flat,
                       std::stod(argv[4])};
    lithocam::TriMesh mesh(tris);
    std::ifstream q(argv[2]);
    double x, y;
    while (q >> x >> y) {
        double z = lithocam::drop_cutter(c, mesh, x, y);
        if (std::isinf(z)) std::printf("nan\n");
        else std::printf("%.12f\n", z);
    }
    return 0;
}
