//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

#include <ippp/environment/cad/CadDrawing.h>

#include <ippp/util/Logging.h>

namespace ippp {
namespace cad {

std::pair<MatrixXi, Vector2i> create2dspace(const AABB &boundary, const int fillValue) {
    Vector3 dia = boundary.diagonal();
    Vector3 min = boundary.min();
    // pair with Eigen::Matrix as space and the offset of the AABB to the origin
    return std::make_pair(MatrixXi::Constant(static_cast<int>(dia[0]), static_cast<int>(dia[1]), fillValue),
                          Vector2i(-min[0], -min[1]));
}

void drawTriangles(std::pair<MatrixXi, Vector2i> &space, const Mesh &mesh, const int fillValue) {
    auto triangles = generateTriangles(mesh);
    drawTriangles(space, triangles, fillValue);
}

void drawTriangles(std::pair<MatrixXi, Vector2i> &space, std::vector<Mesh> &meshes, const int fillValue) {
    std::vector<Triangle2D> triangles;
    for (auto &mesh : meshes) {
        auto tris = generateTriangles(mesh);
        triangles.insert(std::end(triangles), std::begin(tris), std::end(tris));
    }
    drawTriangles(space, triangles, fillValue);
}

void drawTriangles(std::pair<MatrixXi, Vector2i> &space, const std::vector<Triangle2D> &triangles, const int fillValue) {
    for (auto &triangle : triangles) {
        std::vector<Vector2> points;
        for (size_t i = 1; i < 4; ++i)
            points.push_back(triangle.getP(static_cast<unsigned int>(i)));

        struct Vector2Sorter {
            bool operator()(const Vector2 &lhs, const Vector2 &rhs) const {
                return lhs[1] < rhs[1];
            }
        };
        std::sort(std::begin(points), std::end(points), Vector2Sorter());
        Vector2 v1 = points[0];
        Vector2 v2 = points[1];
        Vector2 v3 = points[2];

        if (v2[1] == v3[1])
            fillBottomFlatTriangle(space, v1, v2, v3, fillValue);
        /* check for trivial case of top-flat triangle */
        else if (v1[1] == v2[1]) {
            fillTopFlatTriangle(space, v1, v2, v3, fillValue);
        } else {
            /* general case - split the triangle in a topflat and bottom-flat one */
            Vector2 v4(v1[0] + ((double)(v2[1] - v1[1]) / (double)(v3[1] - v1[1])) * (v3[0] - v1[0]), v2[1]);
            fillBottomFlatTriangle(space, v1, v2, v4, fillValue);
            fillTopFlatTriangle(space, v2, v4, v3, fillValue);
        }
    }
}

void fillBottomFlatTriangle(std::pair<MatrixXi, Vector2i> &space, Vector2 v1, Vector2 v2, Vector2 v3, int value) {
    double invslope1 = (v2[0] - v1[0]) / (v2[1] - v1[1]);
    double invslope2 = (v3[0] - v1[0]) / (v3[1] - v1[1]);

    double curx1 = v1[0];
    double curx2 = v1[0];

    for (auto scanlineY = static_cast<int>(v1[1]); scanlineY <= v2[1]; scanlineY++) {
        drawLine(space, (int)curx1, (int)curx2, scanlineY, value);
        curx1 += invslope1;
        curx2 += invslope2;
    }
}

void fillTopFlatTriangle(std::pair<MatrixXi, Vector2i> &space, Vector2 v1, Vector2 v2, Vector2 v3, int value) {
    double invslope1 = (v3[0] - v1[0]) / (v3[1] - v1[1]);
    double invslope2 = (v3[0] - v2[0]) / (v3[1] - v2[1]);

    double curx1 = v3[0];
    double curx2 = v3[0];

    for (auto scanlineY = static_cast<int>(v3[1]); scanlineY > v1[1]; scanlineY--) {
        drawLine(space, (int)curx1, (int)curx2, scanlineY, value);
        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

void drawLine(std::pair<MatrixXi, Vector2i> &space, int x1, int x2, int y, int value) {
    if (x1 > x2)
        std::swap(x1, x2);

    for (int x = x1; x <= x2; ++x) {
        if (0 > y || y > space.first.rows() || 0 > x || x > space.first.cols()) {
            Logging::debug("Triangle out of space", "CadProcessing");
            continue;
        }
        space.first(y, x) = value;
    }
}

} /* namespace cad */

} /* namespace ippp */
