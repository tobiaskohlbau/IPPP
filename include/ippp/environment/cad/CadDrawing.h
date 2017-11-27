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

#ifndef CADDRAWING_H
#define CADDRAWING_H

#include <string>
#include <vector>

#include <ippp/environment/cad/CadProcessing.h>

namespace ippp {
namespace cad {

Eigen::MatrixXi create2dspace(const AABB &boundary, int fillValue);
void drawTriangles(Eigen::MatrixXi &space, const Mesh &mesh, int fillValue);
void drawTriangles(Eigen::MatrixXi &space, std::vector<Mesh> &meshes, int fillValue);
void drawTriangles(Eigen::MatrixXi &space, const std::vector<Triangle2D> &triangles, int fillValue);

void fillBottomFlatTriangle(Eigen::MatrixXi &space, Vector2 v1, Vector2 v2, Vector2 v3, int value);
void fillTopFlatTriangle(Eigen::MatrixXi &space, Vector2 v1, Vector2 v2, Vector2 v3, int value);
void drawLine(Eigen::MatrixXi &space, int x1, int x2, int y, int value);

} /* namespace cad */

} /* namespace ippp */

#endif    // CADDRAWING_H
