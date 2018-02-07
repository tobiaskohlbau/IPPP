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

#ifndef CADPROCESSING_H
#define CADPROCESSING_H

#include <string>
#include <vector>

#include <ippp/dataObj/PointList.hpp>
#include <ippp/types.h>
#include <ippp/util/UtilList.hpp>
#include <ippp/environment/model/Mesh.h>

namespace ippp {
namespace cad {

std::vector<Triangle2D> generateTriangles(const Mesh &mesh);
Mesh generateMesh(const std::vector<Triangle2D> &triangles);
Mesh mergeMeshes(const std::vector<Mesh> &meshes);

Vector3 calcCenterOfMesh(const Mesh &mesh);
void centerMeshes(std::vector<Mesh> &meshes);
void centerMesh(Mesh &mesh);

void transformVertices(const Vector6 &config, std::vector<Vector3> &vertices);
void transformVertices(const Transform &T, std::vector<Vector3> &vertices);
std::vector<Vector3> computeNormals(const std::vector<Vector3> &vertices, const std::vector<Vector3i> &faces);

AABB computeAABB(const std::vector<Vector3> &vertices);
AABB computeAABB(const Mesh &mesh);
AABB computeAABB(const std::vector<Triangle2D> &triangles);

} /* namespace cad */

} /* namespace ippp */

#endif    // CADPROCESSING_H
