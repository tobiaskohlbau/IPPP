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

#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <ippp/core/dataObj/PointList.hpp>
#include <ippp/core/types.h>
#include <ippp/core/util/UtilList.hpp>
#include <ippp/environment/model/Mesh.h>

namespace ippp {
namespace cad {

enum class ExportFormat {
    COLLADA,
    X_FILES,
    STEP,
    OBJ,
    STEREOLITHOGRAPHY,
    STEREOLITHOGRAPHY_BINARY,
    STANFORD_POLYGON_LIBRARY,
    STANFORD_POLYGON_LIBRARY_BINARY,
    AUTODESK_3DS,
    GL_TRANSMISSION_FORMAT,
    GL_TRANSMISSION_FORMAT_BINARY,
    ASSIMP_BINARY,
    ASSXML_DOCUMENT,
    EXTENSIBLE_3D
};

bool importMesh(const std::string &filePath, Mesh &mesh, const double scale = 1, const bool calcNormals = false);
bool importMeshes(const std::string &filePath, std::vector<Mesh> &meshes, const double scale = 1, const bool calcNormals = false);
void getMeshes(const aiScene *scene, const aiNode *node, aiMatrix4x4 *trafo, std::vector<Mesh> &meshes);
bool importBYU(const std::string &filePath, Mesh &mesh);
bool exportCad(ExportFormat format, const std::string &filePath, const Mesh &mesh);
bool exportCad(ExportFormat format, const std::string &filePath, const std::vector<Vector3> &vertices,
               const std::vector<Vector3i> &faces);

std::vector<Triangle2D> generateTriangles(const Mesh &mesh);
Mesh generateMesh(const std::vector<Triangle2D> &triangles);
Mesh mergeMeshes(const std::vector<Mesh> &meshes);

Eigen::MatrixXi create2dspace(const AABB &boundary, const int fillValue);
void drawTriangles(Eigen::MatrixXi &space, const std::vector<Triangle2D> &triangles, const int fillValue);

void transformVertices(const Vector6 &config, std::vector<Vector3> &vertices);
void transformVertices(const Matrix4 &T, std::vector<Vector3> &vertices);
std::vector<Vector3> computeNormals(const std::vector<Vector3> &vertices, const std::vector<Vector3i> &faces);

AABB computeAABB(const std::vector<Vector3> &vertices);
AABB computeAABB(const Mesh &mesh);
AABB computeAABB(const std::vector<Triangle2D> &triangles);

aiScene generateScene(const std::vector<Vector3> &vertices, const std::vector<Vector3i> &faces);

void fillBottomFlatTriangle(Eigen::MatrixXi &space, Vector2 v1, Vector2 v2, Vector2 v3, int value);
void fillTopFlatTriangle(Eigen::MatrixXi &space, Vector2 v1, Vector2 v2, Vector2 v3, int value);
void drawLine(Eigen::MatrixXi &space, int x1, int x2, int y, int value);

} /* namespace cad */

} /* namespace ippp */

#endif    // CADPROCESSING_H
