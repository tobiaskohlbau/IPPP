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

#include <ippp/environment/cad/CadProcessing.h>

#include <numeric>
#include <fstream>
#include <iostream>

#include <ippp/util/Logging.h>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {
namespace cad {

/*!
*  \brief      Generate a Triangle2D from the passed Mesh.
*  \author     Sascha Kaden
*  \param[in]  Mesh
*  \param[out] generated vector of Triangle2D
*  \date       2017-10-07
*/
std::vector<Triangle2D> generateTriangles(const Mesh &mesh) {
    std::vector<Triangle2D> triangles;
    for (auto face : mesh.faces) {
        Triangle2D tri(Vector2(mesh.vertices[face[0]][0], mesh.vertices[face[0]][1]),
                       Vector2(mesh.vertices[face[1]][0], mesh.vertices[face[1]][1]),
                       Vector2(mesh.vertices[face[2]][0], mesh.vertices[face[2]][1]));
        triangles.push_back(tri);
    }
    return triangles;
}

/*!
*  \brief      Generate a Mesh from the passed triangles. Duplicate vertices are not considered.
*  \author     Sascha Kaden
*  \param[in]  vector of triangles
*  \param[out] generated Mesh
*  \date       2017-10-07
*/
Mesh generateMesh(const std::vector<Triangle2D> &triangles) {
    Mesh mesh;
    mesh.vertices.reserve(triangles.size() * 3);
    mesh.faces.reserve(triangles.size());
    size_t vertexCount = 0;
    for (auto &tri : triangles) {
        vertexCount = mesh.vertices.size();
        for (unsigned int i = 1; i < 4; ++i) {
            auto pt = tri.getP(i);
            mesh.vertices.emplace_back(pt[0], pt[1], 0);
        }
        mesh.faces.emplace_back(vertexCount, vertexCount + 1, vertexCount + 2);
    }
    mesh.aabb = computeAABB(mesh.vertices);

    return mesh;
}

/*!
*  \brief      Merge the passed Meshes to one single Mesh.
*  \author     Sascha Kaden
*  \param[in]  vector of Meshes
*  \param[out] merged Mesh
*  \date       2017-10-07
*/
Mesh mergeMeshes(const std::vector<Mesh> &meshes) {
    Mesh mesh;
    size_t vertexCount;
    for (auto &tmpMesh : meshes) {
        vertexCount = mesh.vertices.size();
        for (auto &vertex : tmpMesh.vertices)
            mesh.vertices.push_back(vertex);
        
        for (auto &face : tmpMesh.faces)
            mesh.faces.emplace_back(face[0] + vertexCount, face[1] + vertexCount, face[2] + vertexCount);
    }
    return mesh;
}

/*!
*  \brief      Calculate center point of the vertices of a Mesh.
*  \author     Sascha Kaden
*  \param[in]  Mesh
*  \param[out] center point
*  \date       2017-10-07
*/
Vector3 calcCenterOfMesh(const Mesh &mesh) {
    Vector3 zero(0, 0, 0);
    return std::accumulate(mesh.vertices.begin(), mesh.vertices.end(), zero) / mesh.vertices.size();
}

/*!
*  \brief          Center the by reference passed meshes.
*  \author         Sascha Kaden
*  \param[in, out] vector of Meshes
*  \date           2017-10-07
*/
void centerMeshes(std::vector<Mesh> &meshes) {
    for (auto &mesh : meshes)
        centerMesh(mesh);
}

/*!
*  \brief          Center the by reference passed Mesh.
*  \author         Sascha Kaden
*  \param[in, out] Mesh
*  \date           2017-10-07
*/
void centerMesh(Mesh &mesh) {
    Vector3 centerPoint = calcCenterOfMesh(mesh);

    // set the vertices to the new center point
    for (auto &vertex : mesh.vertices)
        vertex -= centerPoint;
}

/*!
*  \brief          Transform vertices with the passed configuration, Vector with position and rotation.
*  \author         Sascha Kaden
*  \param[in]      configuration
*  \param[in, out] list of vertices
*  \date           2017-02-25
*/
void transformVertices(const Vector6 &config, std::vector<Vector3> &vertices) {
    if (config[0] == 0 && config[1] == 0 && config[2] == 0 && config[3] == 0 && config[4] == 0 && config[5] == 0)
        return;
    auto T = util::toTransform(config);
    transformVertices(T, vertices);
}

/*!
*  \brief          Transform vertices with the passed transformation matrix.
*  \author         Sascha Kaden
*  \param[in]      transformation matrix
*  \param[in, out] list of vertices
*  \date           2017-04-07
*/
void transformVertices(const Transform &T, std::vector<Vector3> &vertices) {
    for (auto &&vertex : vertices)
        vertex = T * vertex;
}

/*!
*  \brief      Compute the normals from the passed vertices and faces, duplicates are removed.
*  \author     Sascha Kaden
*  \param[in]  vertices
*  \param[in]  faces
*  \param[out] normals
*  \date       2017-02-25
*/
std::vector<Vector3> computeNormals(const std::vector<Vector3> &vertices, const std::vector<Vector3i> &faces) {
    std::vector<Vector3> normals;
    if (vertices.empty() || faces.empty())
        return normals;

    for (auto face : faces)
        normals.push_back(util::computeNormal(vertices[face[0]], vertices[face[1]], vertices[face[2]]));

    util::removeDuplicates(normals);
    return normals;
}

/*!
*  \brief      Compute the bounding box from the passed vertices and return AABB bounding box.
*  \author     Sascha Kaden
*  \param[in]  vertices
*  \param[out] AABB bounding box
*  \date       2017-04-04
*/
AABB computeAABB(const std::vector<Vector3> &vertices) {
    double min = std::numeric_limits<double>::min();
    double max = std::numeric_limits<double>::max();
    Vector3 minBoundary(max, max, max);
    Vector3 maxBoundary(min, min, min);

    for (auto vertex : vertices) {
        for (unsigned int i = 0; i < 3; ++i) {
            if (vertex[i] < minBoundary[i]) {
                minBoundary[i] = vertex[i];
            } else if (vertex[i] > maxBoundary[i]) {
                maxBoundary[i] = vertex[i];
            }
        }
    }
    return AABB(minBoundary, maxBoundary);
}

/*!
*  \brief      Compute the bounding box from the passed vertices and return AABB bounding box.
*  \author     Sascha Kaden
*  \param[in]  CadModel
*  \param[out] AABB bounding box
*  \date       2017-04-18
*/
AABB computeAABB(const Mesh &mesh) {
    return computeAABB(mesh.vertices);
}

/*!
*  \brief      Compute the bounding box from the passed vertices and return AABB bounding box.
*  \author     Sascha Kaden
*  \param[in]  CadModel
*  \param[out] AABB bounding box
*  \date       2017-04-18
*/
AABB computeAABB(const std::vector<Triangle2D> &triangles) {
    return computeAABB(generateMesh(triangles));
}

} /* namespace cad */

} /* namespace ippp */
