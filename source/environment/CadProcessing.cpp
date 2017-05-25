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

#include "include/environment/CadProcessing.h"

#include <fstream>

#include <core/util/Logging.h>
#include <core/util/UtilGeo.hpp>

namespace ippp {
namespace cad {

/*!
*  \brief      Import a cad model (vertices and faces) with the assimp library
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] Mesh
*  \date       2017-05-09
*/
bool importMesh(const std::string &filePath, Mesh &mesh, const bool calcNormals) {
    std::size_t found = filePath.find_last_of(".");
    if (filePath.substr(found) == ".g") {
        return importBYU(filePath, mesh);
    }

    std::vector<Mesh> meshes;
    if (!importMeshes(filePath, meshes)) {
        return false;
    }

    // merge meshes to one single mesh
    size_t verticeCount;
    for (auto tmpMesh : meshes) {
        verticeCount = mesh.vertices.size();
        for (auto vertex : tmpMesh.vertices) {
            mesh.vertices.push_back(vertex);
        }
        for (auto face : tmpMesh.faces) {
            mesh.faces.push_back(Vector3i(face[0] + verticeCount, face[1] + verticeCount, face[2] + verticeCount));
        }
    }

    if (calcNormals)
        mesh.normals = computeNormals(mesh.vertices, mesh.faces);
    return true;
}

/*!
*  \brief      Import meshes (vertices and faces) with the assimp library
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] Mesh
*  \date       2017-05-09
*/
bool importMeshes(const std::string &filePath, std::vector<Mesh> &meshes, const bool calcNormals) {
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(filePath, aiProcess_CalcTangentSpace);
    if (!scene) {
        Logging::error("Could not load cad", "CadProcessing");
        Logging::error("Assimp message: " + std::string(importer.GetErrorString()), "CadProcessing");
        return false;
    }

    if (scene->mNumMeshes == 0) {
        Logging::error("Scene contains no meshes", "CadProcessing");
        return false;
    }
    Logging::info("File has: " + std::to_string(scene->mNumMeshes) + " meshes", "CadProcessing");
    meshes.clear();

    aiMatrix4x4 trafo;
    aiIdentityMatrix4(&trafo);
    getMeshes(scene, scene->mRootNode, &trafo, meshes);

    if (calcNormals) {
        for (auto mesh : meshes) {
            mesh.normals = computeNormals(mesh.vertices, mesh.faces);
        }
    }

    return true;
}

/*!
*  \brief      Go through the scene nodes and load and transform all meshes them.
*  \author     Sascha Kaden
*  \param[in]  scene
*  \param[in]  node
*  \param[in]  transformation
*  \param[out] vector of meshes
*  \date       2017-05-09
*/
void getMeshes(const aiScene *scene, const aiNode *node, aiMatrix4x4 *trafo, std::vector<Mesh> &meshes) {
    aiMatrix4x4 prevTrafo;

    prevTrafo = *trafo;
    aiMultiplyMatrix4(trafo, &node->mTransformation);

    for (int i = 0; i < node->mNumMeshes; ++i) {
        Mesh mesh;
        const aiMesh *aimesh = scene->mMeshes[node->mMeshes[i]];
        for (int j = 0; j < aimesh->mNumVertices; ++j) {
            aiVector3D vertex = aimesh->mVertices[j];
            aiTransformVecByMatrix4(&vertex, trafo);
            mesh.vertices.push_back(Vector3(vertex.x, vertex.y, vertex.z));
        }
        for (int j = 0; j < aimesh->mNumFaces; ++j) {
            if (aimesh->mFaces[j].mNumIndices > 2) {
                mesh.faces.push_back(
                    Vector3i(aimesh->mFaces[j].mIndices[0], aimesh->mFaces[j].mIndices[1], aimesh->mFaces[j].mIndices[2]));
            } else {
                Logging::warning("Face array is to short", "CadProcessing");
            }
        }
        meshes.push_back(mesh);
    }

    for (int i = 0; i < node->mNumChildren; ++i) {
        getMeshes(scene, node->mChildren[i], trafo, meshes);
    }
    *trafo = prevTrafo;
}

/*!
*  \brief      Import BYU cad model (vertices and faces)
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] list of vertices
*  \param[out] list of faces
*  \date       2017-02-19
*/
bool importBYU(const std::string &filePath, Mesh &mesh) {
    std::ifstream is(filePath);
    std::string str;
    unsigned int numBodies = 0;
    unsigned int numVertices = 0;
    unsigned int numFaces = 0;

    getline(is, str);
    util::trimWhitespaces(str);
    std::istringstream ss(str);
    ss >> numBodies;
    ss >> numVertices;
    ss >> numFaces;
    mesh.vertices.clear();
    mesh.faces.clear();
    mesh.vertices.reserve(numVertices);
    mesh.faces.reserve(numFaces);

    getline(is, str);
    getline(is, str);
    util::trimWhitespaces(str);
    for (unsigned int i = 0; i < numVertices; ++i) {
        Vector3 vec;
        std::string::size_type sz;
        for (unsigned int j = 0; j < 3; ++j) {
            vec[j] = std::stof(str, &sz);
            str = str.substr(sz);
        }
        mesh.vertices.push_back(vec);
        if (str.size() < 2) {
            getline(is, str);
            util::trimWhitespaces(str);
        }
    }
    for (unsigned int i = 0; i < numFaces; ++i) {
        util::trimWhitespaces(str);
        Vector3i vec;
        std::string::size_type sz;
        for (unsigned int j = 0; j < 3; ++j) {
            vec[j] = std::stoi(str, &sz);
            if (vec[j] < 0) {
                vec[j] = -vec[j] - 1;
                break;
            }
            vec[j] -= 1;
            str = str.substr(sz);
        }
        mesh.faces.push_back(vec);
        getline(is, str);
    }
    mesh.normals = computeNormals(mesh.vertices, mesh.faces);

    return true;
}

/*!
*  \brief      Export a cad model (vertices and faces) with the assimp library
*  \author     Sascha Kaden
*  \param[in]  format
*  \param[in]  filePath
*  \param[in]  list of vertices
*  \param[in]  list of faces
*  \date       2017-02-19
*/
bool exportCad(ExportFormat format, const std::string &filePath, const Mesh &mesh) {
    return exportCad(format, filePath, mesh.vertices, mesh.faces);
}

/*!
*  \brief      Export a cad model (vertices and faces) with the assimp library
*  \author     Sascha Kaden
*  \param[in]  format
*  \param[in]  filePath
*  \param[in]  list of vertices
*  \param[in]  list of faces
*  \date       2017-02-19
*/
bool exportCad(ExportFormat format, const std::string &filePath, const std::vector<Vector3> &vertices,
               const std::vector<Vector3i> &faces) {
    if (filePath == "") {
        Logging::warning("Empty output file path", "CadProcessing");
        return false;
    }

    Assimp::Exporter exporter;
    size_t maxFormatCount = exporter.GetExportFormatCount();
    size_t formatCount;
    switch (format) {
        case ExportFormat::COLLADA:
            formatCount = 0;
            break;
        case ExportFormat::X_FILES:
            formatCount = 1;
            break;
        case ExportFormat::STEP:
            formatCount = 2;
            break;
        case ExportFormat::OBJ:
            formatCount = 3;
            break;
        case ExportFormat::STEREOLITHOGRAPHY:
            formatCount = 4;
            break;
        case ExportFormat::STEREOLITHOGRAPHY_BINARY:
            formatCount = 5;
            break;
        case ExportFormat::STANFORD_POLYGON_LIBRARY:
            formatCount = 6;
            break;
        case ExportFormat::STANFORD_POLYGON_LIBRARY_BINARY:
            formatCount = 7;
            break;
        case ExportFormat::AUTODESK_3DS:
            formatCount = 8;
            break;
        case ExportFormat::GL_TRANSMISSION_FORMAT:
            formatCount = 9;
            break;
        case ExportFormat::GL_TRANSMISSION_FORMAT_BINARY:
            formatCount = 10;
            break;
        case ExportFormat::ASSIMP_BINARY:
            formatCount = 11;
            break;
        case ExportFormat::ASSXML_DOCUMENT:
            formatCount = 12;
            break;
        case ExportFormat::EXTENSIBLE_3D:
            formatCount = 13;
            break;
    }

    if (formatCount > maxFormatCount) {
        Logging::warning("Selected format is not supported", "CadProcessing");
        return false;
    }

    aiScene scene = generateScene(vertices, faces);

    const aiExportFormatDesc *formatDesc = exporter.GetExportFormatDescription(formatCount);
    exporter.Export(&scene, formatDesc->id, filePath + "." + formatDesc->fileExtension, aiProcess_Triangulate);
    return true;
}

Eigen::MatrixXi create2dspace(const AABB &boundary, const int fillValue) {
    Vector3 dia = boundary.diagonal();
    Eigen::MatrixXi space = Eigen::MatrixXi::Constant(dia[1], dia[2], fillValue);
    return space;
}

void drawTriangles(Eigen::MatrixXi &space, const std::vector<Triangle2D> &triangles, const int fillValue) {
    for (auto triangle : triangles) {
        std::vector<Vector2> points;
        for (int i = 1; i < 4; ++i) {
            points.push_back(triangle.getP(i));
        }
        struct Vector2Sorter {
            bool operator()(const Vector2 &lhs, const Vector2 &rhs) const {
                return lhs[1] < rhs[1];
            }
        };
        std::sort(std::begin(points), std::end(points), Vector2Sorter());
        Vector2 v1 = points[0];
        Vector2 v2 = points[1];
        Vector2 v3 = points[2];

        if (v2[1] == v3[1]) {
            fillBottomFlatTriangle(space, v1, v2, v3, fillValue);
        }
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

/*!
*  \brief           Transform vertices with the passed configuration, Vector with position and rotation.
*  \author          Sascha Kaden
*  \param[in]       configuration
*  \param[in, out]  list of vertices
*  \date            2017-02-25
*/
void transformVertices(const Vector6 &config, std::vector<Vector3> &vertices) {
    if (config[0] == 0 && config[1] == 0 && config[2] == 0 && config[3] == 0 && config[4] == 0 && config[5] == 0) {
        return;
    }
    Matrix3 R;
    Vector3 t;
    util::poseVecToRandT(config, R, t);
    for (auto &&vertex : vertices) {
        vertex = (R * vertex) + t;
    }
}

/*!
*  \brief           Transform vertices with the passed transformation matrix.
*  \author          Sascha Kaden
*  \param[in]       transformation matrix
*  \param[in, out]  list of vertices
*  \date            2017-04-07
*/
void transformVertices(const Matrix4 &T, std::vector<Vector3> &vertices) {
    Matrix3 R;
    Vector3 t;
    util::decomposeT(T, R, t);
    for (auto &&vertex : vertices) {
        vertex = (R * vertex) + t;
    }
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
    if (vertices.empty() || faces.empty()) {
        return normals;
    }

    for (auto face : faces) {
        normals.push_back(util::computeNormal(vertices[face[0]], vertices[face[1]], vertices[face[2]]));
    }
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
*  \brief      Generates a assimp aiScene from passed vertices and faces
*  \author     Sascha Kaden
*  \param[in]  list of vertices
*  \param[in]  list of faces
*  \param[out] scene
*  \date       2017-02-19
*/
aiScene generateScene(const std::vector<Vector3> &vertices, const std::vector<Vector3i> &faces) {
    aiScene scene;

    scene.mRootNode = new aiNode();

    scene.mMaterials = new aiMaterial *[1];
    scene.mMaterials[0] = nullptr;
    scene.mNumMaterials = 1;
    scene.mMaterials[0] = new aiMaterial();

    scene.mMeshes = new aiMesh *[1];
    scene.mMeshes[0] = nullptr;
    scene.mNumMeshes = 1;
    scene.mMeshes[0] = new aiMesh();
    scene.mMeshes[0]->mMaterialIndex = 0;

    scene.mRootNode->mMeshes = new unsigned int[1];
    scene.mRootNode->mMeshes[0] = 0;
    scene.mRootNode->mNumMeshes = 1;

    auto pMesh = scene.mMeshes[0];

    // add vertices to the mesh
    pMesh->mVertices = new aiVector3D[vertices.size()];
    pMesh->mNumVertices = vertices.size();
    for (size_t i = 0; i < vertices.size(); ++i) {
        pMesh->mVertices[i] = aiVector3D(vertices[i].x(), vertices[i].y(), vertices[i].z());
    }

    pMesh->mFaces = new aiFace[faces.size()];
    pMesh->mNumFaces = faces.size();
    for (size_t i = 0; i < faces.size(); ++i) {
        aiFace &face = pMesh->mFaces[i];

        face.mIndices = new unsigned int[3];
        face.mNumIndices = 3;
        for (int j = 0; j < 3; ++j) {
            face.mIndices[j] = faces[i][j];
        }
    }
    return scene;
}

void fillBottomFlatTriangle(Eigen::MatrixXi &space, Vector2 v1, Vector2 v2, Vector2 v3, int value) {
    double invslope1 = (v2[0] - v1[0]) / (v2[1] - v1[1]);
    double invslope2 = (v3[0] - v1[0]) / (v3[1] - v1[1]);

    double curx1 = v1[0];
    double curx2 = v1[0];

    for (int scanlineY = v1[1]; scanlineY <= v2[1]; scanlineY++) {
        drawLine(space, (int)curx1, (int)curx2, scanlineY, value);
        curx1 += invslope1;
        curx2 += invslope2;
    }
}

void fillTopFlatTriangle(Eigen::MatrixXi &space, Vector2 v1, Vector2 v2, Vector2 v3, int value) {
    double invslope1 = (v3[0] - v1[0]) / (v3[1] - v1[1]);
    double invslope2 = (v3[0] - v2[0]) / (v3[1] - v2[1]);

    double curx1 = v3[0];
    double curx2 = v3[0];

    for (int scanlineY = v3[1]; scanlineY > v1[1]; scanlineY--) {
        drawLine(space, (int)curx1, (int)curx2, scanlineY, value);
        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

void drawLine(Eigen::MatrixXi &space, int x1, int x2, int y, int value) {
    if (x1 > x2) {
        std::swap(x1, x2);
    }
    for (int x = x1; x <= x2; ++x) {
        if (0 > y || y > space.rows() || 0 > x || x > space.cols()) {
            Logging::debug("Triangle out of space", "CadProcessing");
            continue;
        }
        space(y, x) = value;
    }
}

} /* namespace cad */

} /* namespace ippp */
