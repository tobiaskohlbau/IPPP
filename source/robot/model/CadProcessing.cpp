//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#include "include/robot/model/CadProcessing.h"

#include <fstream>

#include <core/utility/Logging.h>
#include <core/utility/UtilGeo.hpp>

namespace rmpl {

/*!
*  \brief      Import a cad model (vertices and faces) with the assimp library
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] list of vertices
*  \param[out] list of faces
*  \date       2017-02-19
*/
bool importCad(const std::string &filePath, std::vector<Vector3> &vertices, std::vector<Vector3i> &faces,
               std::vector<Vector3> &normals) {
    std::size_t found = filePath.find_last_of(".");
    if (filePath.substr(found) == ".g") {
        return importBYU(filePath, vertices, faces, normals);
    }

    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(filePath, aiProcess_CalcTangentSpace);
    if (!scene) {
        Logging::error("Could not load cad", "CadProcessing");
        return false;
    }

    if (scene->mNumMeshes == 0) {
        return false;
    }
    vertices.clear();
    faces.clear();
    size_t verticeCount;
    for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
        const aiMesh *mesh = scene->mMeshes[i];
        verticeCount = vertices.size();
        // load vertices
        for (unsigned int j = 0; j < mesh->mNumVertices; ++j) {
            vertices.push_back(Vector3(mesh->mVertices[j].x, mesh->mVertices[j].y, mesh->mVertices[j].z));
        }
        // load faces
        for (unsigned int j = 0; j < mesh->mNumFaces; ++j) {
            if (mesh->mFaces[j].mNumIndices > 2) {
                faces.push_back(Vector3i(mesh->mFaces[j].mIndices[0] + verticeCount, mesh->mFaces[j].mIndices[1] + verticeCount,
                                         mesh->mFaces[j].mIndices[2] + verticeCount));
            } else {
                Logging::warning("Face array is to short", "CadProcessing");
            }
        }
    }
    normals = computeNormals(vertices, faces);
    return true;
}

/*!
*  \brief      Import BYU cad model (vertices and faces)
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] list of vertices
*  \param[out] list of faces
*  \date       2017-02-19
*/
bool importBYU(const std::string &filePath, std::vector<Vector3> &vertices, std::vector<Vector3i> &faces,
               std::vector<Vector3> &normals) {
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
    vertices.clear();
    faces.clear();
    vertices.reserve(numVertices);
    faces.reserve(numFaces);

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
        vertices.push_back(vec);
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
        faces.push_back(vec);
        getline(is, str);
    }
    normals = computeNormals(vertices, faces);

    return true;
}

/*!
*  \brief      Export a cad model (vertices and faces) with the assimp library
*  \author     Sascha Kaden
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
    exporter.Export(&scene, formatDesc->id, filePath + "." + formatDesc->fileExtension, 0);
    return true;
}

/*!
*  \brief           Transform vertices with the passed configuration, Vector with position and rotation.
*  \author          Sascha Kaden
*  \param[in]       configuration
*  \param[in, out]  list of vertices
*  \date            2017-02-25
*/
void transformCad(const Vector6 &config, std::vector<Vector3> &vertices) {
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

std::vector<Vector3> computeNormals(const std::vector<Vector3> &vertices, const std::vector<Vector3i> &faces) {
    std::vector<Vector3> normals;
    if (vertices.empty() || faces.empty()) {
        return normals;
    }

    for (auto face : faces) {
        normals.push_back(util::computeNormal(vertices[face[0]], vertices[face[1]], vertices[face[2]]));
    }
    // sort normal list
    struct {
        bool operator()(Vector3 a, Vector3 b) {
            return a.x() < b.x();
        }
    } customCompare;
    std::sort(normals.begin(), normals.end(), customCompare);

    // remove duplicates
    for (auto normal = normals.begin(); normal != normals.end(); ++normal) {
        int i = 1;
        while(normal+i != normals.end() && normal->x() - (normal+i)->x() == 0) {
            if ((*normal - *(normal + i)).norm() < 0.0001) {
                normals.erase(normal + i);
            }
            else {
                ++i;
            }
        }
    }
    return normals;
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

} /* namespace rmpl */
