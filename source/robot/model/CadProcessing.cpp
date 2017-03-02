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
bool importCad(const std::string &filePath, std::vector<Vector3> &vertices, std::vector<Vector3i> &faces) {
    std::size_t found = filePath.find_last_of(".");
    if (filePath.substr(found) == ".g") {
        return importBYU(filePath, vertices, faces);
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
    unsigned int verticeCount;
    for (int i = 0; i < scene->mNumMeshes; ++i) {
        const aiMesh *mesh = scene->mMeshes[i];
        verticeCount = vertices.size();
        // load vertices
        for (int j = 0; j < mesh->mNumVertices; ++j) {
            vertices.push_back(Vector3(mesh->mVertices[j].x, mesh->mVertices[j].y, mesh->mVertices[j].z));
        }
        // load faces
        for (int j = 0; j < mesh->mNumFaces; ++j) {
            if (mesh->mFaces[j].mNumIndices > 2) {
                faces.push_back(Vector3i(mesh->mFaces[j].mIndices[0] + verticeCount, mesh->mFaces[j].mIndices[1] + verticeCount,
                                         mesh->mFaces[j].mIndices[2] + verticeCount));
            } else {
                Logging::warning("Face array is to short", "CadProcessing");
            }
        }
    }
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
bool importBYU(const std::string &filePath, std::vector<Vector3> &vertices, std::vector<Vector3i> &faces) {
    std::ifstream is(filePath);
    std::string str;
    unsigned int numBodies = 0;
    unsigned int numVertices = 0;
    unsigned int numFaces = 0;
    vertices.clear();
    faces.clear();

    getline(is, str);
    utilList::trimWhitespaces(str);
    std::istringstream ss(str);
    ss >> numBodies;
    ss >> numVertices;
    ss >> numFaces;
    getline(is, str);
    getline(is, str);
    utilList::trimWhitespaces(str);
    std::size_t found;
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
            utilList::trimWhitespaces(str);
        }
    }
    for (unsigned int i = 0; i < numFaces; ++i) {
        utilList::trimWhitespaces(str);
        Vector3i vec;
        std::string::size_type sz;
        for (unsigned int j = 0; j < 3; ++j) {
            vec[j] = std::stoi(str, &sz) - 1;
            if (vec[j] < 0) {
                vec[j] = -vec[j];
                break;
            }
            str = str.substr(sz);
        }
        faces.push_back(vec);
        getline(is, str);
    }
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
    Matrix3 R = utilGeo::getRotMat3D(config[3], config[4], config[5]);
    Vector3 t(config[0], config[1], config[2]);
    for (auto vertice : vertices) {
        vertice = (R * vertice) + t;
    }
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
    for (auto itVertice = vertices.begin(); itVertice != vertices.end(); ++itVertice) {
        pMesh->mVertices[itVertice - vertices.begin()] = aiVector3D(itVertice->x(), itVertice->y(), itVertice->z());
    }

    pMesh->mFaces = new aiFace[faces.size()];
    pMesh->mNumFaces = faces.size();
    for (auto itFace = faces.begin(); itFace != faces.end(); ++itFace) {
        aiFace &face = pMesh->mFaces[itFace - faces.begin()];

        face.mIndices = new unsigned int[3];
        face.mNumIndices = 3;
        for (int i = 0; i < 3; ++i) {
            face.mIndices[i] = (*itFace)[i];
        }
    }
    return scene;
}

} /* namespace rmpl */
