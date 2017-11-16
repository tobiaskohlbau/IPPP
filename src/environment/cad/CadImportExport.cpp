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

#include <ippp/environment/cad/CadImportExport.h>

#include <fstream>
#include <iostream>

#include <ippp/util/Logging.h>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {
namespace cad {

/*!
*  \brief      Import a cad model (vertices and faces) with the assimp library
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] Mesh
*  \date       2017-05-09
*/
bool importMesh(const std::string &filePath, Mesh &mesh, const double scale, const bool calcNormals, const bool useTrafo) {
    std::size_t found = filePath.find_last_of(".");
    if (filePath.substr(found) == ".g")
        return importBYU(filePath, mesh);

    std::vector<Mesh> meshes;
    if (!importMeshes(filePath, meshes, scale, calcNormals, useTrafo))
        return false;

    // merge meshes to one single mesh
    mesh = mergeMeshes(meshes);

    return true;
}

/*!
*  \brief      Import meshes (vertices and faces) with the assimp library
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] Mesh
*  \date       2017-05-09
*/
bool importMeshes(const std::string &filePath, std::vector<Mesh> &meshes, const double scale, const bool calcNormals,
                  const bool useTrafo) {
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
    Logging::info("File has: " + std::to_string(scene->mNumMeshes) + " mesh(es)", "CadProcessing");
    meshes.clear();

    aiMatrix4x4 trafo;
    aiIdentityMatrix4(&trafo);
    getMeshes(scene, scene->mRootNode, &trafo, meshes, useTrafo);

    for (auto &&mesh : meshes)
        for (auto &&vertex : mesh.vertices)
            vertex *= scale;

    if (calcNormals)
        for (auto mesh : meshes)
            mesh.normals = computeNormals(mesh.vertices, mesh.faces);

    return true;
}

Matrix4 importTransformation(const std::string &filePath) {
    Matrix4 X = Matrix4::Zero(4, 4);
    std::ifstream fin(filePath);

    if (fin.is_open()) {
        for (size_t row = 0; row < 4; row++) {
            for (size_t col = 0; col < 4; col++) {
                float item = 0.0;
                fin >> item;
                X(row, col) = item;
            }
        }
        fin.close();
    }
    std::cout << "X = " << std::endl << X << std::endl;
    return X;
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
void getMeshes(const aiScene *scene, const aiNode *node, aiMatrix4x4 *trafo, std::vector<Mesh> &meshes, const bool useTrafo) {
    aiMatrix4x4 prevTrafo;

    prevTrafo = *trafo;
    aiMultiplyMatrix4(trafo, &node->mTransformation);

    for (size_t i = 0; i < node->mNumMeshes; ++i) {
        Mesh mesh;
        const aiMesh *aimesh = scene->mMeshes[node->mMeshes[i]];
        for (size_t j = 0; j < aimesh->mNumVertices; ++j) {
            aiVector3D vertex = aimesh->mVertices[j];
            if (useTrafo)
                aiTransformVecByMatrix4(&vertex, trafo);
            mesh.vertices.push_back(Vector3(vertex.x, vertex.y, vertex.z));
        }
        for (size_t j = 0; j < aimesh->mNumFaces; ++j) {
            if (aimesh->mFaces[j].mNumIndices > 2) {
                mesh.faces.push_back(
                    Vector3i(aimesh->mFaces[j].mIndices[0], aimesh->mFaces[j].mIndices[1], aimesh->mFaces[j].mIndices[2]));
            } else {
                Logging::warning("Face array is to short", "CadProcessing");
            }
        }
        meshes.push_back(mesh);
    }

    for (size_t i = 0; i < node->mNumChildren; ++i) {
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
    size_t numBodies = 0;
    size_t numVertices = 0;
    size_t numFaces = 0;

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
    while (str == "")
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
    while (str == "")
        getline(is, str);
    for (unsigned int i = 0; i < numFaces; ++i) {
        util::trimWhitespaces(str);
        Vector3i vec;
        std::string::size_type sz;
        for (unsigned int j = 0; j < 3; ++j) {
            int index = std::stoi(str, &sz);
            if (index < 0) {
                vec[j] = -index - 1;
                break;
            } else {
                vec[j] = index;
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
    pMesh->mNumVertices = static_cast<unsigned int>(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i)
        pMesh->mVertices[i] = aiVector3D(static_cast<ai_real>(vertices[i].x()), static_cast<ai_real>(vertices[i].y()), static_cast<ai_real>(vertices[i].z()));

    pMesh->mFaces = new aiFace[faces.size()];
    pMesh->mNumFaces = static_cast<unsigned int>(faces.size());
    for (size_t i = 0; i < faces.size(); ++i) {
        aiFace &face = pMesh->mFaces[i];

        face.mIndices = new unsigned int[3];
        face.mNumIndices = 3;
        for (size_t j = 0; j < 3; ++j)
            face.mIndices[j] = static_cast<unsigned int>(faces[i][j]);
    }
    return scene;
}

} /* namespace cad */

} /* namespace ippp */
