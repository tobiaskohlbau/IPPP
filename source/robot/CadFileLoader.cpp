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

#include <robot/CadFileLoader.h>

#include <core/Logging.h>
#include <include/core/Vec.hpp>

using namespace rmpl;

/*!
*  \brief      Constructor of the class CadFileLoader
*  \author     Sascha Kaden
*  \date       2016-07-12
*/
CadFileLoader::CadFileLoader() : Base("CadFileLoader") {
}

/*!
*  \brief      Load cad file from passed path and return pointer to PQP_Model
*  \author     Sascha Kaden
*  \param[in]  path to cad file
*  \param[out] shared pointer to PQP_Model, nullptr by incorrect path
*  \date       2016-07-14
*/
std::shared_ptr<PQP_Model> CadFileLoader::loadFile(const std::string filename) {
    std::string extension = getFileExt(filename);

    if (extension == "obj")
        return readObj(filename);
    else
        Logging::error("File type is not supported", this);

    return nullptr;
}

/*!
*  \brief      Load .obj file from passed path and return pointer to PQP_Model
*  \author     Sascha Kaden
*  \param[in]  path to cad file
*  \param[out] shared pointer to PQP_Model, nullptr by incorrect path
*  \date       2016-07-14
*/
std::shared_ptr<PQP_Model> CadFileLoader::readObj(const std::string filename) {
    Vec<PQP_REAL> vertice;
    std::vector<Vec<PQP_REAL>> vertices;
    std::vector<int> face;
    std::vector<std::vector<int>> faces;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcessPreset_TargetRealtime_MaxQuality);
    if (!scene) {
        Logging::error("Mesh could not be loaded", this);
        return nullptr;
    }

    int numMeshes = scene->mNumMeshes;
    Logging::info("Scene has: " + std::to_string(numMeshes) + " meshes", this);

    for (int i = 0; i < numMeshes; ++i) {
        const aiMesh* mesh = scene->mMeshes[i];

        for (int j = 0; j < mesh->mNumVertices; ++j) {
            vertice = Vec<PQP_REAL>(mesh->mVertices[j].x, mesh->mVertices[j].y, mesh->mVertices[j].z);
            vertices.push_back(vertice);
        }

        for (int j = 0; j < mesh->mNumFaces; ++j) {
            face.clear();
            for (int k = 0; k < mesh->mFaces[j].mNumIndices; ++k) {
                face.push_back(mesh->mFaces[j].mIndices[k]);
                face.push_back(mesh->mFaces[j].mIndices[k]);
                face.push_back(mesh->mFaces[j].mIndices[k]);
                faces.push_back(face);
            }
        }
    }

    std::shared_ptr<PQP_Model> model(new PQP_Model());
    model->BeginModel();
    // create pqp triangles
    PQP_REAL p[3][3];
    for (int i = 0; i < faces.size(); ++i) {
        // go through faces
        for (int j = 0; j < 3; ++j) {
            // go through face
            int vert = faces[i][j];
            for (int k = 0; k < 3; ++k) {
                p[j][k] = vertices[vert][k];
            }
        }
        model->AddTri(p[0], p[1], p[2], i);
    }
    model->EndModel();
    model->MemUsage(1);

    return model;
}

/*!
*  \brief      Return file extension from input path
*  \author     Sascha Kaden
*  \param[in]  path
*  \param[out] file extension
*  \date       2016-07-14
*/
std::string CadFileLoader::getFileExt(const std::string& s) {
    size_t i = s.rfind('.', s.length());

    if (i != std::string::npos) {
        std::string ext = s.substr(i + 1, s.length() - i);
        for (std::basic_string<char>::iterator p = ext.begin(); p != ext.end(); ++p)
            *p = tolower(*p);
        return ext;
    }

    return ("");
}
