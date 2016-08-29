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

#include <robot/MeshContainer.h>

#include <core/Logging.h>

using namespace rmpl;

/*!
*  \brief      Standard contructor of the MeshContainer class
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
MeshContainer::MeshContainer() : Base("MeshContainer") {
    m_fclModel = nullptr;
    m_pqpModel = nullptr;
}

/*!
*  \brief      Contructor of the MeshContainer class, loads the mesh files from the passed file path
*  \param[in]  filePath
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
MeshContainer::MeshContainer(std::string filePath) : Base("MeshContainer") {
    m_fclModel = nullptr;
    m_pqpModel = nullptr;
    loadFile(filePath);
}

/*!
*  \brief      Contructor of the MeshContainer class
*  \param[in]  fcl model
*  \param[in]  pqp model
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
MeshContainer::MeshContainer(std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>>& fclModel, std::shared_ptr<PQP_Model>& pqpModel)
    : Base("MeshContainer") {
    m_fclModel = fclModel;
    m_pqpModel = pqpModel;
}

/*!
*  \brief      Load .obj file from passed path and return pointer to PQP_Model
*  \author     Sascha Kaden
*  \param[in]  path to cad file
*  \param[out] shared pointer to PQP_Model, nullptr by incorrect path
*  \date       2016-07-14
*/
bool MeshContainer::loadFile(const std::string filePath) {
    Vec<PQP_REAL> vertice;
    std::vector<Vec<PQP_REAL>> vertices;
    std::vector<int> face;
    std::vector<std::vector<int>> faces;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filePath, aiProcessPreset_TargetRealtime_MaxQuality);
    if (!scene) {
        Logging::error("Mesh could not be loaded", this);
        return false;
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


    m_fclModel = std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>>(new fcl::BVHModel<fcl::OBBRSS<float>>());
    std::vector<fcl::Vector3f> verts;
    std::vector<fcl::Triangle> triangles;
    for (auto vert : vertices)
        verts.push_back(fcl::Vector3f(vert[0], vert[1], vert[2]));
    for (auto tmp : faces)
        triangles.push_back(fcl::Triangle(tmp[0], tmp[1], tmp[2]));
    m_fclModel->beginModel();
    m_fclModel->addSubModel(verts, triangles);
    m_fclModel->endModel();

    m_pqpModel = std::shared_ptr<PQP_Model>(new PQP_Model());
    m_pqpModel->BeginModel();
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
        m_pqpModel->AddTri(p[0], p[1], p[2], i);
    }
    m_pqpModel->EndModel();
    //m_pqpModel->MemUsage(1);
}

/*!
*  \brief      Return pointer to the fcl model
*  \param[out] fcl model
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> MeshContainer::getFcl() {
    return m_fclModel;
}

/*!
*  \brief      Return pointer to the pqp model
*  \param[out] pqp model
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
std::shared_ptr<PQP_Model> MeshContainer::getPqp() {
    if (m_pqpModel == nullptr)
        Logging::warning("PQP model is empty", this);
    return m_pqpModel;
}