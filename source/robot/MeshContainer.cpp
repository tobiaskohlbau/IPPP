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

#include <fstream>
#include <iostream>
#include <sstream>

#include <core/utility/Logging.h>
#include <core/utility/Utility.h>

namespace rmpl {

/*!
*  \brief      Standard contructor of the MeshContainer class
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
MeshContainer::MeshContainer() {
    m_fclModel = nullptr;
    m_pqpModel = nullptr;
}

/*!
*  \brief      Contructor of the MeshContainer class, loads the mesh files from the passed file path
*  \param[in]  filePath
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
MeshContainer::MeshContainer(std::string filePath) {
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
MeshContainer::MeshContainer(std::shared_ptr<FCLModel>& fclModel, std::shared_ptr<PQP_Model>& pqpModel) {
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
    Eigen::Vector3f vertice;
    std::vector<Eigen::Vector3f> vertices;
    int num;
    std::vector<int> face;
    std::vector<std::vector<int>> faces;

    if (false) {
        std::ifstream input(filePath);
        for (std::string line; getline(input, line);) {
            if (line.at(0) == 'v') {
                line = line.substr(2);
                std::stringstream iss(line);
                for (int i = 0; i < 3; ++i) {
                    iss >> vertice[i];
                }
                vertices.push_back(vertice);
            } else if (line.at(0) == 'f') {
                line = line.substr(2);
                std::stringstream iss(line);
                face.clear();
                for (int i = 0; i < 3; ++i) {
                    iss >> num;
                    face.push_back(num - 1);
                }
                faces.push_back(face);
            }
        }
    } else {
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(filePath, aiProcess_SortByPType);
        if (!scene) {
            Logging::error("Mesh could not be loaded");
            return false;
        }

        int numMeshes = scene->mNumMeshes;
        Logging::info("Scene has: " + std::to_string(numMeshes) + " meshes");

        for (int i = 0; i < numMeshes; ++i) {
            const aiMesh* mesh = scene->mMeshes[i];

            for (int j = 0; j < mesh->mNumVertices; ++j) {
                vertice = Eigen::Vector3f(mesh->mVertices[j].x, mesh->mVertices[j].y, mesh->mVertices[j].z);
                vertices.push_back(vertice);
            }

            for (int j = 0; j < mesh->mNumFaces; ++j) {
                face.clear();
                for (int k = 0; k < mesh->mFaces[j].mNumIndices; ++k)
                    face.push_back(mesh->mFaces[j].mIndices[k]);
                faces.push_back(face);
            }
        }
    }
    m_vertices = vertices;
    m_faces = faces;
    std::cout << "Vertices size:" << vertices.size() << std::endl;
    std::cout << "Faces size:" << faces.size() << std::endl;

    m_fclModel = std::shared_ptr<FCLModel>(new FCLModel());
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
    m_pqpModel->MemUsage(1);
    return true;
}

/*!
*  \brief      Save transformed mesh by the passed T to passed path as obj file
*  \author     Sascha Kaden
*  \param[in]  path for obj file
*  \param[in]  transformations matrix for mesh
*  \param[out] binary result
*  \date       2016-07-14
*/
bool MeshContainer::saveObj(const std::string path, Eigen::Matrix4f T) {
    std::vector<Eigen::Vector3f> verts;
    for (auto vertice : m_vertices) {
        Eigen::Vector4f temp(utilVec::append<3>(vertice, (float)1));
        temp = T * temp;
        verts.push_back(Eigen::Vector3f(temp(0), temp(1), temp(2)));
    }

    std::ofstream myfile;
    myfile.open(path);
    for (auto vertice : verts) {
        myfile << "v " << vertice[0] << " " << vertice[1] << " " << vertice[2];
        myfile << std::endl;
    }
    for (auto face : m_faces) {
        myfile << "f " << face[0] + 1 << " " << face[1] + 1 << " " << face[2] + 1;
        myfile << std::endl;
    }
    myfile.close();
    return 0;
}

/*!
*  \brief      Return pointer to the fcl model
*  \param[out] fcl model
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
std::shared_ptr<FCLModel> MeshContainer::getFcl() {
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
        Logging::error("PQP model is empty");
    return m_pqpModel;
}

} /* namespace rmpl */