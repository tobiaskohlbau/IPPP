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

#include <ippp/environment/model/ModelFactoryPqp.h>

#include <ippp/util/Logging.h>

namespace ippp {

/*!
*  \brief      Standard constructor of ModelFactoryPqp
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
ModelFactoryPqp::ModelFactoryPqp() : ModelFactory("ModelFactoryPqp"){};

/*!
*  \brief      Creates an ModelPqp from the passed source cad file.
*  \details    The model contains the vertices and faces of the loaded cad model.
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] smart pointer to ModelPqp
*  \date       2017-02-19
*/
std::shared_ptr<ModelContainer> ModelFactoryPqp::createModelFromFile(const std::string &filePath) {
    if (filePath.empty()) {
        Logging::error("Empty file path", this);
        return nullptr;
    }

    // create model container properties
    std::shared_ptr<ModelPqp> pqpModel(new ModelPqp());
    if (!cad::importMesh(filePath, pqpModel->m_mesh)) {
        Logging::error("Could not load mesh", this);
        return nullptr;
    }
    pqpModel->m_mesh.aabb = cad::computeAABB(pqpModel->m_mesh);

    // create PQP model
    pqpModel->m_pqpModel.BeginModel();
    // create pqp triangles
    
    for (size_t i = 0; i < pqpModel->m_mesh.faces.size(); ++i) {
        PQP_REAL p[3][3];
        // go through faces
        for (size_t j = 0; j < 3; ++j) {
            // go through face
            size_t vertex = pqpModel->m_mesh.faces[i][j];
            for (size_t k = 0; k < 3; ++k) {
                p[j][k] = pqpModel->m_mesh.vertices[vertex][k];
            }
        }
        pqpModel->m_pqpModel.AddTri(p[0], p[1], p[2], static_cast<int>(i));
    }
    pqpModel->m_pqpModel.EndModel();

    if (Logging::getLogLevel() == LogLevel::trace)
        pqpModel->m_pqpModel.MemUsage(1);
    return pqpModel;
}

std::vector<std::shared_ptr<ModelContainer>> ModelFactoryPqp::createModelsFromFile(const std::string &filePath) {
    std::vector<std::shared_ptr<ModelContainer>> models;
    if (filePath.empty()) {
        Logging::error("Empty file path", this);
        return models;
    }
    // create model container properties
    std::vector<Mesh> meshes;

    if (!cad::importMeshes(filePath, meshes)) {
        Logging::error("Could not load mesh", this);
        return models;
    }

    for (const auto &mesh : meshes) {
        std::shared_ptr<ModelPqp> pqpModel(new ModelPqp());
        pqpModel->m_mesh = mesh;
        pqpModel->m_mesh.aabb = cad::computeAABB(pqpModel->m_mesh);

        // create PQP model
        pqpModel->m_pqpModel.BeginModel();
        // create pqp triangles
        PQP_REAL p[3][3];
        for (size_t i = 0; i < pqpModel->m_mesh.faces.size(); ++i) {
            // go through faces
            for (size_t j = 0; j < 3; ++j) {
                // go through face
                size_t vertex = pqpModel->m_mesh.faces[i][j];
                for (size_t k = 0; k < 3; ++k) {
                    p[j][k] = pqpModel->m_mesh.vertices[vertex][k];
                }
            }
            pqpModel->m_pqpModel.AddTri(p[0], p[1], p[2], static_cast<int>(i));
        }
        pqpModel->m_pqpModel.EndModel();
        if (Logging::getLogLevel() == LogLevel::debug) {
            pqpModel->m_pqpModel.MemUsage(1);
        }
        models.push_back(pqpModel);
    }
    return models;
}

/*!
*  \brief      Creates a list of pqp models from the passed source cad files.
*  \author     Sascha Kaden
*  \param[in]  filePaths
*  \param[out] list of pqp models
*  \date       2017-02-19
*/
std::vector<std::shared_ptr<ModelContainer>> ModelFactoryPqp::createModelsFromFiles(const std::vector<std::string> &filePaths) {
    std::vector<std::shared_ptr<ModelContainer>> models;
    std::shared_ptr<ModelContainer> model;
    for (const auto &filePath : filePaths) {
        model = createModelFromFile(filePath);
        if (model)
            models.push_back(model);
        else
            return std::vector<std::shared_ptr<ModelContainer>>();
    }
    return models;
}

} /* namespace ippp */
