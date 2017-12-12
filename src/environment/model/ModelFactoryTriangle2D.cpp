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

#include <ippp/environment/model/ModelFactoryTriangle2D.h>

namespace ippp {

/*!
*  \brief      Standard constructor of ModelFactoryTriangle2D
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
ModelFactoryTriangle2D::ModelFactoryTriangle2D() : ModelFactory("ModelFactoryTriangle"){};

/*!
*  \brief      Creates a ModelTriangle2D from the passed source cad file.
*  \details    The model contains the vertices and faces of the loaded cad model.
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] smart pointer to ModelTriangle2D
*  \date       2017-02-19
*/
std::shared_ptr<ModelContainer> ModelFactoryTriangle2D::createModelFromFile(const std::string &filePath) {
    if (filePath.empty()) {
        Logging::error("Empty file path", this);
        return nullptr;
    }

    std::shared_ptr<ModelTriangle2D> triangleModel(new ModelTriangle2D());
    if (!cad::importMesh(filePath, triangleModel->m_mesh)) {
        Logging::error("Could not load mesh", this);
        return nullptr;
    }
    for (auto vertex : triangleModel->m_mesh.vertices)
        vertex[2] = 0;
    triangleModel->m_mesh.aabb = cad::computeAABB(triangleModel->m_mesh);
    triangleModel->m_triangles = cad::generateTriangles(triangleModel->m_mesh);

    return triangleModel;
}

/*!
*  \brief      Creates a list of ModelTriangle2D from the passed source cad file.
*  \details    Each model contains the vertices and faces of the loaded cad model.
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] list of smart pointer to the ModelTriangle2D
*  \date       2017-02-19
*/
std::vector<std::shared_ptr<ModelContainer>> ModelFactoryTriangle2D::createModelsFromFile(const std::string &filePath) {
    std::vector<std::shared_ptr<ModelContainer>> models;
    if (filePath.empty()) {
        Logging::error("Empty file path", this);
        return models;
    }
    std::vector<Mesh> meshes;
    if (!cad::importMeshes(filePath, meshes)) {
        Logging::error("Could not load mesh", this);
        return models;
    }

    for (const auto& mesh : meshes) {
        std::shared_ptr<ModelTriangle2D> triangleModel(new ModelTriangle2D());
        triangleModel->m_mesh = mesh;
        for (auto vertex : triangleModel->m_mesh.vertices)
            vertex[2] = 0;
        triangleModel->m_mesh.aabb = cad::computeAABB(mesh);
        triangleModel->m_triangles = cad::generateTriangles(triangleModel->m_mesh);
        models.push_back(triangleModel);
    }
    return models;
}

/*!
*  \brief      Creates a list of triangle models from the passed source cad files.
*  \author     Sascha Kaden
*  \param[in]  filePaths
*  \param[out] list of triangle models
*  \date       2017-02-19
*/
std::vector<std::shared_ptr<ModelContainer>> ModelFactoryTriangle2D::createModelsFromFiles(const std::vector<std::string> &filePaths) {
    std::vector<std::shared_ptr<ModelContainer>> models;
    std::shared_ptr<ModelContainer> model;
    for (const auto& filePath : filePaths) {
        model = createModelFromFile(filePath);
        if (model) {
            models.push_back(model);
        } else {
            return std::vector<std::shared_ptr<ModelContainer>>();
        }
    }
    return models;
}

/*!
*  \brief      Creates a triangle model from the passed list of triangles.
*  \author     Sascha Kaden
*  \param[in]  list of triangles
*  \param[out] triangle model
*  \date       2017-02-19
*/
std::shared_ptr<ModelContainer> ModelFactoryTriangle2D::createModel(const std::vector<Triangle2D>& triangles) {
    std::shared_ptr<ModelTriangle2D> triangleModel(new ModelTriangle2D());
    triangleModel->m_triangles = triangles;
    triangleModel->m_mesh = cad::generateMesh(triangles);
    return triangleModel;
}

} /* namespace ippp */
