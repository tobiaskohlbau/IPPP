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

#include <ippp/environment/model/ModelFactoryFcl.h>

#include <ippp/util/Logging.h>
#include <ippp/environment/cad/CadImportExport.h>

namespace ippp {

ModelFactoryFcl::ModelFactoryFcl() : ModelFactory("ModelFactory") {
}

/*!
*  \brief      Creates an ModelFcl from the passed source cad file.
*  \details    The model contains the vertices and faces of the loaded cad model.
*  \author     Sascha Kaden
*  \param[in]  filePath
*  \param[out] smart pointer to ModelFcl
*  \date       2017-02-19
*/
std::shared_ptr<ModelContainer> ModelFactoryFcl::createModel(const std::string &filePath) {
    if (filePath == "") {
        Logging::error("Empty file path", this);
        return nullptr;
    }

    std::shared_ptr<ModelFcl> fclModel(new ModelFcl());
    if (!cad::importMesh(filePath, fclModel->m_mesh)) {
        Logging::error("Could not load mesh", this);
        return nullptr;
    }
    fclModel->m_mesh.aabb = cad::computeAABB(fclModel->m_mesh);

    std::vector<fcl::Vec3f> vertices;
    std::vector<fcl::Triangle> triangles;
    for (auto vertex : fclModel->m_mesh.vertices)
        vertices.emplace_back(vertex[0], vertex[1], vertex[2]);
    for (auto face : fclModel->m_mesh.faces)
        triangles.emplace_back(face[0], face[1], face[2]);
    fclModel->m_fclModel.beginModel();
    fclModel->m_fclModel.addSubModel(vertices, triangles);
    fclModel->m_fclModel.endModel();

    return fclModel;
}

std::vector<std::shared_ptr<ModelContainer>> ModelFactoryFcl::createModels(const std::string &filePath) {
    std::vector<std::shared_ptr<ModelContainer>> models;
    if (filePath == "") {
        Logging::error("Empty file path", this);
        return models;
    }
    // create model container properties
    std::vector<Mesh> meshes;

    if (!cad::importMeshes(filePath, meshes)) {
        Logging::error("Could not load mesh", this);
        return models;
    }

    for (auto &mesh : meshes) {
        std::shared_ptr<ModelFcl> fclModel(new ModelFcl());
        fclModel->m_mesh = mesh;
        fclModel->m_mesh.aabb = cad::computeAABB(fclModel->m_mesh);

        std::vector<fcl::Vec3f> vertices;
        std::vector<fcl::Triangle> triangles;
        for (auto vertex : fclModel->m_mesh.vertices)
            vertices.emplace_back(vertex[0], vertex[1], vertex[2]);
        for (auto face : fclModel->m_mesh.faces)
            triangles.emplace_back(face[0], face[1], face[2]);
        fclModel->m_fclModel.beginModel();
        fclModel->m_fclModel.addSubModel(vertices, triangles);
        fclModel->m_fclModel.endModel();
        models.push_back(fclModel);
    }
    return models;
}

/*!
*  \brief      Creates a list of fcl models from the passed source cad files.
*  \author     Sascha Kaden
*  \param[in]  filePaths
*  \param[out] list of fcl models
*  \date       2017-02-19
*/
std::vector<std::shared_ptr<ModelContainer>> ModelFactoryFcl::createModels(const std::vector<std::string> &filePaths) {
    std::vector<std::shared_ptr<ModelContainer>> models;
    std::shared_ptr<ModelContainer> model;
    for (auto filePath : filePaths) {
        model = createModel(filePath);
        if (model)
            models.push_back(model);
        else
            return std::vector<std::shared_ptr<ModelContainer>>();
    }
    return models;
}

} /* namespace ippp */
