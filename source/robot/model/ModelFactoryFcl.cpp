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

#include <robot/model/ModelFactoryFcl.h>

#include <core/utility/Logging.h>

namespace rmpl {

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
    std::vector<Vector3> vertices;
    std::vector<Vector3i> faces;


    std::shared_ptr<ModelFcl> fclModel(new ModelFcl());
    if (!importCad(filePath, fclModel->m_vertices, fclModel->m_faces, fclModel->m_normals)) {
        Logging::error("Could not load mesh", this);
        return nullptr;
    }
    fclModel->m_boundingBox = computeBoundingBox(fclModel->m_vertices);

    std::vector<fcl::Vector3f> vertices;
    std::vector<fcl::Triangle> triangles;
    for (auto vertex : pqpModel->m_vertices)
        vertices.push_back(fcl::Vector3f(vertex[0], vertex[1], vertex[2]));
    for (auto face : fclModel->m_faces)
        triangles.push_back(fcl::Triangle(face[0], face[1], face[2]));
    fclModel->m_fclModel.beginModel();
    fclModel->m_fclModel.addSubModel(vertices, triangles);
    fclModel->m_fclModel.endModel();

    return fclModel;
}

std::vector<std::shared_ptr<ModelContainer>> ModelFactoryFcl::createModels(const std::string &filePath) {

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

} /* namespace rmpl */
