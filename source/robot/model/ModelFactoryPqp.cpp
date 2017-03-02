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

#include <robot/model/ModelFactoryPqp.h>

#include <core/utility/Logging.h>

namespace rmpl {

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
std::shared_ptr<ModelContainer> ModelFactoryPqp::createModel(const std::string &filePath) {
    if (filePath == "") {
        Logging::error("Empty file path", this);
        return nullptr;
    }
    std::vector<Vector3> vertices;
    std::vector<Vector3i> faces;
    if (!importCad(filePath, vertices, faces)) {
        Logging::error("Could not load mesh", this);
        return nullptr;
    }

    std::shared_ptr<ModelPqp> pqpModel(new ModelPqp());
    pqpModel->m_vertices = vertices;
    pqpModel->m_faces = faces;
    pqpModel->m_pqpModel.BeginModel();
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
        pqpModel->m_pqpModel.AddTri(p[0], p[1], p[2], i);
    }
    pqpModel->m_pqpModel.EndModel();
    if (Logging::getLogLevel() == LogLevel::debug) {
        pqpModel->m_pqpModel.MemUsage(1);
    }
    return pqpModel;
}

/*!
*  \brief      Creates a list of pqp models from the passed source cad files.
*  \author     Sascha Kaden
*  \param[in]  filePaths
*  \param[out] list of pqp models
*  \date       2017-02-19
*/
std::vector<std::shared_ptr<ModelContainer>> ModelFactoryPqp::createModels(const std::vector<std::string> &filePaths) {
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
