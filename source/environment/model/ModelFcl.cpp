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

#include <environment/model/ModelFcl.h>

#include <environment/CadProcessing.h>

namespace ippp {

ModelFcl::ModelFcl() : ModelContainer("ModelFcl") {
}

/*!
*  \brief      Return true if model is empty
*  \author     Sascha Kaden
*  \param[out] state
*  \date       2017-02-19
*/
bool ModelFcl::empty() const {
    if (m_fclModel.num_vertices == 0) {
        return true;
    } else {
        return false;
    }
}

/*!
*  \brief      Transform the internal fcl model with the passed transformation matrix.
*  \author     Sascha Kaden
*  \param[in]  transformation matrix
*  \date       2017-04-04
*/
void ModelFcl::transformModel(const Matrix4 &T) {
    if (empty()) {
        return;
    }
    transformCad(config, m_vertices);
    m_boundingBox = computeBoundingBox(m_vertices);

    updateFclModel();
}

/*!
*  \brief      Transform the internal fcl model with the passed configuration.
*  \author     Sascha Kaden
*  \param[in]  config
*  \date       2017-04-04
*/
void ModelFcl::transformModel(const Vector6 &config) {
    if (config[0] == 0 && config[1] == 0 && config[2] == 0 && config[3] == 0 && config[4] == 0 && config[5] == 0) {
        return;
    }
    if (empty()) {
        return;
    }
    transformVertices(config, m_vertices);
    m_boundingBox = computeBoundingBox(m_vertices);

    updateFclModel();
}

void ModelFcl::updateFclModel() {
    std::vector<fcl::Vector3> vertices;
    std::vector<fcl::Triangle> triangles;
    for (auto vertex : m_vertices) {
        vertices.push_back(fcl::Vector3f(vertex[0], vertex[1], vertex[2]));
    }
    for (auto face : m_faces) {
        triangles.push_back(fcl::Triangle(face[0], face[1], face[2]));
    }
    m_fclModel = FCLModel();
    m_fclModel.beginModel();
    m_fclModel.addSubModel(vertices, triangles);
    m_fclModel.endModel();
}

} /* namespace ippp */
