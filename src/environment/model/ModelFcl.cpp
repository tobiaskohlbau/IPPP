//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#include <ippp/environment/model/ModelFcl.h>

#include <ippp/environment/cad/CadProcessing.h>

namespace ippp {

ModelFcl::ModelFcl() : ModelContainer("ModelFcl"), m_fclModel(std::make_shared<FCLModel>()){
}

/*!
*  \brief      Return true if model is empty
*  \author     Sascha Kaden
*  \param[out] state
*  \date       2017-02-19
*/
bool ModelFcl::empty() const {
    return (m_fclModel == nullptr || m_fclModel->num_vertices == 0);
}

/*!
*  \brief      Transform the internal fcl model with the passed transformation matrix.
*  \author     Sascha Kaden
*  \param[in]  transformation matrix
*  \date       2017-04-04
*/
void ModelFcl::transformModel(const Transform &T) {
    if (empty())
        return;

    cad::transformVertices(T, m_mesh.vertices);
    m_mesh.aabb = cad::computeAABB(m_mesh);

    updateFclModel();
}

/*!
*  \brief      Transform the internal fcl model with the passed configuration.
*  \author     Sascha Kaden
*  \param[in]  config
*  \date       2017-04-04
*/
void ModelFcl::transformModel(const Vector6 &config) {
    if (config[0] == 0 && config[1] == 0 && config[2] == 0 && config[3] == 0 && config[4] == 0 && config[5] == 0)
        return;
    if (empty())
        return;

    cad::transformVertices(config, m_mesh.vertices);
    m_mesh.aabb = cad::computeAABB(m_mesh);

    updateFclModel();
}

void ModelFcl::updateFclModel() {
    std::vector<fcl::Vec3f> vertices;
    std::vector<fcl::Triangle> triangles;
    for (auto vertex : m_mesh.vertices)
        vertices.emplace_back(vertex[0], vertex[1], vertex[2]);

    for (auto face : m_mesh.faces)
        triangles.emplace_back(face[0], face[1], face[2]);

    m_fclModel = std::make_shared<FCLModel>();
    m_fclModel->beginModel();
    m_fclModel->addSubModel(vertices, triangles);
    m_fclModel->endModel();
}

} /* namespace ippp */
