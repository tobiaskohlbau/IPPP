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

#include <ippp/environment/model/ModelPqp.h>

#include <include/ippp/environment/cad/CadProcessing.h>

namespace ippp {

/*!
*  \brief   Standard constructor of the ModelPqp
*  \author  Sascha Kaden
*  \date    2017-04-07
*/
ModelPqp::ModelPqp() : ModelContainer("ModelPqp") {
}

/*!
*  \brief      Return true if model is empty
*  \author     Sascha Kaden
*  \param[out] state
*  \date       2017-02-19
*/
bool ModelPqp::empty() const {
    if (m_pqpModel.num_tris == 0) {
        return true;
    } else {
        return false;
    }
}

/*!
*  \brief      Transform the internal pqp model with the passed configuration.
*  \author     Sascha Kaden
*  \param[in]  config
*  \date       2017-04-04
*/
void ModelPqp::transformModel(const Vector6 &config) {
    if (config[0] == 0 && config[1] == 0 && config[2] == 0 && config[3] == 0 && config[4] == 0 && config[5] == 0) {
        return;
    }
    if (empty()) {
        return;
    }
    cad::transformVertices(config, m_mesh.vertices);
    m_mesh.aabb = cad::computeAABB(m_mesh);

    updatePqpModel();
}

/*!
*  \brief      Transform the internal pqp model with the passed transformation matrix.
*  \author     Sascha Kaden
*  \param[in]  transformation matrix
*  \date       2017-04-04
*/
void ModelPqp::transformModel(const Matrix4 &T) {
    if (empty()) {
        return;
    }
    cad::transformVertices(T, m_mesh.vertices);
    m_mesh.aabb = cad::computeAABB(m_mesh);

    updatePqpModel();
}

/*!
*  \brief   Update the internal pqp model with the internal vertices.
*  \author  Sascha Kaden
*  \date    2017-04-04
*/
void ModelPqp::updatePqpModel() {
    m_pqpModel.BeginModel();
    // create pqp triangles
    PQP_REAL p[3][3];
    for (size_t i = 0; i < m_mesh.faces.size(); ++i) {
        // go through faces
        for (size_t j = 0; j < 3; ++j) {
            // go through face
            int vert = m_mesh.faces[i][j];
            for (size_t k = 0; k < 3; ++k) {
                p[j][k] = m_mesh.vertices[vert][k];
            }
        }
        m_pqpModel.AddTri(p[0], p[1], p[2], i);
    }
    m_pqpModel.EndModel();
}

} /* namespace ippp */
