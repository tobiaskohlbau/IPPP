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

#include <robot/model/ModelPqp.h>

#include <robot/model/CadProcessing.h>

namespace rmpl {

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

void ModelPqp::transform(const Vector6 &config) {
    if (empty()) {
        return;
    }
    m_vertices = transformCad(config, m_vertices);

    m_pqpModel.BeginModel();
    // create pqp triangles
    PQP_REAL p[3][3];
    for (int i = 0; i < m_faces.size(); ++i) {
        // go through faces
        for (int j = 0; j < 3; ++j) {
            // go through face
            int vert = m_faces[i][j];
            for (int k = 0; k < 3; ++k) {
                p[j][k] = m_vertices[vert][k];
            }
        }
        m_pqpModel.AddTri(p[0], p[1], p[2], i);
    }
    m_pqpModel.EndModel();
}

} /* namespace rmpl */
