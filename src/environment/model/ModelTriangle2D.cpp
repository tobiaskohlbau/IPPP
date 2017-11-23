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

#include <ippp/environment/cad/CadProcessing.h>
#include <ippp/environment/model/ModelTriangle2D.h>

namespace ippp {

ModelTriangle2D::ModelTriangle2D() : ModelContainer("ModelTriangle2D") {
}

/*!
*  \brief      Return true if model is empty
*  \author     Sascha Kaden
*  \param[out] state
*  \date       2017-02-19
*/
bool ModelTriangle2D::empty() const {
    return m_triangles.empty();
}

void ModelTriangle2D::transformModel(const Transform &T) {
    Matrix2 R = T.rotation().block<2, 2>(0, 0);
    Vector2 t = T.translation().block<2, 1>(0, 0);
    for (auto &triangle : m_triangles)
        triangle.transform(R, t);

    m_mesh = cad::generateMesh(m_triangles);
    m_mesh.aabb = cad::computeAABB(m_mesh);
}

void ModelTriangle2D::transformModel(const Vector6 &config) {
    Matrix2 R = util::getRotMat2D(config[2]);
    Vector2 t(config[0], config[1]);
    for (auto &triangle : m_triangles)
        triangle.transform(R, t);

    m_mesh = cad::generateMesh(m_triangles);
    m_mesh.aabb = cad::computeAABB(m_mesh);
}

} /* namespace ippp */
