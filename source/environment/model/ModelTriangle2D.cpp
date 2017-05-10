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

#include <environment/model/ModelTriangle2D.h>

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
    if (m_triangles.size() == 0) {
        return true;
    } else {
        return false;
    }
}

void ModelTriangle2D::transformModel(const Matrix4 &T) {
    Matrix2 R;
    Vector2 t;
    Matrix3 transformation = T.block<3, 3>(0, 0);
    util::decomposeT(transformation, R, t);
    for (auto triangle : m_triangles) {
        triangle.transform(R, t);
    }
}

void ModelTriangle2D::transformModel(const Vector6 &config) {
    Matrix2 R = util::getRotMat2D(config[2]);
    Vector2 t(config[0], config[1]);
    for (auto triangle : m_triangles) {
        triangle.transform(R, t);
    }
}

} /* namespace ippp */
