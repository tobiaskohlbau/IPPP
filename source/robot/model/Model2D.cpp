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

#include <robot/model/Model2D.h>

namespace ippp {

/*!
*  \brief      Standard constructor of Model2D
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
Model2D::Model2D() : ModelContainer("Model2D") {
}

/*!
*  \brief      Constructor of Model2D
*  \author     Sascha Kaden
*  \param[in]  space
*  \date       2017-02-19
*/
Model2D::Model2D(Eigen::MatrixXi space) : ModelContainer("Model2D") {
    if (space.cols() == -1) {
        Logging::error("Empty space", this);
        return;
    }
    m_space = space;
}

/*!
*  \brief      Return true if model is empty
*  \author     Sascha Kaden
*  \param[out] state
*  \date       2017-02-19
*/
bool Model2D::empty() const {
    if (m_space.cols() == -1) {
        return true;
    } else {
        return false;
    }
}

void Model2D::transformModel(const Matrix4 &T) {
}

void Model2D::transformModel(const Vector6 &config) {
}

} /* namespace ippp */
