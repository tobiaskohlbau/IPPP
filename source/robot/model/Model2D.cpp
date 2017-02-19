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

#include <robot/model/Model2D.h>

namespace rmpl {

Model2D::Model2D() {

}

Model2D::Model2D(Eigen::MatrixXi space) {
    if (space.cols() == -1) {
        Logging::error("Empty space", "Model2D");
        return;
    }
    m_space = space;
}

bool Model2D::empty() const {
    if (m_space.cols() == -1)
        return true;
    else
        return false;
}

} /* namespace rmpl */

