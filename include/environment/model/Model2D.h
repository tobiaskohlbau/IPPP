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

#ifndef MODEL2D_H
#define MODEL2D_H

#include <core/dataObj/PointList.hpp>
#include <environment/model/ModelContainer.h>

namespace ippp {

/*!
* \brief   2D Model for 2D PointList
* \author  Sascha Kaden
* \date    2017-02-19
*/
class Model2D : public ModelContainer {
  public:
    Model2D();
    Model2D(const std::vector<Triangle2D> triangles);
    bool empty() const;
    void transformModel(const Matrix4 &T);
    void transformModel(const Vector6 &config);

    std::vector<Triangle2D> m_triangles;
};

} /* namespace ippp */

#endif    // MODEL2D_H
