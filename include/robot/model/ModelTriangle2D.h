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

#ifndef TRIANGLEMODEL2D_H
#define TRIANGLEMODEL2D_H

#include <core/dataObj/PointList.hpp>
#include <robot/model/ModelContainer.h>

namespace rmpl {

/*!
* \brief   Triangle2D model class, contains a list of triangles, will be used from the TriangleRobot2D
* \author  Sascha Kaden
* \date    2017-02-19
*/
class ModelTriangle2D : public ModelContainer {
  public:
    ModelTriangle2D();
    bool empty() const;
    void transformModel(const Matrix4 &T);
    void transformModel(const Vector6 &config);

    std::vector<Triangle2D> m_triangles;
};

} /* namespace rmpl */

#endif    // TRIANGLEMODEL_H
