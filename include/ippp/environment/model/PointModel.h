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

#ifndef POINTMODEL_H
#define POINTMODEL_H

#include <ippp/dataObj/PointList.hpp>
#include <ippp/environment/model/ModelContainer.h>

namespace ippp {

/*!
* \brief   Triangle2D model class, contains a list of triangles, will be used from the TriangleRobot2D
* \author  Sascha Kaden
* \date    2017-02-19
*/
class PointModel : public ModelContainer {
  public:
    PointModel();
    bool empty() const;
    void transformModel(const Transform &T);
    void transformModel(const Vector6 &config);
};

} /* namespace ippp */

#endif    // POINTMODEL_H
