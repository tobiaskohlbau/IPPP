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

#ifndef TRIANGLEMODEL_H
#define TRIANGLEMODEL_H

#include <robot/model/ModelContainer.h>
#include <core/dataObj/PointList.hpp>

namespace rmpl {

class ModelTriangle : public ModelContainer {
public:
    bool empty() const;
    std::vector<Triangle2D> m_triangles;

};

} /* namespace rmpl */

#endif //TRIANGLEMODEL_H
