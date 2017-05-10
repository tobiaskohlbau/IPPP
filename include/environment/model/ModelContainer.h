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

#ifndef MODELCONTAINER_H
#define MODELCONTAINER_H

#include <core/module/Identifier.h>
#include <core/utility/UtilVec.hpp>

#include <environment/model/Mesh.h>

namespace ippp {

/*!
* \brief   Base class for all robot models
* \author  Sascha Kaden
* \date    2017-02-19
*/
class ModelContainer : public Identifier {
  public:
    virtual ~ModelContainer();

  protected:
    ModelContainer(const std::string &name, const AABB &boundingBox = AABB());

  public:
    virtual bool empty() const = 0;
    virtual void transformModel(const Matrix4 &T) = 0;
    virtual void transformModel(const Vector6 &config) = 0;

    Mesh m_mesh;
};

} /* namespace ippp */

#endif    // MODELCONTAINER_H
