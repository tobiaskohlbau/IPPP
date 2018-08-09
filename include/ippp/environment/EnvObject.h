//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#ifndef ENVOBJECT_H
#define ENVOBJECT_H

#include <ippp/Identifier.h>
#include <ippp/types.h>

namespace ippp {

/*!
* \brief   Base class of all objects inside of the Environment. It contains basically the pose of the objects.
* \author  Sascha Kaden
* \date    2018-07-25
*/
class EnvObject : public Identifier {
  public:
    virtual ~EnvObject();

  protected:
    EnvObject(const std::string &name, EnvObjectType type);
    EnvObject(const std::string &name, EnvObjectType type, const Transform &pose);

  public:
    const EnvObjectType m_type;
    virtual bool isStatic() const = 0;

    void setPose(const Vector6 &pose);
    void setPose(const Transform &pose);
    Transform getPose() const;

  protected:
    Transform m_pose; /*!< pose of the EnvObject inside of the world coordinate system */
};

} /* namespace ippp */

#endif /* ENVOBJECT_H */