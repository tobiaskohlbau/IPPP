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

#include <ippp/environment/EnvObject.h>
#include <ippp/util/Logging.h>
#include <ippp/util/UtilGeo.hpp>
#include <ippp/util/UtilVec.hpp>

namespace ippp {

/*!
*  \brief      Standard destructor of the EnvObject
*  \author     Sascha Kaden
*  \date       2018-01-10
*/
EnvObject::~EnvObject() = default;

EnvObject::EnvObject(const std::string &name, EnvObjectType type)
    : Identifier(name), m_type(type), m_pose(Transform::Identity()) {
}

EnvObject::EnvObject(const std::string &name, EnvObjectType type, const Transform &pose)
    : Identifier(name), m_type(type), m_pose(pose) {
}

/*!
*  \brief      Set pose of EnvObject
*  \author     Sascha Kaden
*  \param[in]  Transform pose
*  \date       2017-11-01
*/
void EnvObject::setPose(const Vector6 &pose) {
    if (util::empty<6>(pose)) {
        Logging::warning("Set empty pose", this);
        m_pose = Transform::Identity();
        return;
    }

    m_pose = util::toTransform(pose);
}

/*!
*  \brief      Set pose of EnvObject
*  \author     Sascha Kaden
*  \param[in]  Transform pose
*  \date       2017-11-01
*/
void EnvObject::setPose(const Transform &pose) {
    m_pose = pose;
}

/*!
*  \brief      Get pose of EnvObject
*  \author     Sascha Kaden
*  \param[out] Transform pose
*  \date       2017-11-01
*/
Transform EnvObject::getPose() const {
    return m_pose;
}

} /* namespace ippp */