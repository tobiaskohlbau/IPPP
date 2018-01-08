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

#include <ippp/environment/EnvObject.h>

namespace ippp {

/*!
*  \brief      Standard destructor of the EnvObject
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
EnvObject::~EnvObject() = default;

/*!
*  \brief      Standard constructor of EnvObject
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
EnvObject::EnvObject(const std::string &name, EnvObjectType type) : Identifier(name), m_type(type) {
}

} /* namespace ippp */