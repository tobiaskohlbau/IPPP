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

#include <ippp/core/Identifier.h>

namespace ippp {

/*!
*  \brief      Standard deconstructor of the ModuleBase
*  \author     Sasch Kaden
*  \date       2016-05-30
*/
Identifier::~Identifier() {
}

/*!
*  \brief      Constructor of the ModuleBase
*  \author     Sasch Kaden
*  \param[in]  module name
*  \date       2016-05-30
*/
Identifier::Identifier(const std::string& name) : m_name(name), m_hash(m_hashFn(name)) {
}

/*!
*  \brief      Return name
*  \author     Sasch Kaden
*  \param[out] name
*  \date       2016-05-30
*/
const std::string& Identifier::getName() const {
    return m_name;
}

/*!
*  \brief      Return name
*  \author     Sasch Kaden
*  \param[out] name
*  \date       2016-05-30
*/
const size_t& Identifier::getHash() const {
    return m_hash;
}

} /* namespace ippp */
