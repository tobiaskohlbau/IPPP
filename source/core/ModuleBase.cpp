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

#include <core/ModuleBase.h>

using namespace rmpl;

/*!
*  \brief      Standard deconstructor of the ModuleBase
*  \author     Sasch Kaden
*  \date       2016-05-30
*/
ModuleBase::~ModuleBase() {
}

/*!
*  \brief      Constructor of the ModuleBase
*  \author     Sasch Kaden
*  \param[in]  module name
*  \date       2016-05-30
*/
ModuleBase::ModuleBase(const std::string &name)
    : m_name(name)
{
}

/*!
*  \brief      Return name
*  \author     Sasch Kaden
*  \param[out] name
*  \date       2016-05-30
*/
const std::string& ModuleBase::getName() {
    return m_name;
}
