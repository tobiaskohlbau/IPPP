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
*  \brief      Standard constructor of the class Base
*  \author     Sasch Kaden
*  \date       2016-05-30
*/
ModuleBase::ModuleBase() {
    m_name = "Unknown";
}

/*!
*  \brief      Constructor of the class Base
*  \author     Sasch Kaden
*  \param[in]  name
*  \date       2016-05-30
*/
ModuleBase::ModuleBase(const std::string &name) {
    m_name = name;
}

/*!
*  \brief      Return name
*  \author     Sasch Kaden
*  \param[out] name
*  \date       2016-05-30
*/
std::string ModuleBase::getName() {
    return m_name;
}
