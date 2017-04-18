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

#include <core/utility/Logging.h>
#include <robot/model/ModelContainer.h>

namespace rmpl {

/*!
*  \brief      Standard destructor of the ModelContainer
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
ModelContainer::~ModelContainer() {
}

/*!
*  \brief      Standard constructor of ModelContainer
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
ModelContainer::ModelContainer(const std::string &name, const AABB &boundingBox)
    : Identifier(name) {
}

} /* namespace rmpl */
