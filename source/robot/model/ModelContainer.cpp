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
*  \brief      Standard deconstructor of the ModelContainer
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
ModelContainer::ModelContainer(const std::string &name, const Vector3 &minBoundary, const Vector3 &maxBoundary)
    : Identifier(name) {
}

/*!
*  \brief      Standard constructor of ModelContainer
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
void ModelContainer::setBoundingBox(const Vector3 &minBoundary, const Vector3 &maxBoundary) {
    bool equivalentBoundaries = true;
    for (unsigned int i = 0; i < 3; ++i) {
        if (minBoundary[i] != maxBoundary[i]) {
            equivalentBoundaries = false;
            break;
        }
    }
    if (equivalentBoundaries) {
        Logging::error("Boundaries are equivalent", this);
        return;
    }

    m_minBounding = minBoundary;
    m_maxBounding = maxBoundary;
}

} /* namespace rmpl */
