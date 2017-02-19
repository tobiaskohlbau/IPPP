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

#include <robot/Joint.h>

#include <core/utility/Logging.h>

namespace rmpl {

/*!
*  \brief      Standard contructor of the Joint class
*  \author     Sascha Kaden
*  \date       2016-10-25
*/
Joint::Joint() {
}

/*!
*  \brief      Contructor of the Joint class
*  \param[in]  minimum Boundary
*  \param[in]  maximum Boundary
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
Joint::Joint(float minBound, float maxBound) {
    setBoundaries(minBound, maxBound);
}

/*!
*  \brief      Contructor of the Joint class
*  \param[in]  minimum Boundary
*  \param[in]  maximum Boundary
*  \param[in]  MeshContainer
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
Joint::Joint(float minBound, float maxBound, std::shared_ptr<ModelContainer> model) {
    setBoundaries(minBound, maxBound);
    setModel(model);
}

/*!
*  \brief      Set the MeshContainer
*  \param[in]  MeshContainer
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
void Joint::setModel(std::shared_ptr<ModelContainer> &model) {
    if (model->empty()) {
        Logging::error("Emtpy model", "Joint");
        return;
    }
    m_model = model;
}

/*!
*  \brief      Return the MeshContainer
*  \param[out] MeshContainer
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
std::shared_ptr<ModelContainer> Joint::getModel() {
    return m_model;
}

/*!
*  \brief      Set the Boundaries
*  \param[in]  minimum Boundary
*  \param[in]  maximum Boundary
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
void Joint::setBoundaries(float minBound, float maxBound) {
    if (minBound > maxBound) {
        m_maxBound = minBound;
        m_minBound = maxBound;
    } else {
        m_minBound = minBound;
        m_maxBound = maxBound;
    }
}

/*!
*  \brief      Return the Boundaries
*  \param[out] minimum Boundary
*  \param[out] maximum Boundary
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
void Joint::getBoundaries(float &minBound, float &maxBound) {
    minBound = m_minBound;
    maxBound = m_maxBound;
}

} /* namespace rmpl */