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

#include <ippp/environment/robot/Joint.h>

#include <ippp/util/Logging.h>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
*  \brief      Standard contructor of the Joint class
*  \author     Sascha Kaden
*  \date       2016-10-25
*/
Joint::Joint() = default;

/*!
*  \brief      Contructor of the Joint class
*  \param[in]  minimum Boundary
*  \param[in]  maximum Boundary
*  \param[in]  MeshContainer
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
Joint::Joint(double minBound, double maxBound, const DhParameter &params, std::shared_ptr<ModelContainer> &linkModel,
             const Transform &linkOffset) {
    setBoundaries(minBound, maxBound);
    setLinkModel(linkModel);
    m_dhParameter = params;
    m_linkOffset = linkOffset;
}

/*!
*  \brief      Set the DH parameter
*  \param[in]  params
*  \author     Sascha Kaden
*  \date       2018-10-11
*/
void Joint::setDhParameter(const DhParameter &params) {
    m_dhParameter = params;
}

/*!
*  \brief      Return the DH parameter
*  \param[out] params
*  \author     Sascha Kaden
*  \date       2018-10-11
*/
DhParameter Joint::getDhParameter() const {
    return m_dhParameter;
}

/*!
*  \brief      Set the link ModelContainer
*  \param[in]  ModelContainer
*  \author     Sascha Kaden
*  \date       2018-10-11
*/
void Joint::setLinkModel(std::shared_ptr<ModelContainer> &model) {
    if (!model || model->empty()) {
        Logging::error("Emtpy model", "Joint");
        return;
    }
    m_linkModel = model;
}

/*!
*  \brief      Return the link ModelContainer
*  \param[out] ModelContainer
*  \author     Sascha Kaden
*  \date       2018-10-11
*/
std::shared_ptr<ModelContainer> Joint::getLinkModel() const {
    return m_linkModel;
}

/*!
*  \brief      Set the link offset
*  \param[in]  link offset
*  \author     Sascha Kaden
*  \date       2018-10-11
*/
void Joint::setLinkOffset(const Vector6 &offset) {
    setLinkOffset(util::toTransform(offset));
}

/*!
*  \brief      Set the link offset
*  \param[in]  link offset
*  \author     Sascha Kaden
*  \date       2018-10-11
*/
void Joint::setLinkOffset(const Transform &offset) {
    m_linkOffset = offset;
}

/*!
*  \brief      Return the link offset
*  \param[out] link offset
*  \author     Sascha Kaden
*  \date       2018-10-11
*/
Transform Joint::getLinkOffset() const {
    return m_linkOffset;
}

/*!
*  \brief      Set the Boundaries
*  \param[in]  minimum Boundary
*  \param[in]  maximum Boundary
*  \author     Sascha Kaden
*  \date       2016-08-25
*/
void Joint::setBoundaries(double minBound, double maxBound) {
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
std::pair<double, double> Joint::getBoundaries() const {
    return std::make_pair(m_minBound, m_maxBound);
}

} /* namespace ippp */
