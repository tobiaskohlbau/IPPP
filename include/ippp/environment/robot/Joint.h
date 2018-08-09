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

#ifndef JOINT_H
#define JOINT_H

#include <memory>

#include <ippp/dataObj/DhParameter.h>
#include <ippp/environment/model/ModelContainer.h>

namespace ippp {

/*!
* \brief   Joint class contains the boundaries of a joint and the MeshContainer from it
* \author  Sascha Kaden
* \date    2016-08-25
*/
class Joint {
  public:
    Joint();
    Joint(double minBound, double maxBound, const DhParameter &params, std::shared_ptr<ModelContainer> &linkModel,
          const Transform &linkOffset = Transform::Identity());

    void setLinkModel(std::shared_ptr<ModelContainer> &model);
    std::shared_ptr<ModelContainer> getLinkModel() const;

    void setLinkOffset(const Vector6 &offset);
    void setLinkOffset(const Transform &offset);
    Transform getLinkOffset() const;

    void setDhParameter(const DhParameter &params);
    DhParameter getDhParameter() const;

    void setBoundaries(double minBound, double maxBound);
    std::pair<double, double> getBoundaries() const;

  private:
    DhParameter m_dhParameter;                             /*!< DH-parameter of the single joint */
    double m_minBound = 0;                                 /*!< minimal joint boundary */
    double m_maxBound = 0;                                 /*!< maximal joint boundary */
    std::shared_ptr<ModelContainer> m_linkModel = nullptr; /*!< model container of the link related to the joint */
    Transform m_linkOffset = Transform::Identity();        /*!< optional offset to the link model */
};

} /* namespace ippp */

#endif    // JOINT_H
