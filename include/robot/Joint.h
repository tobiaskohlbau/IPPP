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

#ifndef JOINT_H_
#define JOINT_H_

#include <memory>

#include <robot/model/ModelContainer.h>

namespace rmpl {

/*!
* \brief   Joint class contains the boundaries of a joint and the MeshContainer from it
* \author  Sascha Kaden
* \date    2016-08-25
*/
class Joint {
  public:
    Joint();
    Joint(float minBound, float maxBound);
    Joint(float minBound, float maxBound, std::shared_ptr<ModelContainer> model);

    void setModel(std::shared_ptr<ModelContainer> &model);
    std::shared_ptr<ModelContainer> getModel();

    void setBoundaries(float minBound, float maxBound);
    void getBoundaries(float &minBound, float &maxBound);

  private:
    float m_minBound = 0;
    float m_maxBound = 0;
    std::shared_ptr<ModelContainer> m_model = nullptr;
};

} /* namespace rmpl */

#endif    // JOINT_H_
