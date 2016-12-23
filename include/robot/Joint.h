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

#include <robot/MeshContainer.h>

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
    Joint(float minBound, float maxBound, std::shared_ptr<MeshContainer> &mesh);

    void setMesh(std::shared_ptr<MeshContainer> &mesh);
    std::shared_ptr<MeshContainer> getMesh();

    void setBoundaries(float minBound, float maxBound);
    void getBoundaries(float &minBound, float &maxBound);

  private:
    float m_minBound;
    float m_maxBound;
    std::shared_ptr<MeshContainer> m_mesh;
};

} /* namespace rmpl */

#endif    // JOINT_H_
