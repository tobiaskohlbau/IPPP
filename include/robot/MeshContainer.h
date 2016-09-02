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

#ifndef MESHCONTAINER_H_
#define MESHCONTAINER_H_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <PQP.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/fcl.h>

#include <core/ModuleBase.h>
#include <core/Vec.hpp>

namespace rmpl {

/*!
* \brief   Class MeshContainer
* \details Contains all mesh models for the collision detection
* \author  Sascha Kaden
* \date    2016-08-25
*/
class MeshContainer {
  public:
    MeshContainer();
    MeshContainer(std::string filepath);
    MeshContainer(std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> &fclModel, std::shared_ptr<PQP_Model> &pqpModel);
    bool loadFile(const std::string filePath);

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> getFcl();
    std::shared_ptr<PQP_Model> getPqp();

  private:
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> m_fclModel;
    std::shared_ptr<PQP_Model> m_pqpModel;
};

} /* namespace rmpl */

#endif    // MESHCONTAINER_H_
