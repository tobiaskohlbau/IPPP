//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
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

#ifndef MODELFCL_H
#define MODELFCL_H

#include <fcl/BV/BV.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>

#include <ippp/environment/model/ModelContainer.h>

namespace ippp {

typedef fcl::BVHModel<fcl::OBB> FCLModel;

/*!
* \brief   Fcl model class
* \author  Sascha Kaden
* \date    2017-02-19
*/
class ModelFcl : public ModelContainer {
  public:
    ModelFcl();
    bool empty() const;
    void transformModel(const Transform &T);
    void transformModel(const Vector6 &config);

    FCLModel m_fclModel; /*!< fcl model for the collision check with the fcl library */

  private:
    void updateFclModel();
};

} /* namespace ippp */

#endif    // MODELFCL_H
