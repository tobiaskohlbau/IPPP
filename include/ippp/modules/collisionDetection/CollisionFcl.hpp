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

#ifndef COLLISIONFCL_HPP
#define COLLISIONFCL_HPP

#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>

#include <ippp/environment/model/ModelFcl.h>
#include <ippp/environment/robot/SerialRobot.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/statistic/Stats.h>
#include <ippp/statistic/StatsCollisionCollector.h>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   Class for collision detection with the fcl library
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionFcl : public CollisionDetection<dim> {
  public:
    CollisionFcl(const std::string &name, const std::shared_ptr<Environment> &environment,
                 const CollisionRequest &request = CollisionRequest());

  protected:
    bool checkFCL(const std::shared_ptr<FCLModel> &model1, const std::shared_ptr<FCLModel> &model2, const Transform &T1,
                  const Transform &T2) const;

    Transform m_identity;
    AABB m_workspaceBounding;
    std::vector<std::pair<std::shared_ptr<FCLModel>, Transform>> m_obstacles;
    bool m_obstacleExists = false;
    std::shared_ptr<StatsCollisionCollector> m_collisionCollector = nullptr;

    using CollisionDetection<dim>::m_environment;
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionFcl<dim>::CollisionFcl(const std::string &name, const std::shared_ptr<Environment> &environment,
                                const CollisionRequest &request)
    : CollisionDetection<dim>(name, environment, request),
      m_identity(Transform::Identity()),
      m_workspaceBounding(environment->getSpaceBoundary()),
      m_collisionCollector(std::make_shared<StatsCollisionCollector>("CollisionCount")) {
    Stats::addCollector(m_collisionCollector);

    if (environment->numObstacles() > 0) {
        m_obstacleExists = true;
        for (auto &obstacle : environment->getObstacles())
            m_obstacles.push_back(
                std::make_pair(std::static_pointer_cast<ModelFcl>(obstacle->model)->m_fclModel, obstacle->getPose()));
    } else {
        Logging::warning("No obstacles set", this);
    }
}

/*!
*  \brief      Check for collision with FCL library
*  \author     Sascha Kaden
*  \param[in]  FCL mesh model one
*  \param[in]  FCL mesh model two
*  \param[in]  rotation matrix one
*  \param[in]  rotation matrix two
*  \param[in]  translation vector one
*  \param[in]  translation vector two
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionFcl<dim>::checkFCL(const std::shared_ptr<FCLModel> &model1, const std::shared_ptr<FCLModel> &model2,
                                 const Transform &T1, const Transform &T2) const {
    auto &R1 = T1.rotation();
    auto &t1 = T1.translation();
    fcl::Matrix3f fclR1(R1(0, 0), R1(0, 1), R1(0, 2), R1(1, 0), R1(1, 1), R1(1, 2), R1(2, 0), R1(2, 1), R1(2, 2));
    fcl::Vec3f fclT1(t1(0), t1(1), t1(2));

    auto &R2 = T2.rotation();
    auto &t2 = T2.translation();
    fcl::Matrix3f fclR2(R2(0, 0), R2(0, 1), R2(0, 2), R2(1, 0), R2(1, 1), R2(1, 2), R2(2, 0), R2(2, 1), R2(2, 2));
    fcl::Vec3f fclT2(t2(0), t2(1), t2(2));

    fcl::CollisionObject o1(model1, fclR1, fclT1);
    fcl::CollisionObject o2(model2, fclR2, fclT2);
    fcl::CollisionRequest request;    // (1000, true);    // default setting
    fcl::CollisionResult result;
    fcl::collide(&o1, &o2, request, result);

    return result.isCollision();
}

} /* namespace ippp */

#endif /* COLLISIONFCL_HPP */
