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

#ifndef KDNODE_H_
#define KDNODE_H_

#include <Eigen/Core>
#include <cstdint>

namespace rmpl {

/*!
* \brief   Class KDNode is the node object of the KDTree
* \details Owns the position of the node, pointer to left and right leaf, pointer to an extern object, axis split value and the
* split value itself.
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim, typename T>
class KDNode {
  public:
    KDNode(const Eigen::VectorXf &vec, const T &node);

    std::shared_ptr<KDNode<dim, T>> left;
    std::shared_ptr<KDNode<dim, T>> right;
    Eigen::VectorXf vec;
    T node;
    unsigned int axis;
    float value;
};

/*!
*  \brief      Constructor of the class KDNode
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  pointer to an extern object
*  \date       2016-05-27
*/
template <unsigned int dim, typename T>
KDNode<dim, T>::KDNode(const Eigen::VectorXf &vec, const T &node) {
    this->vec = vec;
    this->node = node;
    axis = 0;
    value = 0;
}
} /* namespace rmpl */

#endif /* KDNODE_H_ */
