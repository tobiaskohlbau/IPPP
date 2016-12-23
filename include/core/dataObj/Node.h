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

#ifndef NODE_H_
#define NODE_H_

#include <assert.h>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include <core/dataObj/Edge.h>

namespace rmpl {

/*!
* \brief   Class Node to present nodes of path planner
* \details Consists of the position by an Vec, a cost parameter, pointer to an parent Node and a list of child
* \author  Sascha Kaden
* \date    2016-05-23
*/
class Node {
  public:
    Node();
    Node(float x, float y);
    Node(float x, float y, float z);
    Node(float x, float y, float z, float rx);
    Node(float x, float y, float z, float rx, float ry);
    Node(float x, float y, float z, float rx, float ry, float rz);
    Node(const Eigen::VectorXf &vec);

    float getX();
    float getY();
    float getZ();

    unsigned int getDim();
    bool empty() const;
    float getDist(const std::shared_ptr<Node> &node);
    float getDist(const Node &node);
    float getDistToParent();

    void setCost(float cost);
    void addCost(float cost);
    float getCost();

    void setParent(std::shared_ptr<Node> &parent);
    std::shared_ptr<Node> getParentNode();
    std::shared_ptr<Edge> getParentEdge();
    void clearParent();
    void addChild(std::shared_ptr<Node> &child);
    std::vector<std::shared_ptr<Node>> getChildNodes();
    std::vector<std::shared_ptr<Edge>> getChildEdges();
    void clearChildes();

    Eigen::VectorXf getValues() const;

  private:
    Eigen::VectorXf m_vec;
    float m_cost = -1;

    std::shared_ptr<Edge> m_parent = nullptr;
    std::vector<std::shared_ptr<Edge>> m_childes;
};

} /* namespace rmpl */

#endif /* NODE_H_ */
