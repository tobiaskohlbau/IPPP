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

#ifndef GRAPH_H_
#define GRAPH_H_

#include <algorithm>
#include <mutex>

#include <core/Base.h>
#include <core/KDTree.hpp>
#include <core/Node.h>

namespace rmpl {

/*!
* \brief   Class Graph contain all nodes of the planner and offers the nearest neighbor and range search through a KDTree
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Graph : public Base {
  public:
    Graph();
    void addNode(const std::shared_ptr<Node> &node);
    void removeNode(int index);
    std::vector<std::shared_ptr<Node>> getNodes();

    std::shared_ptr<Node> getNearestNode(const Node &node);
    std::shared_ptr<Node> getNearestNode(const std::shared_ptr<Node> &node);
    std::vector<std::shared_ptr<Node>> getNearNodes(const std::shared_ptr<Node> node, float distance);

  private:
    std::vector<std::shared_ptr<Node>> m_nodes;
    KDTree<std::shared_ptr<Node>> m_kdTree;
    std::mutex m_mutexAddNode;
};

} /* namespace rmpl */

#endif /* GRAPH_H_ */
