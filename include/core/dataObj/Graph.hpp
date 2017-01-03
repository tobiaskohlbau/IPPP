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

#include <boost/thread/shared_mutex.hpp>

#include <core/dataObj/Node.hpp>
#include <core/module/ModuleBase.h>
#include <core/utility/KDTree.hpp>
#include <core/utility/Logging.h>

namespace rmpl {

/*!
* \brief   Class Graph contain all nodes of the planner and offers the nearest neighbor and range search through a KDTree.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class Graph : public ModuleBase {
  public:
    Graph();
    void addNode(const std::shared_ptr<Node<dim>> &node);
    std::vector<std::shared_ptr<Node<dim>>> getNodes();

    std::shared_ptr<Node<dim>> getNearestNode(const Node<dim> &node);
    std::shared_ptr<Node<dim>> getNearestNode(const std::shared_ptr<Node<dim>> &node);
    std::vector<std::shared_ptr<Node<dim>>> getNearNodes(const Node<dim> &node, float distance);
    std::vector<std::shared_ptr<Node<dim>>> getNearNodes(const std::shared_ptr<Node<dim>> node, float distance);

    unsigned int size();

  private:
    std::vector<std::shared_ptr<Node<dim>>> m_nodes;
    std::shared_ptr<KDTree<dim, std::shared_ptr<Node<dim>>>> m_kdTree;
    boost::shared_mutex m_mutex;
};

/*!
*  \brief      Default constructor of the class Graph
*  \author     Sascha Kaden
*  \date       2016-06-02
*/
template <unsigned int dim>
Graph<dim>::Graph() : ModuleBase("Graph") {
    m_kdTree = std::shared_ptr<KDTree<dim, std::shared_ptr<Node<dim>>>>(new KDTree<dim, std::shared_ptr<Node<dim>>>());
}

/*!
* \brief      Add a Node to the graph
* \author     Sascha Kaden
* \param[in]  Node
* \date       2016-05-25
*/
template <unsigned int dim>
void Graph<dim>::addNode(const std::shared_ptr<Node<dim>> &node) {
    m_kdTree->addNode(node->getValues(), node);
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    m_nodes.push_back(node);
    if (m_nodes.size() % 4000 == 0) {
        m_kdTree = std::shared_ptr<KDTree<dim, std::shared_ptr<Node<dim>>>>(new KDTree<dim, std::shared_ptr<Node<dim>>>(m_nodes));
        Logging::info("KD Tree has been sorted and have: " + std::to_string(m_nodes.size()) + " Nodes", this);
    }
}

/*!
* \brief      Returns the list of all nodes from the Graph
* \author     Sascha Kaden
* \param[out] vector of nodes
* \date       2016-05-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNodes() {
    return m_nodes;
}

/*!
* \brief      Search for nearrest neighbor
* \author     Sascha Kaden
* \param[in]  Node from where the search starts
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const Node<dim> &node) {
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return m_kdTree->searchNearestNeighbor(node.getValues());
}

/*!
* \brief      Search for nearrest neighbor
* \author     Sascha Kaden
* \param[in]  Node from where the search starts
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const std::shared_ptr<Node<dim>> &node) {
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return m_kdTree->searchNearestNeighbor(node->getValues());
}

/*!
* \brief      Search range
* \author     Sascha Kaden
* \param[in]  Node for the search
* \param[in]  range around the passed Node
* \param[out] list of nodes inside the range
* \date       2016-05-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const Node<dim> &node, float range) {
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return m_kdTree->searchRange(node.getValues(), range);
}

/*!
* \brief      Search range
* \author     Sascha Kaden
* \param[in]  Node for the search
* \param[in]  range around the passed Node
* \param[out] list of nodes inside the range
* \date       2016-05-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const std::shared_ptr<Node<dim>> node, float range) {
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return m_kdTree->searchRange(node->getValues(), range);
}

/*!
* \brief      Return size of the graph
* \author     Sascha Kaden
* \param[out] size of Node vector
* \date       2016-08-09
*/
template <unsigned int dim>
unsigned int Graph<dim>::size() {
    return m_nodes.size();
}

} /* namespace rmpl */

#endif /* GRAPH_H_ */
