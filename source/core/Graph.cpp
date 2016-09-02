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

#include <core/Graph.h>

#include <core/Logging.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Default constructor of the class Graph
*  \author     Sascha Kaden
*  \date       2016-06-02
*/
Graph::Graph() : ModuleBase("Graph") {
    m_kdTree = shared_ptr<KDTree<shared_ptr<Node>>>(new KDTree<shared_ptr<Node>>());
}

/*!
* \brief      Add Node to the graph
* \author     Sascha Kaden
* \param[in]  Node
* \date       2016-05-25
*/
void Graph::addNode(const shared_ptr<Node> &node) {
    m_kdTree->addNode(node->getVec(), node);
    m_mutex.lock();
    m_nodes.push_back(node);
    if (m_nodes.size() % 2500 == 0) {
        m_kdTree = shared_ptr<KDTree<shared_ptr<Node>>>(new KDTree<shared_ptr<Node>>(m_nodes));
        Logging::info("KD Tree has been sorted and have: " + std::to_string(m_nodes.size()) + " Nodes", this);
    }
    m_mutex.unlock();
}

/*!
* \brief      Remove Node from the graph
* \author     Sascha Kaden
* \param[in]  index
* \date       2016-05-25
*/
void Graph::removeNode(int index) {
    if (index < m_nodes.size()) {
        m_nodes.erase(m_nodes.begin() + index);
    }
}

/*!
* \brief      Return a the list of nodes
* \author     Sascha Kaden
* \param[out] list of nodes
* \date       2016-05-25
*/
std::vector<shared_ptr<Node>> Graph::getNodes() {
    return m_nodes;
}

/*!
* \brief      Search for nearrest neighbor
* \author     Sascha Kaden
* \param[in]  Node for the search
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
shared_ptr<Node> Graph::getNearestNode(const Node &node) {
    return m_kdTree->searchNearestNeighbor(node.getVec());
}

/*!
* \brief      Search for nearrest neighbor
* \author     Sascha Kaden
* \param[in]  pointer of the Node for the search
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
shared_ptr<Node> Graph::getNearestNode(const shared_ptr<Node> &node) {
    return m_kdTree->searchNearestNeighbor(node->getVec());
}

/*!
* \brief      Search range
* \author     Sascha Kaden
* \param[in]  Node for the search
* \param[in]  distance around the passed Node
* \param[out] list of nodes inside the range
* \date       2016-05-25
*/
std::vector<shared_ptr<Node>> Graph::getNearNodes(const Node &node, float range) {
    return m_kdTree->searchRange(node.getVec(), range);
}

/*!
* \brief      Search range
* \author     Sascha Kaden
* \param[in]  Node for the search
* \param[in]  distance around the passed Node
* \param[out] list of nodes inside the range
* \date       2016-05-25
*/
std::vector<shared_ptr<Node>> Graph::getNearNodes(const shared_ptr<Node> node, float range) {
    return m_kdTree->searchRange(node->getVec(), range);
}

/*!
* \brief      Return size of the graph
* \author     Sascha Kaden
* \param[out] size of Node vector
* \date       2016-08-09
*/
unsigned int Graph::size() {
    return m_nodes.size();
}
