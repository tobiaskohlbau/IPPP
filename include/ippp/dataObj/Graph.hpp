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

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <ippp/Identifier.h>
#include <ippp/dataObj/Node.hpp>
#include <ippp/modules/neighborFinders/KDTree.hpp>
#include <ippp/statistic/Stats.h>
#include <ippp/statistic/StatsGraphCollector.h>
#include <ippp/util/Logging.h>

namespace ippp {

/*!
* \brief   Class Graph contain all nodes of the planner and offers the nearest neighbor and range search through a KDTree.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class Graph : public Identifier {
  public:
    Graph(size_t sortCount, std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> neighborFinder);
    ~Graph();

    bool addNode(const std::shared_ptr<Node<dim>> &node);
    void addNodeList(const std::vector<std::shared_ptr<Node<dim>>> &nodes);
    bool containNode(const std::shared_ptr<Node<dim>> &node);
    double getMaxNodeCost() const;

    std::shared_ptr<Node<dim>> getNode(size_t index) const;
    std::shared_ptr<Node<dim>> getNode(const Vector<dim> &config) const;
    std::vector<std::shared_ptr<Node<dim>>> getNodes() const;
    std::vector<std::shared_ptr<Node<dim>>> getNodes(size_t startIndex, size_t endIndex = 0) const;

    std::shared_ptr<Node<dim>> getNearestNode(const Vector<dim> &config) const;
    std::shared_ptr<Node<dim>> getNearestNode(const Node<dim> &node) const;
    std::vector<std::shared_ptr<Node<dim>>> getNearNodes(const Vector<dim> &config, double range) const;
    std::vector<std::shared_ptr<Node<dim>>> getNearNodes(const Node<dim> &node, double range) const;

    void sortTree();
    void clearQueryParents();

    bool empty() const;
    size_t numNodes() const;
    size_t numEdges() const;
    size_t getSortCount() const;
    bool autoSort() const;
    void preserveNodePtr();
    void updateStats() const;

    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> getNeighborFinder();

    bool operator<(std::shared_ptr<Graph<dim>> const &a) {
        if (a->getNode(0) && this->getNode(0))
            return a->getNode(0)->getValues().norm() < this->getNode(0)->getValues().norm();
        else
            return false;
    }

  private:
    void clearPointer();

    std::vector<std::shared_ptr<Node<dim>>> m_nodes;                                             /*!< vector of all graph nodes */
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> m_neighborFinder = nullptr; /*!< search module */
    std::shared_ptr<StatsGraphCollector> m_collector = nullptr;
    std::mutex m_mutex; /*!< mutex for adding and changing of the vector of nodes */

    const size_t m_sortCount = 0;   /*!< divider, when the NeighborFinder has to be sorted */
    bool m_autoSort = false;        /*!< Flag to set the auto sorting */
    bool m_preserveNodePtr = false; /*!< Flag to save all pointers inside of the nodes */
    double m_maxNodeCost = 0;
};

/*!
*  \brief      Default constructor of the class Graph
*  \author     Sascha Kaden
*  \param[in]  count when the graph has to be sorted
*  \param[in]  NeighborFinder
*  \date       2016-06-02
*/
template <unsigned int dim>
Graph<dim>::Graph(size_t sortCount, std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> neighborFinder)
    : Identifier("Graph"),
      m_sortCount(sortCount),
      m_neighborFinder(neighborFinder),
      m_collector(std::make_shared<StatsGraphCollector>("GraphStats")) {
    Logging::debug("Initialize", this);
    Stats::addCollector(m_collector);

    m_autoSort = (sortCount != 0);
    // reserve memory for the node vector to reduce computation time at new memory allocation
    m_nodes.reserve(1000);
}

/*!
*  \brief      Destructor of the class Graph, if preserve Node ptr is set false, all ptr inside the nodes will be removed.
*  \author     Sascha Kaden
*  \date       2017-01-07
*/
template <unsigned int dim>
Graph<dim>::~Graph() {
    if (!m_preserveNodePtr) {
        for (auto &&node : m_nodes) {
            if (node) {
                node->clearPointer();
                node = nullptr;
            }
        }
    }
}

/*!
* \brief      Add a Node to the graph
* \author     Sascha Kaden
* \param[in]  Node
* \param[out] result, false if the Node is soon inside of the Graph
* \date       2016-05-25
*/
template <unsigned int dim>
bool Graph<dim>::addNode(const std::shared_ptr<Node<dim>> &node) {
    m_mutex.lock();
    m_neighborFinder->addNode(node->getValues(), node);
    if (node->getCost() > m_maxNodeCost)
        m_maxNodeCost = node->getCost();
    m_nodes.push_back(node);
    m_mutex.unlock();
    if (m_autoSort && (m_nodes.size() % m_sortCount) == 0)
        sortTree();

    return true;
}

/*!
* \brief      Add Node list to the graph
* \author     Sascha Kaden
* \param[in]  Node list
* \date       2017-04-03
*/
template <unsigned int dim>
void Graph<dim>::addNodeList(const std::vector<std::shared_ptr<Node<dim>>> &nodes) {
    for (auto node : nodes)
        addNode(node);
}

/*!
* \brief      Checks the graph, if it contains the passed Node.
* \author     Sascha Kaden
* \param[in]  Node
* \param[out] result, true if it contains
* \date       2017-10-01
*/
template <unsigned int dim>
bool Graph<dim>::containNode(const std::shared_ptr<Node<dim>> &node) {
    for (const auto &graphNode : m_nodes)
        if (node == graphNode)
            return true;
    return false;
}

/*!
* \brief      Returns the maximum node cost of all nodes
* \author     Sascha Kaden
* \param[out] maximum node cost
* \date       2017-10-01
*/
template <unsigned int dim>
double Graph<dim>::getMaxNodeCost() const {
    return m_maxNodeCost;
}

/*!
* \brief      Return Node from index
* \author     Sascha Kaden
* \param[in]  index
* \param[out] Node
* \date       2017-04-03
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNode(size_t index) const {
    if (index < m_nodes.size())
        return m_nodes[index];
    return nullptr;
}

/*!
* \brief      Return Node from config
* \author     Sascha Kaden
* \param[in]  Vector
* \param[out] Node
* \date       2017-10-01
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNode(const Vector<dim> &config) const {
    // todo: at the time brute force search over all nodes, add a more clever search
    for (const auto &node : m_nodes)
        if (node->getValues().isApprox(config, IPPP_EPSILON))
            return node;
    return nullptr;
}

/*!
* \brief      Returns the list of all nodes from the Graph
* \author     Sascha Kaden
* \param[out] vector of nodes
* \date       2016-05-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNodes() const {
    return m_nodes;
}

/*!
* \brief      Returns the list of all nodes from the Graph
* \author     Sascha Kaden
* \param[out] vector of nodes
* \date       2016-05-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNodes(size_t startIndex, size_t endIndex) const {
    if (startIndex >= m_nodes.size())
        return std::vector<std::shared_ptr<Node<dim>>>();

    if (endIndex == 0)
        return std::vector<std::shared_ptr<Node<dim>>>(m_nodes.begin() + startIndex, m_nodes.end());
    else if (startIndex < m_nodes.size())
        return std::vector<std::shared_ptr<Node<dim>>>(m_nodes.begin() + startIndex, m_nodes.begin() + endIndex);

    return std::vector<std::shared_ptr<Node<dim>>>();
}

/*!
* \brief      Search for nearest neighbor
* \author     Sascha Kaden
* \param[in]  Vector from where the search starts
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const Vector<dim> &config) const {
    return m_neighborFinder->searchNearestNeighbor(config);
}

/*!
* \brief      Search for nearest neighbor
* \author     Sascha Kaden
* \param[in]  Node from where the search starts
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const Node<dim> &node) const {
    return m_neighborFinder->searchNearestNeighbor(node.getValues());
}

/*!
* \brief      Search range
* \author     Sascha Kaden
* \param[in]  Vector for the search
* \param[in]  range around the passed Vector
* \param[out] list of nodes inside the range
* \date       2016-05-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const Vector<dim> &config, double range) const {
    return m_neighborFinder->searchRange(config, range);
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
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const Node<dim> &node, double range) const {
    return m_neighborFinder->searchRange(node.getValues(), range);
}

/*!
* \brief      Rebase the NeighborFinder with the node list from the graph.
* \author     Sascha Kaden
* \date       2017-01-09
*/
template <unsigned int dim>
void Graph<dim>::sortTree() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_neighborFinder->rebaseSorted(m_nodes);
    Logging::debug("Graph has been sorted and has: " + std::to_string(m_nodes.size()) + " Nodes", this);
}

/*!
* \brief      Clear all query parents of the nodes.
* \author     Sascha Kaden
* \date       2017-01-09
*/
template <unsigned int dim>
void Graph<dim>::clearQueryParents() {
    for (auto &&node : m_nodes)
        node->clearQueryParent();
}

/*!
* \brief      Return true if Graph is empty
* \author     Sascha Kaden
* \param[out] state of Graph
* \date       2017-04-03
*/
template <unsigned int dim>
bool Graph<dim>::empty() const {
    if (m_nodes.empty())
        return true;
    return false;
}

/*!
* \brief      Return size of the nodes of the graph
* \author     Sascha Kaden
* \param[out] size of Node vector
* \date       2016-08-09
*/
template <unsigned int dim>
size_t Graph<dim>::numNodes() const {
    return m_nodes.size();
}

/*!
* \brief      Return size of the edges of the graph
* \author     Sascha Kaden
* \param[out] size of edges
* \date       2016-08-09
*/
template <unsigned int dim>
size_t Graph<dim>::numEdges() const {
    size_t edgeSize = 0;
    for (auto &node : m_nodes) {
        edgeSize += node->getChildSize();
        if (node->getParentEdge().first)
            ++edgeSize;
    }
    return edgeSize;
}

/*!
* \brief      Return sort count, if 0 auto sort is false
* \author     Sascha Kaden
* \param[out] count
* \date       2017-01-09
*/
template <unsigned int dim>
size_t Graph<dim>::getSortCount() const {
    return m_sortCount;
}

/*!
* \brief      Return auto sort of Graph
* \author     Sascha Kaden
* \param[out] auto sort
* \date       2017-01-09
*/
template <unsigned int dim>
bool Graph<dim>::autoSort() const {
    return m_autoSort;
}

/*!
* \brief      Clear all parent pointer from the nodes
* \author     Sascha Kaden
* \date       2017-04-03
*/
template <unsigned int dim>
void Graph<dim>::clearPointer() {
    for (auto &&node : m_nodes)
        node->clearPointer();
}

/*!
* \brief      Set node pointer preserve flag, to prevent clearing from node pointer at destruction of the Graph
* \author     Sascha Kaden
* \date       2017-04-03
*/
template <unsigned int dim>
void Graph<dim>::preserveNodePtr() {
    m_preserveNodePtr = true;
}

template <unsigned int dim>
void Graph<dim>::updateStats() const {
    m_collector->setNodeCount(m_nodes.size());
    m_collector->setEdgeCount(numEdges());
}

/*!
* \brief      Return the NeighborFinder instance of the Graph
* \author     Sascha Kaden
* \param[out] NeighborFinder
* \date       2017-04-03
*/
template <unsigned int dim>
std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> Graph<dim>::getNeighborFinder() {
    return m_neighborFinder;
}

} /* namespace ippp */

#endif /* GRAPH_HPP */
