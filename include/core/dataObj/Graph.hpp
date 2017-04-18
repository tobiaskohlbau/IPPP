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

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <core/dataObj/Node.hpp>
#include <core/module/Identifier.h>
#include <core/utility/KDTree.hpp>
#include <core/utility/Logging.h>

namespace rmpl {

/*!
* \brief   Class Graph contain all nodes of the planner and offers the nearest neighbor and range search through a KDTree.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class Graph : public Identifier {
  public:
    Graph(const unsigned int sortCount);
    ~Graph();

    void addNode(const std::shared_ptr<Node<dim>> &node);
    void addNodeList(const std::vector<std::shared_ptr<Node<dim>>> &nodes);
    std::shared_ptr<Node<dim>> getNode(const unsigned int index) const;
    std::vector<std::shared_ptr<Node<dim>>> getNodes() const;

    std::shared_ptr<Node<dim>> getNearestNode(const Vector<dim> &vec) const;
    std::shared_ptr<Node<dim>> getNearestNode(const Node<dim> &node) const;
    std::shared_ptr<Node<dim>> getNearestNode(const std::shared_ptr<Node<dim>> &node) const;
    std::vector<std::shared_ptr<Node<dim>>> getNearNodes(const Vector<dim> &vec, const float distance) const;
    std::vector<std::shared_ptr<Node<dim>>> getNearNodes(const Node<dim> &node, const float distance) const;
    std::vector<std::shared_ptr<Node<dim>>> getNearNodes(const std::shared_ptr<Node<dim>> node, const float distance) const;

    void sortTree();
    bool eraseNode(const std::shared_ptr<Node<dim>> &node);

    bool empty() const;
    size_t size() const;
    unsigned int getSortCount() const;
    bool autoSort() const;
    void preserveNodePtr();

    void clearParents();
    void clearQueryParents();
    void clearChildes();

    bool operator<(std::shared_ptr<Graph<dim>> const &a) {
        if (a->getNode(0) && this->getNode(0)) {
            return a->getNode(0)->getValues().norm() < this->getNode(0)->getValues().norm();
        } else {
            return false;
        }
    }

  private:
    std::vector<std::shared_ptr<Node<dim>>> m_nodes;
    std::shared_ptr<KDTree<dim, std::shared_ptr<Node<dim>>>> m_kdTree;
    std::mutex m_mutex;
    const unsigned int m_sortCount;
    bool m_autoSort = false;
    bool m_preserveNodePtr = false;
};

/*!
*  \brief      Default constructor of the class Graph
*  \author     Sascha Kaden
*  \date       2016-06-02
*/
template <unsigned int dim>
Graph<dim>::Graph(const unsigned int sortCount) : Identifier("Graph"), m_sortCount(sortCount) {
    m_autoSort = (sortCount != 0);
    m_kdTree = std::shared_ptr<KDTree<dim, std::shared_ptr<Node<dim>>>>(new KDTree<dim, std::shared_ptr<Node<dim>>>());
}

/*!
*  \brief      Destructor of the class Graph
*  \author     Sascha Kaden
*  \date       2017-01-07
*/
template <unsigned int dim>
Graph<dim>::~Graph() {
    if (!m_preserveNodePtr) {
        for (auto &&node : m_nodes) {
            if (node) {
                node->clearParent();
                node->clearQueryParent();
                node->clearChildes();
                node = nullptr;
            }
        }
    }
}

/*!
* \brief      Add a Node to the graph
* \author     Sascha Kaden
* \param[in]  Node
* \date       2016-05-25
*/
template <unsigned int dim>
void Graph<dim>::addNode(const std::shared_ptr<Node<dim>> &node) {
    m_mutex.lock();
    m_kdTree->addNode(node->getValues(), node);
    m_nodes.push_back(node);
    m_mutex.unlock();
    if (m_autoSort && (m_nodes.size() % m_sortCount) == 0) {
        sortTree();
    }
}

/*!
* \brief      Add Node list to the graph
* \author     Sascha Kaden
* \param[in]  Node list
* \date       2017-04-03
*/
template <unsigned int dim>
void Graph<dim>::addNodeList(const std::vector<std::shared_ptr<Node<dim>>> &nodes) {
    for (auto node : nodes) {
        addNode(node);
    }
}

/*!
* \brief      Return Node from index
* \author     Sascha Kaden
* \param[in]  Node
* \date       2017-04-03
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNode(const unsigned int index) const {
    if (index < m_nodes.size()) {
        return m_nodes[index];
    } else {
        return nullptr;
    }
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
* \brief      Search for nearrest neighbor
* \author     Sascha Kaden
* \param[in]  Vector from where the search starts
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const Vector<dim> &vec) const {
    return m_kdTree->searchNearestNeighbor(vec);
}

/*!
* \brief      Search for nearrest neighbor
* \author     Sascha Kaden
* \param[in]  Node from where the search starts
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const Node<dim> &node) const {
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
std::shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const std::shared_ptr<Node<dim>> &node) const {
    return m_kdTree->searchNearestNeighbor(node->getValues());
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
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const Vector<dim> &vec, const float range) const {
    return m_kdTree->searchRange(vec, range);
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
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const Node<dim> &node, const float range) const {
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
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const std::shared_ptr<Node<dim>> node, const float range) const {
    return m_kdTree->searchRange(node->getValues(), range);
}

/*!
* \brief      Creates a new sorted KDTree and set them as member
* \author     Sascha Kaden
* \date       2017-01-09
*/
template <unsigned int dim>
void Graph<dim>::sortTree() {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::shared_ptr<KDTree<dim, std::shared_ptr<Node<dim>>>> kdTree =
        std::shared_ptr<KDTree<dim, std::shared_ptr<Node<dim>>>>(new KDTree<dim, std::shared_ptr<Node<dim>>>(m_nodes));
    m_kdTree = kdTree;
    Logging::info("KD Tree has been sorted and have: " + std::to_string(m_nodes.size()) + " Nodes", this);
}

/*!
* \brief      Remove Node from Graph, erasing from the vector of Nodes
* \details    KDTree has to be sorted after erasing of the Nodes.
* \author     Sascha Kaden
* \param[in]  Node pointer
* \param[out] true, if node was found and erased
* \date       2017-04-18
*/
template <unsigned int dim>
bool Graph<dim>::eraseNode(const std::shared_ptr<Node<dim>> &node) {
    for (auto &&graphNode : m_nodes) {
        if (graphNode == node) {
            m_nodes.erase(graphNode);
            return true;
        }
    }
    return false;
    // Todo: add removing Node at KDTree
}

/*!
* \brief      Return true if Graph is empty
* \author     Sascha Kaden
* \param[out] state of Graph
* \date       2017-04-03
*/
template <unsigned int dim>
bool Graph<dim>::empty() const {
    if (m_nodes.size() == 0) {
        return true;
    } else {
        return false;
    }
}

/*!
* \brief      Return size of the graph
* \author     Sascha Kaden
* \param[out] size of Node vector
* \date       2016-08-09
*/
template <unsigned int dim>
size_t Graph<dim>::size() const {
    return m_nodes.size();
}

/*!
* \brief      Return sort count, if 0 auto sort is false
* \author     Sascha Kaden
* \param[out] count
* \date       2017-01-09
*/
template <unsigned int dim>
unsigned int Graph<dim>::getSortCount() const {
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
void Graph<dim>::clearParents() {
    for (auto &&node : m_nodes) {
        node->clearParent();
    }
}

/*!
* \brief      Clear all query parent pointer from the nodes
* \author     Sascha Kaden
* \date       2017-04-03
*/
template <unsigned int dim>
void Graph<dim>::clearQueryParents() {
    for (auto &&node : m_nodes) {
        node->clearQueryParent();
    }
}

/*!
* \brief      Clear all child pointer from the nodes
* \author     Sascha Kaden
* \date       2017-04-03
*/
template <unsigned int dim>
void Graph<dim>::clearChildes() {
    for (auto &&node : m_nodes) {
        node->clearChildes();
    }
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

} /* namespace rmpl */

#endif /* GRAPH_HPP */
