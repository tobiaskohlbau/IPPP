#ifndef GRAPH_H_
#define GRAPH_H_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include "KDTree.h"
#include "Node.h"
#include "SearchNearestNeighbor.h"

using std::shared_ptr;

template<unsigned int dim>
class Graph
{
public:
    void addNode(shared_ptr<Node<dim>> node);
    void removeNode(const int index);
    std::vector<shared_ptr<Node<dim>>> getNodes() const;

    shared_ptr<Node<dim>> getNearestNode(const Node<dim> node);
    std::vector<shared_ptr<Node<dim>>> getNearNodes(const shared_ptr<Node<dim>> node, const float &distance);

private:
    std::vector<shared_ptr<Node<dim>>> m_nodes;
    KDTree<dim, shared_ptr<Node<dim>>> m_kdTree;
};

template<unsigned int dim>
void Graph<dim>::addNode(shared_ptr<Node<dim>> node) {
    m_nodes.push_back(node);
    m_kdTree.addNode(node->getVec(), node);
}

template<unsigned int dim>
void Graph<dim>::removeNode(const int index) {
    if (index < m_nodes.size()) {
        m_nodes.erase(m_nodes.begin() + index);
    }
}

template<unsigned int dim>
std::vector<shared_ptr<Node<dim>>> Graph<dim>::getNodes() const{
    return m_nodes;
}

template<unsigned int dim>
shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const Node<dim> node) {
    return m_kdTree.searchNearestNeighbor(node.getVec());
}

template<unsigned int dim>
std::vector<shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const shared_ptr<Node<dim>> node, const float &distance) {
    return m_kdTree.searchRange(node->getVec(), distance);
}


#endif /* GRAPH_H_ */
