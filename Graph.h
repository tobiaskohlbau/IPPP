#ifndef GRAPH_H_
#define GRAPH_H_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include "KDTree.h"
#include "Node.h"
#include "SearchNearestNeighbor.h"

template<uint16_t dim>
class Graph
{
public:
    void addNode(std::shared_ptr<Node<dim>> node);
    void removeNode(const int index);
    std::vector<std::shared_ptr<Node<dim>>> getNodes() const;

    std::shared_ptr<Node<dim>> getNearestNode(const Node<dim> node);
    std::vector<std::shared_ptr<Node<dim>>> getNearNodes(const std::shared_ptr<Node<dim>>, const float distance);

private:
    std::vector<std::shared_ptr<Node<dim>>> m_nodes;
    KDTree<dim, std::shared_ptr<Node<dim>>> m_kdTree;
};

template<uint16_t dim>
void Graph<dim>::addNode(std::shared_ptr<Node<dim>> node) {
    m_nodes.push_back(node);
    m_kdTree.addNode(node->getVec(), node);
}

template<uint16_t dim>
void Graph<dim>::removeNode(const int index) {
    if (index < m_nodes.size()) {
        m_nodes.erase(m_nodes.begin() + index);
    }
}

template<uint16_t dim>
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNodes() const{
    return m_nodes;
}

template<uint16_t dim>
std::shared_ptr<Node<dim>> Graph<dim>::getNearestNode(const Node<dim> node) {
    return m_kdTree.getNearestNeighbor(node.getVec());
}

template<uint16_t dim>
std::vector<std::shared_ptr<Node<dim>>> Graph<dim>::getNearNodes(const std::shared_ptr<Node<dim>> node, const float distance) {
    return SearchNearestNeighbor::searchNearNodes(m_nodes, node, distance);
}


#endif /* GRAPH_H_ */
