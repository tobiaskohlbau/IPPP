#ifndef GRAPH_H_
#define GRAPH_H_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include <core/KDTree.hpp>
#include <core/Node.h>

using std::shared_ptr;

/*!
* \brief   Class Graph contain all nodes of the planner and offers the nearest neighbor and range search through a KDTree
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Graph
{
public:
    void addNode(shared_ptr<Node> node);
    void removeNode(const int index);
    std::vector<shared_ptr<Node>> getNodes() const;

    shared_ptr<Node> getNearestNode(const Node node);
    std::vector<shared_ptr<Node>> getNearNodes(const shared_ptr<Node> node, const float &distance);

private:
    std::vector<shared_ptr<Node>> m_nodes;
    KDTree<shared_ptr<Node>> m_kdTree;
};

#endif /* GRAPH_H_ */
