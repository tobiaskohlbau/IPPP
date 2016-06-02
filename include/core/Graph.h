#ifndef GRAPH_H_
#define GRAPH_H_

#include <algorithm>

#include <core/Base.h>
#include <core/KDTree.hpp>
#include <core/Node.h>

namespace rmpl{

/*!
* \brief   Class Graph contain all nodes of the planner and offers the nearest neighbor and range search through a KDTree
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Graph : public Base
{
public:
    Graph();
    void addNode(const std::shared_ptr<Node> &node);
    void removeNode(const int index);
    std::vector<std::shared_ptr<Node>> getNodes() const;

    std::shared_ptr<Node> getNearestNode(const Node node);
    std::vector<std::shared_ptr<Node>> getNearNodes(const std::shared_ptr<Node> node, const float &distance);

private:
    std::vector<std::shared_ptr<Node>> m_nodes;
    KDTree<std::shared_ptr<Node>> m_kdTree;
};

} /* namespace rmpl */

#endif /* GRAPH_H_ */
