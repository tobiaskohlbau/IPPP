#include <core/Graph.h>

#include <math.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Default constructor of the class Graph
*  \author     Sascha Kaden
*  \date       2016-06-02
*/
Graph::Graph()
    : Base("Graph") {
    m_treeSorted = false;
}

/*!
* \brief      Add Node to the graph
* \author     Sascha Kaden
* \param[in]  Node
* \date       2016-05-25
*/
void Graph::addNode(const shared_ptr<Node> &node) {
    m_nodes.push_back(node);
    m_kdTree.addNode(node->getVec(), node);
    if (m_nodes.size() > 20000 && !m_treeSorted) {
        m_treeSorted = true;
        m_kdTree = KDTree<std::shared_ptr<Node>>();
        m_kdTree.rebuildTree(m_nodes);
        this->sendMessage("KD Tree has been sorted", Message::info);
    }
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
    return m_kdTree.searchNearestNeighbor(node.getVec());
}

/*!
* \brief      Search for nearrest neighbor
* \author     Sascha Kaden
* \param[in]  pointer of the Node for the search
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
shared_ptr<Node> Graph::getNearestNode(const shared_ptr<Node> &node) {
    return m_kdTree.searchNearestNeighbor(node->getVec());
}

/*!
* \brief      Search range
* \author     Sascha Kaden
* \param[in]  Node for the search
* \param[in]  distance around the given Node
* \param[out] list of nodes inside the range
* \date       2016-05-25
*/
std::vector<shared_ptr<Node>> Graph::getNearNodes(const shared_ptr<Node> node, float distance) {
    return m_kdTree.searchRange(node->getVec(), distance);
}
