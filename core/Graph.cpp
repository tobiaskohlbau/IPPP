#include <core/Graph.h>

using std::shared_ptr;

/*!
* \brief      Add Node to the graph
* \author     Sascha Kaden
* \param[in]  Node
* \date       2016-05-25
*/
void Graph::addNode(shared_ptr<Node> node) {
    m_nodes.push_back(node);
    m_kdTree.addNode(node->getVec(), node);
}

/*!
* \brief      Remove Node from the graph
* \author     Sascha Kaden
* \param[in]  index
* \date       2016-05-25
*/
void Graph::removeNode(const int index) {
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
std::vector<shared_ptr<Node>> Graph::getNodes() const{
    return m_nodes;
}

/*!
* \brief      Search for nearrest neighbor
* \author     Sascha Kaden
* \param[in]  Node for the search
* \param[out] nearest neighbor Node
* \date       2016-05-25
*/
shared_ptr<Node> Graph::getNearestNode(const Node node) {
    return m_kdTree.searchNearestNeighbor(node.getVec());
}

/*!
* \brief      Search range
* \author     Sascha Kaden
* \param[in]  Node for the search
* \param[in]  distance around the given Node
* \param[out] list of nodes inside the range
* \date       2016-05-25
*/
std::vector<shared_ptr<Node>> Graph::getNearNodes(const shared_ptr<Node> node, const float &distance) {
    return m_kdTree.searchRange(node->getVec(), distance);
}
