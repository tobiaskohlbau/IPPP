#include <planner/StarRRTPlanner.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief         Computation of the new Node by the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in]     random Vec
*  \param[in,out] new Node
*  \date          2016-06-02
*/
void StarRRTPlanner::computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode) {
    // get nearest neighbor
    shared_ptr<Node> nearestNode = this->m_graph->getNearestNode(Node(randVec));
    // set node new fix fixed step size of 10
    Vec<float> newVec = RRTPlanner::computeNodeNew(randVec, nearestNode->getVec());
    newNode = shared_ptr<Node>(new Node(newVec));

    std::vector<shared_ptr<Node>> nearNodes;
    chooseParent(newNode, nearestNode, nearNodes);

    if (this->m_collision->controlCollision(newNode)) {
        newNode = nullptr;
        return;
    }
    else if (!this->m_planner->controlTrajectory(newNode, nearestNode, 0.01)) {
        newNode = nullptr;
        return;
    }

    newNode->setCost(newNode->getDist(*nearestNode) + nearestNode->getCost());
    newNode->setParent(nearestNode);
    nearestNode->addChild(newNode);

    reWire(newNode, nearestNode, nearNodes);
}

/*!
*  \brief         Choose parent algorithm from the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in,out] new Node
*  \param[in,out] nearest Node
*  \param[in,out] vecotr of nearest nodes
*  \date          2016-06-02
*/
void StarRRTPlanner::chooseParent(shared_ptr<Node> &newNode, shared_ptr<Node> &nearestNode, std::vector<shared_ptr<Node>> &nearNodes) {
    // get near nodes to the new node
    nearNodes = this->m_graph->getNearNodes(newNode, this->m_stepSize * 1.5);

    float nearestNodeCost = nearestNode->getCost();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < nearestNodeCost)
            if (this->m_planner->controlTrajectory(newNode, nearNodes[i], 0.01)) {
                nearestNodeCost = nearNodes[i]->getCost();
                nearestNode = nearNodes[i];
            }
    }
}

/*!
*  \brief         Rewire algorithm from the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in,out] new Node
*  \param[in,out] parent Node
*  \param[in,out] vecotr of nearest nodes
*  \date          2016-06-02
*/
void StarRRTPlanner::reWire(shared_ptr<Node> &newNode, shared_ptr<Node> &parentNode, std::vector<shared_ptr<Node>> &nearNodes) {
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i] != parentNode && nearNodes[i]->getChilds().size() != 0) {
            float oldDist = nearNodes[i]->getCost();
            float newDist = nearNodes[i]->getDist(newNode) + newNode->getCost();
            if (newDist < oldDist) {
                if (this->m_planner->controlTrajectory(newNode, nearNodes[i], 0.01)) {
                    float cost = nearNodes[i]->getCost() - nearNodes[i]->getDist(nearNodes[i]->getParent());
                    cost += newNode->getDist(nearNodes[i]);
                    nearNodes[i]->setCost(cost);
                    nearNodes[i]->setParent(newNode);
                }
            }
        }
    }
}
