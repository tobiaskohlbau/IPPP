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

    if (this->m_collision->controlCollision(newNode->getVec())) {
        newNode = nullptr;
        return;
    }
    else if (!this->m_planner->controlTrajectory(newNode->getVec(), nearestNode->getVec())) {
        newNode = nullptr;
        return;
    }

    newNode->setCost(newNode->getDist(*nearestNode) + nearestNode->getCost());
    newNode->setParent(nearestNode);
    m_mutex.lock();
    nearestNode->addChild(newNode);
    m_mutex.unlock();

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
    nearNodes = this->m_graph->getNearNodes(newNode, this->m_stepSize);

    float nearestNodeCost = nearestNode->getCost();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < nearestNodeCost) {
            if (this->m_planner->controlTrajectory(newNode->getVec(), nearNodes[i]->getVec())) {
                nearestNodeCost = nearNodes[i]->getCost();
                nearestNode = nearNodes[i];
            }
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
    for (auto nearNode : nearNodes) {
        if (nearNode != parentNode && nearNode->getChildEdges().size() != 0) {
            float oldDist = nearNode->getCost();
            float newDist = nearNode->getDist(newNode) + newNode->getCost();
            if (newDist < oldDist) {
                if (this->m_planner->controlTrajectory(newNode->getVec(), nearNode->getVec())) {
                    float cost = nearNode->getCost() - nearNode->getDist(nearNode->getParent());
                    cost += newNode->getDist(nearNode);
                    m_mutex.lock();
                    nearNode->setCost(cost);
                    nearNode->setParent(newNode);
                    m_mutex.unlock();
                }
            }
        }
    }
}
