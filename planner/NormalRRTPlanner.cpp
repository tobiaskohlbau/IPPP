#include <planner/NormalRRTPlanner.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief         Computation of the new Node by the normal RRT algorithm
*  \author        Sascha Kaden
*  \param[in]     random Vec
*  \param[in,out] new Node
*  \date          2016-06-02
*/
void NormalRRTPlanner::computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode) {
    // get nearest neighbor
    shared_ptr<Node> nearestNode = this->m_graph->getNearestNode(Node(randVec));

    // compute node new with fixed step size
    Vec<float> newVec = RRTPlanner::computeNodeNew(randVec, nearestNode->getVec());
    newNode = shared_ptr<Node>(new Node(newVec));

    if (this->m_collision->controlCollision(newNode->getVec())) {
        newNode = nullptr;
        return;
    }
    else if (!this->m_planner->controlTrajectory(newNode->getVec(), nearestNode->getVec())) {
        newNode = nullptr;
        return;
    }

    newNode->setParent(nearestNode);

    m_mutex.lock();
    nearestNode->addChild(newNode);
    m_mutex.unlock();
}

/*!
*  \brief      Connect goal Node
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was possible
*  \date       2016-05-27
*/
bool NormalRRTPlanner::connectGoalNode(Node goal) {
    if (this->m_collision->controlCollision(goal.getVec()))
        return false;

    shared_ptr<Node> goalNode(new Node(goal));
    std::vector<shared_ptr<Node>> nearNodes;
    nearNodes = this->m_graph->getNearNodes(goalNode, this->m_stepSize * 3);

    shared_ptr<Node> nearestNode = nullptr;
    for (auto node : nearNodes) {
        if (this->m_planner->controlTrajectory(goal.getVec(), node->getVec())) {
            nearestNode = node;
            break;
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode);
        this->m_goalNode = goalNode;
        this->m_graph->addNode(goalNode);
        //this->sendMessage("Goal Node is connected", Message::info);
        this->m_pathPlanned = true;
        return true;
    }

    //this->sendMessage("Goal Node is NOT connected", Message::warning);

    return false;
}
