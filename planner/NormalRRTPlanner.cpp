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
    nearestNode->addChild(newNode);
}
