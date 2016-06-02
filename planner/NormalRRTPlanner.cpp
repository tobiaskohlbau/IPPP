#include <planner/NormalRRTPlanner.h>

using namespace rmpl;
using std::shared_ptr;

void NormalRRTPlanner::computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode) {
    // get nearest neighbor
    shared_ptr<Node> nearestNode = this->m_graph->getNearestNode(Node(randVec));

    // compute node new with fixed step size
    Vec<float> newVec = RRTPlanner::computeNodeNew(randVec, nearestNode->getVec());
    newNode = shared_ptr<Node>(new Node(newVec));

    if (this->m_collision->controlCollision(newNode)) {
        newNode = nullptr;
        return;
    }
    else if (!this->m_planner->controlTrajectory(newNode, nearestNode, 0.01)) {
        newNode = nullptr;
        return;
    }

    newNode->setParent(nearestNode);
    nearestNode->addChild(newNode);
}
