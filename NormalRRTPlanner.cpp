#include "NormalRRTPlanner.h"
#include "CollisionDetection.h"

void NormalRRTPlanner::computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode) {
    // get nearest neighbor
    shared_ptr<Node> nearestNode = this->m_graph.getNearestNode(Node(randVec));
    if (nearestNode == NULL)
        std::cout << "NULL" << std::endl;
    // compute node new with fixed step size
    Vec<float> newVec = RRTPlanner::computeNodeNew(randVec, nearestNode->getVec());
    newNode = shared_ptr<Node>(new Node(newVec));

    if (this->m_collision->controlCollision(newNode)) {
        newNode = NULL;
        return;
    }
    else if (!this->m_planner->computeTrajectory(newNode, nearestNode)) {
        newNode = NULL;
        return;
    }

    newNode->setParent(nearestNode);
    nearestNode->addChild(newNode);
}
