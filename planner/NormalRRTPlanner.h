#ifndef NORMALRRTPLANNER_H_
#define NORMALRRTPLANNER_H_

#include "RRTPlanner.h"

using std::shared_ptr;

/*!
* \brief   Class of the NormalRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class NormalRRTPlanner : public RRTPlanner
{
public:
    NormalRRTPlanner(const unsigned int &dim, float stepSize, TrajectoryMethod trajectory = TrajectoryMethod::linear, SamplingMethod sampling = SamplingMethod::randomly)
    : RRTPlanner(dim, stepSize, trajectory, sampling)
    {
    }

protected:
    void computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode);
};


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

#endif /* NORMALRRTPLANNER_H_ */
