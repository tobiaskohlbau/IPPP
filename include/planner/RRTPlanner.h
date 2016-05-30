#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "Planner.h"

using std::shared_ptr;

/*!
* \brief   Super class of the RRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class RRTPlanner : public Planner
{
public:
    RRTPlanner(const unsigned int &dim, float stepSize, TrajectoryMethod trajectory, SamplingMethod sampling);

    bool setInitNode(Node node);
    bool computeTree(const int nbOfNodes);
    bool connectGoalNode(shared_ptr<Node> goalNode);

protected:
    virtual void computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode) = 0;
    Vec<float> computeNodeNew(const Vec<float> &randNode, const Vec<float> &nearestNode);

    // variables
    Node  m_initNode;
    Node  m_goalNode;
};

#endif /* RRTPLANNER_H_ */
