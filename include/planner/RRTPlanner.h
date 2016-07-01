#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "Planner.h"

namespace rmpl {

/*!
* \brief   Super class of the RRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class RRTPlanner : public Planner
{
public:
    RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase> &robot, const float &stepSize, const float &trajectoryStepSize, TrajectoryMethod trajectory, SamplingMethod sampling);

    bool setInitNode(const Node &node);
    bool computeTree(const int &nbOfNodes);
    bool connectGoalNode(const Node &goalNode);
    std::vector<std::shared_ptr<Node>> getPathNodes();
    std::vector<Vec<float>> getPath();
    std::shared_ptr<Node> getInitNode();
    std::shared_ptr<Node> getGoalNode();

protected:
    virtual void computeRRTNode(const Vec<float> &randVec, std::shared_ptr<Node> &newNode) = 0;
    Vec<float> computeNodeNew(const Vec<float> &randNode, const Vec<float> &nearestNode);

    // variables
    std::shared_ptr<Node>  m_initNode;
    std::shared_ptr<Node>  m_goalNode;
};

} /* namespace rmpl */

#endif /* RRTPLANNER_H_ */
