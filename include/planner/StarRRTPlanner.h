#ifndef STARRRTPLANNER_H_
#define STARRRTPLANNER_H_

#include "RRTPlanner.h"

using std::shared_ptr;

/*!
* \brief   Class of the StarRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class StarRRTPlanner : public RRTPlanner
{
public:
    StarRRTPlanner(const unsigned int &dim, const float stepSize, const TrajectoryMethod trajectory = TrajectoryMethod::linear, const SamplingMethod sampling = SamplingMethod::randomly)
    : RRTPlanner(dim, stepSize, trajectory, sampling) // Argumente an Basisklassenkonstruktor weiterleiten
    {
    }

protected:
    void computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode);
    void chooseParent(shared_ptr<Node> &newNode, shared_ptr<Node> &nearestNode, std::vector<shared_ptr<Node>> &nearNodes);
    void reWire(shared_ptr<Node> &newNode, shared_ptr<Node> &nearestNode, std::vector<shared_ptr<Node>> &nearNodes);

};

#endif /* STARRRTPLANNER_H_ */
