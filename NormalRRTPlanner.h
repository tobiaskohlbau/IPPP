#ifndef NORMALRRTPLANNER_H_
#define NORMALRRTPLANNER_H_

#include "RRTPlanner.h"

template<uint16_t dim>
class NormalRRTPlanner : public RRTPlanner<dim>
{
public:
    NormalRRTPlanner(float stepSize, TrajectoryPlanner::TrajectoryMethod trajectory = TrajectoryPlanner::TrajectoryMethod::linear, SamplingMethod sampling = SamplingMethod::randomly)
    : RRTPlanner<dim>(stepSize, trajectory, sampling) // Argumente an Basisklassenkonstruktor weiterleiten
    {
    }

protected:
    void computeRRTNode(const Vec<dim, float> &randVec, std::shared_ptr<Node<dim>> &newNode) {
        // get nearest neighbor
        std::shared_ptr<Node<dim>> nearestNode = this->m_graph.getNearestNode(Node<dim>(randVec));
        if (nearestNode == NULL)
            std::cout << "NULL" << std::endl;
        // compute node new with fixed step size
        Vec<dim, float> newVec = RRTPlanner<dim>::computeNodeNew(randVec, nearestNode->getVec());
        newNode = std::shared_ptr<Node<dim>>(new Node<dim>(newVec));

        if (this->m_collision->template controlCollision<dim>(newNode)) {
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
};

#endif /* NORMALRRTPLANNER_H_ */
