#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "Planner.h"

using std::shared_ptr;

template<uint16_t dim>
class RRTPlanner : public Planner<dim>
{
public:
    RRTPlanner(float stepSize, TrajectoryMethod trajectory, SamplingMethod sampling);

    bool setInitNode(Node<dim> node);
    bool computeTree(const int nbOfNodes);
    bool connectGoalNode(shared_ptr<Node<dim>> goalNode);

protected:
    virtual void computeRRTNode(const Vec<dim, float> &randVec, shared_ptr<Node<dim>> &newNode) = 0;
    Vec<dim, float> computeNodeNew(const Vec<dim, float> &randNode, const Vec<dim, float> &nearestNode);

    // variables
    Node<dim>  m_initNode;
    Node<dim>  m_goalNode;
};

template<uint16_t dim>
RRTPlanner<dim>::RRTPlanner(float stepSize, TrajectoryMethod trajectory, SamplingMethod sampling)
    : Planner<dim>(stepSize, trajectory, sampling)
{
}

template<uint16_t dim>
bool RRTPlanner<dim>::setInitNode(Node<dim> node) {
    this->controlConstraints();
    for (uint16_t i = 0; i < dim; ++i)
        if (node.getVec()[i] < this->m_minBoundary[i] || node.getVec()[i] > this->m_maxBoundary[i])
            return false;

    shared_ptr<Node<dim>> shrNode(new Node<dim>(node));
    if (this->m_collision->template controlCollision<dim>(shrNode))
        return false;

    m_initNode = node;
    this->m_graph.addNode(shrNode);
    return true;
}

template<uint16_t dim>
bool RRTPlanner<dim>::computeTree(const int nbOfNodes)
{
    if (!this->controlConstraints())
        return false;

    for (int i = 0; i < nbOfNodes; ++i)
    {
        shared_ptr<Node<dim>> newNode;
        // compute randomly sample
        Vec<dim, float> randVec = this->m_sampler->getSample(i, nbOfNodes);

        computeRRTNode(randVec, newNode);

        if (newNode == NULL)
            continue;

        this->m_graph.addNode(newNode);
    }

    return true;
}

template<uint16_t dim>
bool RRTPlanner<dim>::connectGoalNode(shared_ptr<Node<dim>> goalNode) {
    if (this->m_collision->template controlCollision<dim>(goalNode))
        return false;

    std::vector<shared_ptr<Node<dim>>> nearNodes = this->m_graph.getNearNodes(goalNode, this->m_stepSize * 3);

    shared_ptr<Node<dim>> nearestNode;
    float minCost = std::numeric_limits<float>::max();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < minCost) {
            if (this->m_planner->computeTrajectory(goalNode, nearNodes[i])) {
                minCost = nearNodes[i]->getCost();
                nearestNode = nearNodes[i];
            }
        }
    }

    if (minCost < std::numeric_limits<float>::max()) {
        goalNode->setParent(nearestNode);
        this->m_graph.addNode(goalNode);
        std::cout << "goalNode connected" << std::endl;
        return true;
    }

    return false;
}

template<uint16_t dim>
Vec<dim, float> RRTPlanner<dim>::computeNodeNew(const Vec<dim, float> &randNode, const Vec<dim, float> &nearestNode) {
    if (randNode.getDist(nearestNode) < this->m_stepSize)
        return randNode;

    // p = a + k * (b-a)
    // ||u|| = ||b - a||
    // k = stepSize / ||u||
    Vec<dim, float> u = randNode - nearestNode;
    u *= this->m_stepSize / u.norm();
    u += nearestNode;
    return u;
}

#endif /* RRTPLANNER_H_ */
