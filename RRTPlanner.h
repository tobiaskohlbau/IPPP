#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "Planner.h"

template<uint16_t dim>
class RRTPlanner : public Planner<dim>
{
public:
    RRTPlanner(float stepSize, TrajectoryPlanner::TrajectoryMethod trajectory, Sampling::SamplingMethod sampling);

    bool setInitNode(Node<dim> node);
    bool computeTree(const int nbOfNodes);
    bool connectGoalNode(std::shared_ptr<Node<dim>> goalNode);

protected:
    virtual void computeRRTNode(const Vec<dim, float> &randVec, std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode) = 0;
    Vec<dim, float> computeNodeNew(const Vec<dim, float> &randNode, const Vec<dim, float> &nearestNode);

    // variables
    Node<dim>  m_initNode;
    Node<dim>  m_goalNode;
};

template<uint16_t dim>
RRTPlanner<dim>::RRTPlanner(float stepSize, TrajectoryPlanner::TrajectoryMethod trajectory, Sampling::SamplingMethod sampling)
    : Planner<dim>(stepSize, trajectory, sampling)
{
}

template<uint16_t dim>
bool RRTPlanner<dim>::setInitNode(Node<dim> node) {
    cv::Size workspaceSize = this->m_workspace.size();
    if (node.getX() > workspaceSize.width || node.getY() > workspaceSize.height)
        return false;

    std::shared_ptr<Node<dim>> shrNode(new Node<dim>(node));
    if (this->m_collision->template controlCollision<dim>(shrNode))
        return false;

    m_initNode = node;
    this->m_graph.addNode(shrNode);
    return true;
}

template<uint16_t dim>
bool RRTPlanner<dim>::computeTree(const int nbOfNodes)
{
    if (dim == 2 && this->m_workspace.empty())
        return false;
    if (!this->controlConstraints())
        return false;

    this->m_sampler->setMaxWorkspaceSize(this->m_workspace.size().width);

    for (int i = 0; i < nbOfNodes; ++i)
    {
        std::shared_ptr<Node<dim>> nearestNode, newNode;
        // compute random sample
        Vec<dim, float> randVec = this->m_sampler->template getSample<dim>(i, nbOfNodes);

        computeRRTNode(randVec, newNode, nearestNode);

        if (newNode == NULL)
            continue;

        this->m_graph.addNode(newNode);
    }

    return true;
}

template<uint16_t dim>
bool RRTPlanner<dim>::connectGoalNode(std::shared_ptr<Node<dim>> goalNode) {
    if (this->m_collision->template controlCollision<dim>(goalNode))
        return false;
    std::shared_ptr<Node<dim>> nearestNode;

    std::vector<std::shared_ptr<Node<dim>>> nearNodes = this->m_graph.getNearNodes(goalNode, this->m_stepSize * 3);

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
    Vec<dim, float> u = randNode;
    u -= nearestNode;
    float k = this->m_stepSize / u.norm();
    u *= this->m_stepSize / u.norm();
    u += nearestNode;
    return u;
}

#endif /* RRTPLANNER_H_ */
