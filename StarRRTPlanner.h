#ifndef STARRRTPLANNER_H_
#define STARRRTPLANNER_H_

#include "RRTPlanner.h"

template<uint16_t dim>
class StarRRTPlanner : public RRTPlanner<dim>
{
public:
    StarRRTPlanner(const float stepSize, const TrajectoryPlanner::TrajectoryMethod trajectory = TrajectoryPlanner::TrajectoryMethod::linear, const SamplingMethod sampling = SamplingMethod::randomly)
    : RRTPlanner<dim>(stepSize, trajectory, sampling) // Argumente an Basisklassenkonstruktor weiterleiten
    {
    }

protected:
    void computeRRTNode(const Vec<dim, float> &randVec, std::shared_ptr<Node<dim>> &newNode);
    void chooseParent(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode, std::vector<std::shared_ptr<Node<dim>>> &nearNodes);
    void reWire(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode, std::vector<std::shared_ptr<Node<dim>>> &nearNodes);

};

template<uint16_t dim>
void StarRRTPlanner<dim>::computeRRTNode(const Vec<dim, float> &randVec, std::shared_ptr<Node<dim>> &newNode) {
    // get nearest neighbor
    std::shared_ptr<Node<dim>> nearestNode = this->m_graph.getNearestNode(Node<dim>(randVec));
    // set node new fix fixed step size of 10
    Vec<dim, float> newVec = RRTPlanner<dim>::computeNodeNew(randVec, nearestNode->getVec());
    newNode = std::shared_ptr<Node<dim>>(new Node<dim>(newVec));

    std::vector<std::shared_ptr<Node<dim>>> nearNodes;
    chooseParent(newNode, nearestNode, nearNodes);

    if (this->m_collision->template controlCollision(newNode)) {
        newNode = NULL;
        return;
    }
    else if (!this->m_planner->computeTrajectory(newNode, nearestNode)) {
        newNode = NULL;
        return;
    }

    newNode->setCost(newNode->getDist(*nearestNode) + nearestNode->getCost());
    newNode->setParent(nearestNode);
    nearestNode->addChild(newNode);

    reWire(newNode, nearestNode, nearNodes);
}

template<uint16_t dim>
void StarRRTPlanner<dim>::chooseParent(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode, std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    // get near nodes to the new node
    nearNodes = this->m_graph.getNearNodes(newNode, this->m_stepSize * 1.5);

    float nearestNodeCost = nearestNode->getCost();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < nearestNodeCost)
            if (this->m_planner->computeTrajectory(newNode, nearNodes[i])) {
                nearestNodeCost = nearNodes[i]->getCost();
                nearestNode = nearNodes[i];
            }
    }
}

template<uint16_t dim>
void StarRRTPlanner<dim>::reWire(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &parentNode, std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i] != parentNode && nearNodes[i]->getChilds().size() != 0) {
            float oldDist = nearNodes[i]->getCost();
            float newDist = nearNodes[i]->getDist(newNode) + newNode->getCost();
            if (newDist < oldDist) {
                if (this->m_planner->computeTrajectory(newNode, nearNodes[i])) {
                    float cost = nearNodes[i]->getCost() - nearNodes[i]->getDist(nearNodes[i]->getParent());
                    cost += newNode->getDist(nearNodes[i]);
                    nearNodes[i]->setCost(cost);
                    nearNodes[i]->setParent(newNode);
                }
            }
        }
    }
}

#endif /* STARRRTPLANNER_H_ */
