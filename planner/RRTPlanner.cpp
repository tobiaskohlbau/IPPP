#include <planner/RRTPlanner.h>

/*!
*  \brief      Constructor of the class RRTPlanner
*  \author     Sascha Kaden
*  \date       2016-05-27
*/
RRTPlanner::RRTPlanner(const unsigned int &dim, float stepSize, TrajectoryMethod trajectory, SamplingMethod sampling)
    : Planner(dim, stepSize, trajectory, sampling)
{
}

/*!
*  \brief      Set init Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  initial Node
*  \param[out] true, if set was possible
*  \date       2016-05-27
*/
bool RRTPlanner::setInitNode(Node node) {
    this->controlConstraints();
    for (unsigned int i = 0; i < this->m_dim; ++i)
        if (node.getVec()[i] < this->m_minBoundary[i] || node.getVec()[i] > this->m_maxBoundary[i])
            return false;

    shared_ptr<Node> shrNode(new Node(node));
    if (this->m_collision->controlCollision(shrNode))
        return false;

    m_initNode = node;
    this->m_graph.addNode(shrNode);
    return true;
}

/*!
*  \brief      Compute tree of the RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[out] return check of the constraints
*  \date       2016-05-27
*/
bool RRTPlanner::computeTree(const int nbOfNodes)
{
    if (!this->controlConstraints())
        return false;

    for (int i = 0; i < nbOfNodes; ++i)
    {
        shared_ptr<Node> newNode;
        // compute randomly sample
        Vec<float> randVec = this->m_sampler->getSample(m_dim, i, nbOfNodes);

        computeRRTNode(randVec, newNode);

        if (newNode == nullptr)
            continue;

        //newNode->getVec().print();
        this->m_graph.addNode(newNode);
    }

    return true;
}

/*!
*  \brief      Connect goal Node
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was possible
*  \date       2016-05-27
*/
bool RRTPlanner::connectGoalNode(shared_ptr<Node> goalNode) {
    if (this->m_collision->controlCollision(goalNode))
        return false;

    std::vector<shared_ptr<Node>> nearNodes = this->m_graph.getNearNodes(goalNode, this->m_stepSize * 3);

    shared_ptr<Node> nearestNode;
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

/*!
*  \brief      Computation of a new Node from the RRT algorithm
*  \author     Sascha Kaden
*  \param[in]  random Node
*  \param[in]  nearest Node
*  \param[out] Node new
*  \date       2016-05-27
*/
Vec<float> RRTPlanner::computeNodeNew(const Vec<float> &randNode, const Vec<float> &nearestNode) {
    std::cout << (randNode.getDist(nearestNode)) << std::endl;
    if (randNode.getDist(nearestNode) < this->m_stepSize)
        return randNode;

    // p = a + k * (b-a)
    // ||u|| = ||b - a||
    // k = stepSize / ||u||
    Vec<float> u = randNode - nearestNode;
    u *= this->m_stepSize / u.norm();
    u += nearestNode;
    std::cout << "U: ";
    u.print();
    return u;
}
