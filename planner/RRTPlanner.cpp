#include <planner/RRTPlanner.h>

using namespace rmpl;
using std::shared_ptr;

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

    shared_ptr<Node> initNode(new Node(node));
    if (this->m_collision->controlCollision(initNode))
        return false;

    m_initNode = initNode;
    this->m_graph->addNode(initNode);
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
        this->m_graph->addNode(newNode);
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
bool RRTPlanner::connectGoalNode(Node goal) {
    shared_ptr<Node> goalNode(new Node(goal));
    if (this->m_collision->controlCollision(goalNode))
        return false;

    std::vector<shared_ptr<Node>> nearNodes = this->m_graph->getNearNodes(goalNode, this->m_stepSize * 3);

    shared_ptr<Node> nearestNode;
    float minCost = std::numeric_limits<float>::max();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < minCost) {
            if (this->m_planner->controlTrajectory(goalNode, nearNodes[i], 0.01)) {
                minCost = nearNodes[i]->getCost();
                nearestNode = nearNodes[i];
            }
        }
    }

    if (minCost < std::numeric_limits<float>::max()) {
        m_goalNode = goalNode;
        goalNode->setParent(nearestNode);
        this->m_graph->addNode(goalNode);
        std::cout << "Goal node is connected" << std::endl;
        this->m_pathPlanned = true;
        return true;
    }

    std::cout << "Goal node is NOT connected" << std::endl;

    return false;
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2016-05-31
*/
std::vector<std::shared_ptr<Node>> RRTPlanner::getPathNodes() {
    std::vector<shared_ptr<Node>> nodes;
    if (!this->m_pathPlanned)
        return nodes;

    nodes.push_back(m_goalNode);
    shared_ptr<Node> temp = m_goalNode->getParent();
    while(temp != nullptr) {
        nodes.push_back(temp);
        temp = temp->getParent();
    }
    std::cout << "Path has: " << nodes.size() << " nodes" << std::endl;
    return nodes;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[out] vecs of the path
*  \date       2016-05-31
*/
std::vector<Vec<float>> RRTPlanner::getPath() {
    std::vector<Vec<float>> path;
    if (!this->m_pathPlanned) {
        std::cout << "Path is not complete" << std::endl;
        return path;
    }

    std::vector<std::shared_ptr<Node>> nodes = getPathNodes();
    std::shared_ptr<Node> temp = nodes[0];
    while (temp->getParent() != nullptr) {
        std::vector<Vec<float>> tempVecs = this->m_planner->computeTrajectory(temp->getVec(), temp->getParent()->getVec(), 0.01);
        for (int i = 0; i < tempVecs.size(); ++i) {
            path.push_back(tempVecs[i]);
        }
        temp = temp->getParent();
    }
    std::cout << "Path has: " << path.size() << " points" << std::endl;
    return path;
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
    if (randNode.getDist(nearestNode) < this->m_stepSize)
        return randNode;

    // p = a + k * (b-a)
    // ||u|| = ||b - a||
    // k = stepSize / ||u||
    Vec<float> u = randNode - nearestNode;
    u *= this->m_stepSize / u.norm();
    u += nearestNode;
    return u;
}

/*!
*  \brief      Return the init Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] init Node
* \date        2016-06-01
*/
std::shared_ptr<Node> RRTPlanner::getInitNode() {
    return m_initNode;
}

/*!
*  \brief      Return the goal Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] goal Node
* \date        s2016-06-01
*/
std::shared_ptr<Node> RRTPlanner::getGoalNode() {
    return m_goalNode;
}
