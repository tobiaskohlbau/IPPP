#include <planner/RRTPlanner.h>

#include <thread>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Constructor of the class RRTPlanner
*  \author     Sascha Kaden
*  \date       2016-05-27
*/
RRTPlanner::RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase> &robot, float stepSize, float trajectoryStepSize, TrajectoryMethod trajectory, SamplingMethod sampling)
    : Planner(name, robot, stepSize, trajectoryStepSize, trajectory, sampling)
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

    shared_ptr<Node> initNode(new Node(node));
    if (this->m_collision->controlCollision(initNode->getVec())) {
        this->sendMessage("Init node could not be connected", Message::warning);
        return false;
    }

    m_initNode = initNode;
    this->m_graph->addNode(m_initNode);
    return true;
}

/*!
*  \brief      Compute tree of the RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[out] return check of the constraints
*  \date       2016-05-27
*/
bool RRTPlanner::computeTree(int nbOfNodes, int nbOfThreades)
{
    if (!this->controlConstraints())
        return false;
    if (m_initNode == nullptr) {
        this->sendMessage("Init node is not connected", Message::warning);
        return false;
    }

    if (nbOfThreades == 1) {
        computeTreeThread(nbOfNodes);
    }
    else {
        nbOfNodes /= nbOfThreades;
        std::vector<std::thread> threads;

        for (int i = 0; i < nbOfThreades; ++i) {
            threads.push_back(std::thread(&RRTPlanner::computeTreeThread, this, nbOfNodes));
        }

        for (int i = 0; i < nbOfThreades; ++i)
            threads[i].join();
    }

    return true;
}

void RRTPlanner::computeTreeThread(int nbOfNodes) {
    for (int i = 0; i < nbOfNodes; ++i)
    {
        shared_ptr<Node> newNode;

        // compute randomly sample
        Vec<float> randVec = this->m_sampler->getSample(m_robot->getDim(), i, nbOfNodes);

        computeRRTNode(randVec, newNode);

        if (newNode == nullptr)
            continue;

        //newNode->getVec().print();
        this->m_graph->addNode(newNode);
    }
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
    this->sendMessage("Path has: " + std::to_string(nodes.size()) + " nodes", Message::info);
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
        this->sendMessage("Path is not complete", Message::warning);
        return path;
    }

    std::vector<std::shared_ptr<Node>> nodes = getPathNodes();
    std::shared_ptr<Node> temp = nodes[0];
    while (temp->getParent() != nullptr) {
        std::vector<Vec<float>> tempVecs = this->m_planner->computeTrajectory(temp->getVec(), temp->getParent()->getVec());
        for (int i = 0; i < tempVecs.size(); ++i) {
            path.push_back(tempVecs[i]);
        }
        temp = temp->getParent();
    }
    this->sendMessage("Path has: " + std::to_string(path.size()) + " points", Message::info);
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
* \date        2016-06-01
*/
std::shared_ptr<Node> RRTPlanner::getGoalNode() {
    return m_goalNode;
}

/*!
*  \brief      Return sample point of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] sample Vec
* \date        2016-07-22
*/
Vec<float> RRTPlanner::getSamplePoint() {
    return this->m_sampler->getSample(m_robot->getDim(), 0, 100);
}
