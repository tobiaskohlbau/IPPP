#include <ctime>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

void printTime(clock_t begin, clock_t end) {
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

void simpleRRT() {
    const unsigned int dim = 6;
    std::shared_ptr<Jaco> robot(new Jaco());
    std::shared_ptr<Environment> environment(new Environment(3, AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    // environment->addObstacle(obstacleModel);

    robot->saveMeshConfig(util::Vecd(0, 0, 0, 0, 0, 0));

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setCollisionType(CollisionType::PQP);

    RRT<dim> planner(environment, creator.getRRTOptions(30), creator.getGraph());
    Vector6 start = util::Vecd(180, 180, 180, 180, 180, 180);
    Vector6 goal = util::Vecd(275, 167.5, 57.4, 241, 82.7, 75.5);

    // compute the tree
    clock_t begin = std::clock();
    bool connected = planner.computePath(start, goal, 4000, 6);
    clock_t end = std::clock();
    printTime(begin, end);

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner.getGraphNodes();
    std::vector<Transform> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (auto & node : nodes)
        graphPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(node->getValues()));
    //writer::writeVecsToFile<dim>(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        std::vector<Vector6> pathAngles = planner.getPath(5);

        std::vector<Transform> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(angles));
        //writer::appendVecsToFile<dim>(pathPoints, "example.ASC", 10);

        // Helper vrep(dim);
        // vrep.start();
        // for (auto angles : pathAngles)
        //    vrep.setPos(angles);
    }
}

void treeConnection() {
    const unsigned int dim = 6;
    std::shared_ptr<RobotBase> robot(new Jaco());
    std::shared_ptr<Environment> environment(new Environment(3, AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    // environment->addObstacle(obstacleModel);

    // create two trees from init and from goal
    ModuleConfigurator<dim> creator1;
    creator1.setEnvironment(environment);
    creator1.setCollisionType(CollisionType::PQP);
    ModuleConfigurator<dim> creator2;
    creator2.setEnvironment(environment);
    creator2.setCollisionType(CollisionType::PQP);

    RRTStar<dim> plannerGoalNode(environment, creator1.getRRTOptions(20), creator1.getGraph());
    RRTStar<dim> plannerInitNode(environment, creator2.getRRTOptions(20), creator2.getGraph());

    // set properties to the planners
    plannerInitNode.setInitNode(util::Vecd(180, 180, 180, 180, 180, 180));
    plannerGoalNode.setInitNode(util::Vecd(275, 167.5, 57.4, 241, 82.7, 75.5));

    // compute the tree
    clock_t begin = std::clock();
    plannerInitNode.computeTree(20000, 2);
    plannerGoalNode.computeTree(20000, 2);
    clock_t end = std::clock();
    printTime(begin, end);

    // get random sample from the first pathPlanner and try to connect to both planners
    Eigen::VectorXd goal;
    bool connected = false;
    double minCost = std::numeric_limits<double>::max();
    for (int i = 0; i < 10000; ++i) {
        //        Vec<double> sample = plannerInitNode.getSamplePoint();
        //
        //        bool planner1Connected = plannerInitNode.connectGoalNode(sample);
        //        bool planner2Connected = plannerGoalNode.connectGoalNode(sample);
        //        if (planner1Connected && planner2Connected) {
        //            double cost = plannerInitNode.getGoalNode()->getCost() + plannerGoalNode.getGoalNode()->getCost();
        //            if (cost < minCost) {
        //                goal = sample;
        //                connected = true;
        //            }
        //        }
    }

    std::vector<std::shared_ptr<Node<dim>>> nodes = plannerInitNode.getGraphNodes();
    std::vector<Transform> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (auto & node : nodes)
        graphPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(node->getValues()));
    //writer::writeVecsToFile<dim>(graphPoints, "example.ASC", 10);

    nodes = plannerGoalNode.getGraphNodes();
    std::cout << "Goal Graph has: " << nodes.size() << "nodes" << std::endl;
    for (auto & node : nodes)
        graphPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(node->getValues()));
    //writer::appendVecsToFile<dim>(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        plannerInitNode.connectGoalNode(goal);
        plannerGoalNode.connectGoalNode(goal);

        std::vector<Vector6> pathAngles = plannerInitNode.getPath(5);
        std::vector<Vector6> temp = plannerGoalNode.getPath(5);
        pathAngles.insert(pathAngles.end(), temp.begin(), temp.end());

        std::vector<Transform> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(angles));
        //writer::appendVecsToFile<dim>(pathPoints, "example.ASC", 10);

        // for (int i = 0; i < pathPoints.size(); ++i)
        //    vrep->setPos(pathAngles[i]);
    }
}

int main(int argc, char** argv) {
    // treeConnection();
    simpleRRT();

    return 0;
}
