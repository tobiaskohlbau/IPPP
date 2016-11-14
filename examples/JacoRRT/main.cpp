#include <ctime>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/Jaco.h>
#include <ui/vrep/Helper.h>

#include <ui/Writer.h>

void printTime(clock_t begin, clock_t end) {
    float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

void simpleRRT() {
    std::shared_ptr<rmpl::Jaco> robot(new rmpl::Jaco());
    std::shared_ptr<rmpl::RRTOptions> options(new rmpl::RRTOptions(30, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly));
    rmpl::NormalRRTPlanner planner(robot, options);
    rmpl::Vec<float> start(180, 180, 180, 180, 180, 180);
    //planner.setInitNode(rmpl::Node(0, 0, 0, 0, 0, 0));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(25000, 2);
    clock_t end = std::clock();
    printTime(begin, end);

    bool connected = planner.connectGoalNode(rmpl::Vec<float>(275, 167.5, 57.4, 241, 82.7, 75.5));

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();
    std::vector<rmpl::Vec<float>> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getVec()));
    Writer::writeVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        std::vector<rmpl::Vec<float>> pathAngles = planner.getPath(5, true);

        std::vector<rmpl::Vec<float>> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        Writer::appendVecsToFile(pathPoints, "example.ASC", 10);

        rmpl::Helper vrep(6);
        vrep.start();
        for (auto angles : pathAngles)
            vrep.setPos(angles);
    }
}

void treeConnection() {
    std::shared_ptr<rmpl::Jaco> robot(new rmpl::Jaco());

    // create two trees from init and from goal
    std::shared_ptr<rmpl::RRTOptions> options(new rmpl::RRTOptions(20, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly));
    rmpl::StarRRTPlanner plannerGoalNode(robot, options);
    rmpl::StarRRTPlanner plannerInitNode(robot, options);

    // set properties to the planners
    plannerInitNode.setInitNode(rmpl::Vec<float>(180, 180, 180, 180, 180, 180));
    plannerGoalNode.setInitNode(rmpl::Vec<float>(275, 167.5, 57.4, 241, 82.7, 75.5));

    // compute the tree
    clock_t begin = std::clock();
    plannerInitNode.computeTree(20000, 2);
    plannerGoalNode.computeTree(20000, 2);
    clock_t end = std::clock();
    printTime(begin, end);

    // get random sample from the first pathPlanner and try to connect to both planners
    rmpl::Vec<float> goal;
    bool connected = false;
    float minCost = std::numeric_limits<float>::max();
    for (int i = 0; i < 10000; ++i) {
//        rmpl::Vec<float> sample = plannerInitNode.getSamplePoint();
//
//        bool planner1Connected = plannerInitNode.connectGoalNode(sample);
//        bool planner2Connected = plannerGoalNode.connectGoalNode(sample);
//        if (planner1Connected && planner2Connected) {
//            float cost = plannerInitNode.getGoalNode()->getCost() + plannerGoalNode.getGoalNode()->getCost();
//            if (cost < minCost) {
//                goal = sample;
//                connected = true;
//            }
//        }
    }

    std::vector<std::shared_ptr<rmpl::Node>> nodes = plannerInitNode.getGraphNodes();
    std::vector<rmpl::Vec<float>> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getVec()));
    Writer::writeVecsToFile(graphPoints, "example.ASC", 10);

    nodes = plannerGoalNode.getGraphNodes();
    std::cout << "Goal Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getVec()));
    Writer::appendVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        plannerInitNode.connectGoalNode(goal);
        plannerGoalNode.connectGoalNode(goal);

        std::vector<rmpl::Vec<float>> pathAngles = plannerInitNode.getPath(5, true);
        std::vector<rmpl::Vec<float>> temp = plannerGoalNode.getPath(5, true);
        pathAngles.insert(pathAngles.end(), temp.begin(), temp.end());

        std::vector<rmpl::Vec<float>> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        Writer::appendVecsToFile(pathPoints, "example.ASC", 10);

        // for (int i = 0; i < pathPoints.size(); ++i)
        //    vrep->setPos(pathAngles[i]);
    }
}

int main(int argc, char** argv) {
    // treeConnection();
    simpleRRT();

    return 0;
}
