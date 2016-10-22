#include <ctime>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/KukaKR5.h>

#include <ui/Drawing.h>

void printTime(clock_t begin, clock_t end) {
    float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

int main(int argc, char** argv) {
    std::shared_ptr<rmpl::KukaKR5> robot(new rmpl::KukaKR5());
    std::shared_ptr<rmpl::RRTOptions> options(new rmpl::RRTOptions(30, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly));
    rmpl::NormalRRTPlanner planner(robot, options);

    rmpl::Vec<float> start(0, 90, 0, 0, 270, -180);
    rmpl::Vec<float> goal(275, 167.5, 57.4, 241, 82.7, 75.5);

    // compute the tree
    clock_t begin = std::clock();
    bool connected = planner.computePath(start, goal, 15000, 2);
    clock_t end = std::clock();
    printTime(begin, end);
    robot->saveMeshConfig(goal);

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();
    std::vector<rmpl::Vec<float>> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getVec()));
    Drawing::writeVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        std::vector<rmpl::Vec<float>> pathAngles = planner.getPath(5, true);

        std::vector<rmpl::Vec<float>> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        Drawing::appendVecsToFile(pathPoints, "example.ASC", 10);
    }

    return 0;
}

