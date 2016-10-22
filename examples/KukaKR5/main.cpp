#include <ctime>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <core/Logging.h>
#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/KukaKR5.h>

#include <ui/Drawing.h>

using namespace rmpl;

void printTime(clock_t begin, clock_t end) {
    float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

int main(int argc, char** argv) {
    std::shared_ptr<rmpl::KukaKR5> robot(new rmpl::KukaKR5());
    std::shared_ptr<rmpl::RRTOptions> options(new rmpl::RRTOptions(30, 1, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly));
    NormalRRTPlanner planner(robot, options);

    rmpl::Vec<float> start(0, 90, 0, 0, 270, -180);
    rmpl::Vec<float> goal(150, -60, 90, 0, 82.7, 75.5);

    clock_t begin = std::clock();
    bool connected = planner.computePath(start, goal, 25000, 2);
    clock_t end = std::clock();
    printTime(begin, end);

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();
    std::vector<rmpl::Vec<float>> graphPoints;
    Logging::info("Init Graph has: " + std::to_string(nodes.size()) + "nodes", "Example");
    for (auto node : nodes)
        graphPoints.push_back(robot->directKinematic(node->getVec()));
    Drawing::writeVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<rmpl::Vec<float>> pathAngles = planner.getPath(0.3, true);

        std::vector<rmpl::Vec<float>> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        Drawing::appendVecsToFile(pathPoints, "example.ASC", 10);
    } else {
        Logging::warning("Init and goal could NOT be connected!", "Example");
    }

    return 0;
}

