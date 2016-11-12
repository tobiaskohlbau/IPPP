#include <ctime>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <pathPlanner/PRMPlanner.h>
#include <robot/Jaco.h>
#include <ui/vrep/Helper.h>

#include <ui/Writer.h>

void printTime(clock_t begin, clock_t end) {
    float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

int main(int argc, char** argv) {
    std::shared_ptr<rmpl::Jaco> robot(new rmpl::Jaco());

    std::shared_ptr<rmpl::PRMOptions> options(new rmpl::PRMOptions(30, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly));
    rmpl::PRMPlanner planner(robot, options);

    clock_t begin = std::clock();
    planner.startSamplingPhase(21000, 2);
    planner.startPlannerPhase(2);
    clock_t end = std::clock();
    printTime(begin, end);

    bool connected = planner.queryPath(rmpl::Vec<float>(180, 180, 180, 180, 180, 180),
                                       rmpl::Vec<float>(275, 167.5, 57.4, 241, 82.7, 75.5));

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

    return 0;
}
