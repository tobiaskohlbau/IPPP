#include <chrono>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <core/Logging.h>
#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/KukaKR5.h>

#include <ui/Writer.h>
#include <core/Utilities.h>

using namespace rmpl;

int main(int argc, char** argv) {
    std::shared_ptr<rmpl::KukaKR5> robot(new rmpl::KukaKR5());
    std::shared_ptr<rmpl::RRTOptions> options(
        new rmpl::RRTOptions(40, 1, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly));
    NormalRRTPlanner planner(robot, options);

    rmpl::Vec<float> start(0, 0, 0, 0, 51, 0);
    robot->saveMeshConfig(start);
    rmpl::Vec<float> goal(150, -60, 90, 0, 82.7, 75.5);

    auto startTime = std::chrono::system_clock::now();
    bool connected = planner.computePath(start, goal, 25000, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();
    std::vector<rmpl::Vec<float>> graphPoints;
    Logging::info("Init Graph has: " + std::to_string(nodes.size()) + "nodes", "Example");
    for (auto node : nodes)
        graphPoints.push_back(robot->directKinematic(node->getVec()));
    Writer::writeVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<rmpl::Vec<float>> pathAngles = planner.getPath(0.3, true);

        std::vector<std::vector<rmpl::Vec<float>>> vecs;
        for (auto angle : pathAngles) {
            std::vector<Eigen::Matrix4f> jointTrafos = robot->getJointTrafos(angle);
            std::vector<rmpl::Vec<float>> tmp;
            for (auto joint : jointTrafos)
                tmp.push_back(rmpl::Utilities::poseMatToVec(joint));
            vecs.push_back(tmp);
        }
        Writer::writeTrafosToFile(vecs, "trafos.txt");

        std::vector<rmpl::Vec<float>> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        Writer::appendVecsToFile(pathPoints, "example.ASC", 10);
    } else {
        Logging::warning("Init and goal could NOT be connected!", "Example");
    }

    return 0;
}
