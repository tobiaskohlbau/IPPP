#include <chrono>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <core/utility/Logging.h>
#include <core/utility/Utility.h>
#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/KukaKR5.h>

#include <core/utility/Utility.h>
#include <ui/Writer.h>

using namespace rmpl;

int main(int argc, char** argv) {
    Logging::setOutputFile("output.txt");
    Logging::setLogOutput(LogOutput::terminlAndFile);

    std::shared_ptr<KukaKR5> robot(new KukaKR5());
    RRTOptions options(40, 1);
    NormalRRTPlanner planner(robot, options);

    Eigen::Matrix<float, 6 ,1> start = Vecf(0, 0, 0, 0, 51, 0);
    robot->saveMeshConfig(start);
    Eigen::Matrix<float, 6 ,1> goal = Vecf(150, -60, 90, 0, 82.7, 75.5);

    auto startTime = std::chrono::system_clock::now();
    bool connected = planner.computePath(start, goal, 25000, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    std::vector<std::shared_ptr<Node>> nodes = planner.getGraphNodes();
    std::vector<Eigen::VectorXf> graphPoints;
    Logging::info("Init Graph has: " + std::to_string(nodes.size()) + "nodes", "Example");
    for (auto node : nodes)
        graphPoints.push_back(robot->directKinematic(node->getValues()));
    Writer::writeVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Eigen::VectorXf> pathAngles = planner.getPath(0.3, true);

        std::vector<std::vector<Eigen::VectorXf>> vecs;
        for (auto angle : pathAngles) {
            std::vector<Eigen::Matrix4f> jointTrafos = robot->getJointTrafos(angle);
            std::vector<Eigen::VectorXf> tmp;
            for (auto joint : jointTrafos)
                tmp.push_back(utility::poseMatToVec(joint));
            vecs.push_back(tmp);
        }
        Writer::writeTrafosToFile(vecs, "trafos.txt");

        std::vector<Eigen::VectorXf> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        Writer::appendVecsToFile(pathPoints, "example.ASC", 10);
    } else {
        Logging::warning("Init and goal could NOT be connected!", "Example");
    }

    return 0;
}
