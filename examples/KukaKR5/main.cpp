#include <chrono>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <core/utility/Logging.h>
#include <core/utility/Utility.h>
#include <core/module/collisionDetection/CollisionDetectionPqp.hpp>
#include <pathPlanner/RRTStar.hpp>
#include <environment/KukaKR5.h>

#include <ui/Writer.hpp>

using namespace ippp;

int main(int argc, char** argv) {
    Logging::setOutputFile("output.txt");
    Logging::setLogOutput(LogOutput::terminlAndFile);

    std::shared_ptr<KukaKR5> robot(new KukaKR5());
    std::shared_ptr<CollisionDetection<6>> collision(new CollisionDetectionPqp<6>(robot));
    std::shared_ptr<TrajectoryPlanner<6>> trajectory(new TrajectoryPlanner<6>(collision));
    std::shared_ptr<Sampler<6>> sampler(new Sampler<6>(robot));
    std::shared_ptr<Sampling<6>> sampling(new Sampling<6>(robot, collision, trajectory, sampler));
    RRTOptions<6> options(40, collision, trajectory, sampling);
    RRTStar<6> planner(robot, options);

    Vector6 start = util::Vecf(0, 0, 0, 0, 51, 0);
    robot->saveMeshConfig(start);
    Vector6 goal = util::Vecf(150, -60, 90, 0, 82.7, 75.5);

    auto startTime = std::chrono::system_clock::now();
    bool connected = planner.computePath(start, goal, 25000, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    std::vector<std::shared_ptr<Node<6>>> nodes = planner.getGraphNodes();
    std::vector<Vector6> graphPoints;
    Logging::info("Init Graph has: " + std::to_string(nodes.size()) + "nodes", "Example");
    for (auto node : nodes)
        graphPoints.push_back(robot->directKinematic(node->getValues()));
    writer::writeVecsToFile<6>(graphPoints, "example.ASC", 10);

    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vector6> pathAngles = planner.getPath(1, true);

        std::vector<std::vector<Vector6>> vecs;
        for (auto angle : pathAngles) {
            std::vector<Matrix4> jointTrafos = robot->getJointTrafos(angle);
            std::vector<Vector6> tmp;
            for (auto joint : jointTrafos)
                tmp.push_back(util::poseMatToVec(joint));
            vecs.push_back(tmp);
        }
        writer::writeTrafosToFile(vecs, "trafos.txt");

        std::vector<Vector6> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        writer::appendVecsToFile<6>(pathPoints, "example.ASC", 10);
    } else {
        Logging::warning("Init and goal could NOT be connected!", "Example");
    }

    return 0;
}
