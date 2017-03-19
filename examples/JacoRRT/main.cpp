#include <ctime>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <core/utility/Utility.h>
#include <core/module/collisionDetection/CollisionDetectionPqp.hpp>
#include <pathPlanner/NormalRRTPlanner.hpp>
#include <pathPlanner/RRTStarPlanner.hpp>
#include <robot/Jaco.h>
#include <ui/vrep/Helper.h>

#include <ui/Writer.hpp>

using namespace rmpl;

void printTime(clock_t begin, clock_t end) {
    float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

void simpleRRT() {
    std::shared_ptr<Jaco> robot(new Jaco());

    robot->saveMeshConfig(util::Vecf(0, 0, 0, 0, 0, 0));

    std::shared_ptr<CollisionDetection<6>> collision(new CollisionDetectionPqp<6>(robot));
    std::shared_ptr<TrajectoryPlanner<6>> trajectory(new TrajectoryPlanner<6>(0.5, collision));
    std::shared_ptr<Sampler<6>> sampler(new Sampler<6>(robot));
    std::shared_ptr<Sampling<6>> sampling(new Sampling<6>(robot, collision, trajectory, sampler));
    RRTOptions<6> options(30, collision, trajectory, sampling);
    NormalRRTPlanner<6> planner(robot, options);
    Vector6 start = util::Vecf(180, 180, 180, 180, 180, 180);
    Vector6 goal = util::Vecf(275, 167.5, 57.4, 241, 82.7, 75.5);

    // compute the tree
    clock_t begin = std::clock();
    bool connected = planner.computePath(start, goal, 4000, 6);
    clock_t end = std::clock();
    printTime(begin, end);

    std::vector<std::shared_ptr<Node<6>>> nodes = planner.getGraphNodes();
    std::vector<Vector6> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getValues()));
    writer::writeVecsToFile<6>(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        std::vector<Vector6> pathAngles = planner.getPath(5, true);

        std::vector<Vector6> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        writer::appendVecsToFile<6>(pathPoints, "example.ASC", 10);

        Helper vrep(6);
        vrep.start();
        for (auto angles : pathAngles)
            vrep.setPos(angles);
    }
}

void treeConnection() {
    std::shared_ptr<Jaco> robot(new Jaco());

    // create two trees from init and from goal
    std::shared_ptr<CollisionDetection<6>> collision(new CollisionDetectionPqp<6>(robot));
    std::shared_ptr<TrajectoryPlanner<6>> trajectory(new TrajectoryPlanner<6>(0.5, collision));
    std::shared_ptr<Sampler<6>> sampler(new Sampler<6>(robot));
    std::shared_ptr<Sampling<6>> sampling(new Sampling<6>(robot, collision, trajectory, sampler));
    RRTOptions<6> options(20, collision, trajectory, sampling);
    RRTStarPlanner<6> plannerGoalNode(robot, options);
    RRTStarPlanner<6> plannerInitNode(robot, options);

    // set properties to the planners
    plannerInitNode.setInitNode(util::Vecf(180, 180, 180, 180, 180, 180));
    plannerGoalNode.setInitNode(util::Vecf(275, 167.5, 57.4, 241, 82.7, 75.5));

    // compute the tree
    clock_t begin = std::clock();
    plannerInitNode.computeTree(20000, 2);
    plannerGoalNode.computeTree(20000, 2);
    clock_t end = std::clock();
    printTime(begin, end);

    // get random sample from the first pathPlanner and try to connect to both planners
    Eigen::VectorXf goal;
    bool connected = false;
    float minCost = std::numeric_limits<float>::max();
    for (int i = 0; i < 10000; ++i) {
//        Vec<float> sample = plannerInitNode.getSamplePoint();
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

    std::vector<std::shared_ptr<Node<6>>> nodes = plannerInitNode.getGraphNodes();
    std::vector<Vector6> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getValues()));
    writer::writeVecsToFile<6>(graphPoints, "example.ASC", 10);

    nodes = plannerGoalNode.getGraphNodes();
    std::cout << "Goal Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getValues()));
    writer::appendVecsToFile<6>(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        plannerInitNode.connectGoalNode(goal);
        plannerGoalNode.connectGoalNode(goal);

        std::vector<Vector6> pathAngles = plannerInitNode.getPath(5, true);
        std::vector<Vector6> temp = plannerGoalNode.getPath(5, true);
        pathAngles.insert(pathAngles.end(), temp.begin(), temp.end());

        std::vector<Vector6> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(robot->directKinematic(angles));
        writer::appendVecsToFile<6>(pathPoints, "example.ASC", 10);

        // for (int i = 0; i < pathPoints.size(); ++i)
        //    vrep->setPos(pathAngles[i]);
    }
}

int main(int argc, char** argv) {
    // treeConnection();
    simpleRRT();

    return 0;
}
