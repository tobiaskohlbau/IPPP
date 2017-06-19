#include <ctime>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <ippp/Core>
#include <ippp/Environment>
#include <ippp/Planner>

#include <ui/ModuleCreator.hpp>
#include <ui/Writer.hpp>

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

    std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetectionPqp<dim>(environment));
    ModuleCreator<dim> creator(environment, collision, MetricType::L2, NeighborType::KDTree, PathModifierType::NodeCut,
                               SamplerType::SamplerUniform, SamplingType::Straight, TrajectoryType::Linear, 0.5);

    RRT<dim> planner(environment, creator.getRRTOptions(30), creator.getGraph());
    Vector6 start = util::Vecd(180, 180, 180, 180, 180, 180);
    Vector6 goal = util::Vecd(275, 167.5, 57.4, 241, 82.7, 75.5);

    // compute the tree
    clock_t begin = std::clock();
    bool connected = planner.computePath(start, goal, 4000, 6);
    clock_t end = std::clock();
    printTime(begin, end);

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner.getGraphNodes();
    std::vector<Vector6> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(nodes[i]->getValues()));
    writer::writeVecsToFile<dim>(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        std::vector<Vector6> pathAngles = planner.getPath(5);

        std::vector<Vector6> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(angles));
        writer::appendVecsToFile<dim>(pathPoints, "example.ASC", 10);

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
    std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetectionPqp<dim>(environment));
    ModuleCreator<dim> creator1(environment, collision, MetricType::L2, NeighborType::KDTree, PathModifierType::NodeCut,
                                SamplerType::SamplerUniform, SamplingType::Straight, TrajectoryType::Linear, 0.5);
    ModuleCreator<dim> creator2(environment, collision, MetricType::L2, NeighborType::KDTree, PathModifierType::NodeCut,
                                SamplerType::SamplerUniform, SamplingType::Straight, TrajectoryType::Linear, 0.5);

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
    std::vector<Vector6> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(nodes[i]->getValues()));
    writer::writeVecsToFile<dim>(graphPoints, "example.ASC", 10);

    nodes = plannerGoalNode.getGraphNodes();
    std::cout << "Goal Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(nodes[i]->getValues()));
    writer::appendVecsToFile<dim>(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        plannerInitNode.connectGoalNode(goal);
        plannerGoalNode.connectGoalNode(goal);

        std::vector<Vector6> pathAngles = plannerInitNode.getPath(5);
        std::vector<Vector6> temp = plannerGoalNode.getPath(5);
        pathAngles.insert(pathAngles.end(), temp.begin(), temp.end());

        std::vector<Vector6> pathPoints;
        for (auto angles : pathAngles)
            pathPoints.push_back(std::dynamic_pointer_cast<Jaco>(robot)->directKinematic(angles));
        writer::appendVecsToFile<dim>(pathPoints, "example.ASC", 10);

        // for (int i = 0; i < pathPoints.size(); ++i)
        //    vrep->setPos(pathAngles[i]);
    }
}

int main(int argc, char** argv) {
    // treeConnection();
    simpleRRT();

    return 0;
}
