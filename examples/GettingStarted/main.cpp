#include <memory>

#include <core/collisionDetection/CollisionDetectionPqp.hpp>
#include <environment/Environment.h>
#include <environment/model/ModelFactoryPqp.h>
#include <environment/robot/MobileRobot.h>
#include <pathPlanner/PRM.hpp>
#include <pathPlanner/RRTStar.hpp>

#include <modelDirectory.h>

// set namespace of the motion planner lib
using namespace ippp;

int main(int argc, char** argv) {
    // specify the dimension for the motion planning
    const unsigned int dim = 6;

    // get path to the model directory
    std::string modelDir = getModelDirectory();

    // create a ModelFactory and the models of robot and obstacle (workspace)
    ModelFactoryPqp factoryPqp;
    std::shared_ptr<ModelContainer> robotModel = factoryPqp.createModel(modelDir + "parasol_benchmarks/alpha1.5/robot.obj");
    std::shared_ptr<ModelContainer> obstacleModel = factoryPqp.createModel(modelDir + "parasol_benchmarks/alpha1.5/obstacle.obj");

    // specify the boundaries of the robot
    VectorX minBoundary = util::Vecf(-200, -200, -200, 0, 0, 0);
    VectorX maxBoundary = util::Vecf(200, 200, 200, util::twoPi(), util::twoPi(), util::twoPi());

    // create mobile robot and set the models
    std::shared_ptr<RobotBase> robot(new MobileRobot(2, minBoundary, maxBoundary));
    robot->setBaseModel(robotModel);
    std::shared_ptr<Environment> environment(new Environment(3, AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    environment->addObstacle(obstacleModel);

    // create the modules of the planner
    // as first create the collision detection
    std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetectionPqp<dim>(environment));
    // define step size of the trajectories and create trajectory planner
    float stepSize = 3;
    std::shared_ptr<TrajectoryPlanner<dim>> trajectory(new TrajectoryPlanner<dim>(collision, stepSize));
    // create the sampling module with the sampler
    std::shared_ptr<Sampler<dim>> sampler(new Sampler<dim>(environment));
    std::shared_ptr<Sampling<dim>> sampling(new Sampling<dim>(environment, collision, trajectory, sampler));
    // create the distance metric
    std::shared_ptr<DistanceMetric<dim>> distanceMetric(new DistanceMetric<dim>());

    // define the options of the path planner with the modules
    float rrtStepSize = 30;
    RRTOptions<6> options(rrtStepSize, collision, trajectory, sampling, distanceMetric);
    // create the neighbor finder for the graph
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> neighborFinder(
        new KDTree<dim, std::shared_ptr<Node<dim>>>(distanceMetric));
    // create the graph of the planner
    std::shared_ptr<Graph<dim>> graph(new Graph<dim>(0, neighborFinder));
    // create the path planner
    RRTStar<6> pathPlanner(environment, options, graph);

    // define start and goal position, angles has to be in rad
    Vector<dim> start = util::Vecf(78.240253, 24.147785, -8.133371, 0.286451, 0.769112, 0.706202);
    Vector<dim> goal = util::Vecf(-1.393717, 14.630385, 0.415858, 0.713232, 0.156663, 0.826461);

    // define number of nodes and threads for the path planning
    unsigned int numNodes = 8000;
    unsigned int numThreads = 4;
    // compute path
    bool result = pathPlanner.computePath(start, goal, numNodes, numThreads);

    if (result) {
        std::vector<Vector<dim>> path = pathPlanner.getPath(1, true);
    }

    return 0;
}
