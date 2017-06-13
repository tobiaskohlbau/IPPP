#include <memory>

#include <Core>
#include <Environment>
#include <Planner>

#include <modelDirectory.h>
#include <ui/ModuleCreator.hpp>

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
    VectorX minBoundary = util::Vecd(-200, -200, -200, 0, 0, 0);
    VectorX maxBoundary = util::Vecd(200, 200, 200, util::twoPi(), util::twoPi(), util::twoPi());

    // create mobile robot and set the models
    std::vector<DofType> dofTypes = {DofType::volumetricPos, DofType::volumetricPos, DofType::volumetricPos,
                                     DofType::volumetricRot, DofType::volumetricRot, DofType::volumetricRot};
    std::shared_ptr<RobotBase> robot(new MobileRobot(2, std::make_pair(minBoundary, maxBoundary), dofTypes));
    robot->setBaseModel(robotModel);
    std::shared_ptr<Environment> environment(new Environment(3, AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    environment->addObstacle(obstacleModel);

    // create the modules of the planner
    // as first create the collision detection
    std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetectionPqp<dim>(environment));
    // define step size of the trajectories and create trajectory planner
    double stepSize = 3;
    // create all required core modules with the ModuleCreator
    ModuleCreator<dim> creator(environment, collision, MetricType::L2, NeighborType::KDTree, PathModifierType::NodeCut,
                               SamplerType::SamplerUniform, SamplingType::Straight, TrajectoryType::Linear, stepSize);

    // define the options of the path planner
    double rrtStepSize = 30;
    // create the path planner
    RRTStar<6> pathPlanner(environment, creator.getRRTOptions(rrtStepSize), creator.getGraph());

    // define start and goal position, angles has to be in rad
    Vector<dim> start = util::Vecd(78.240253, 24.147785, -8.133371, 0.286451, 0.769112, 0.706202);
    Vector<dim> goal = util::Vecd(-1.393717, 14.630385, 0.415858, 0.713232, 0.156663, 0.826461);

    // define number of nodes and threads for the path planning
    unsigned int numNodes = 8000;
    unsigned int numThreads = 4;
    // compute path
    bool result = pathPlanner.computePath(start, goal, numNodes, numThreads);

    if (result) {
        std::vector<Vector<dim>> path = pathPlanner.getPath();
    }

    return 0;
}
