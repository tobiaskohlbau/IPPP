#include <memory>

#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

// set namespace of the motion planner lib
using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // specify the dimension for the motion planning
    const unsigned int dim = 6;

    // create a ModelFactory and the models of robot and obstacle (workspace)
    ModelFactoryPqp factoryPqp;
    std::shared_ptr<ModelContainer> robotModel =
        factoryPqp.createModelFromFile(FLAGS_assetsDir + "/parasol_benchmarks/alpha1.5/robot.obj");
    std::shared_ptr<ModelContainer> obstacleModel =
        factoryPqp.createModelFromFile(FLAGS_assetsDir + "/parasol_benchmarks/alpha1.5/obstacle.obj");

    // specify the boundaries of the robot
    VectorX minBoundary = util::Vecd(-200, -200, -200, 0, 0, 0);
    VectorX maxBoundary = util::Vecd(200, 200, 200, util::twoPi(), util::twoPi(), util::twoPi());

    // create mobile robot and set the models
    std::vector<DofType> dofTypes = {DofType::volumetricPos, DofType::volumetricPos, DofType::volumetricPos,
                                     DofType::volumetricRot, DofType::volumetricRot, DofType::volumetricRot};
    std::shared_ptr<RobotBase> robot(new MobileRobot(2, std::make_pair(minBoundary, maxBoundary), dofTypes));
    robot->setBaseModel(robotModel);
    std::shared_ptr<Environment> environment(new Environment(AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    environment->addEnvObject(std::make_shared<ObstacleObject>("obstacle", obstacleModel));

    // define step size of the trajectories and create trajectory planner
    double stepSize = 3;
    // create all required core modules with the ModuleConfigurator
    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setCollisionType(CollisionType::PQP);
    creator.setMetricType(MetricType::L2);
    creator.setPathModifierType(PathModifierType::NodeCut);
    creator.setSamplerType(SamplerType::SamplerUniform);
    creator.setSamplingType(SamplingType::Straight);
    creator.setSamplingProperties(5, 10);
    creator.setTrajectoryType(TrajectoryType::Linear);

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
