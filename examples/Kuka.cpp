#include <ctime>

#include <gflags/gflags.h>

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");

void printTime(clock_t begin, clock_t end) {
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

void simpleRRT() {
    const unsigned int dim = 7;
    EnvironmentConfigurator envConfigurator;

    Vector7 minRobotBound = util::Vecd(-170, -120, -170, -120, -170, -120, -175);
    Vector7 maxRobotBound = util::Vecd(170, 120, 170, 120, 170, 120, 175);
    minRobotBound = util::degToRad<7>(minRobotBound);
    maxRobotBound = util::degToRad<7>(maxRobotBound);
    std::vector<DhParameter> dhParameters({DhParameter(util::toRad(-90), 0, 340), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(util::toRad(90), 0, 400), DhParameter(util::toRad(-90), 0, 0),
                                           DhParameter(util::toRad(-90), 0, 400), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(0, 0, 126)});
    std::vector<std::string> jointModelFiles = {
        FLAGS_assetsDir + "/robotModels/iiwa/link1.obj", FLAGS_assetsDir + "/robotModels/iiwa/link2.obj",
        FLAGS_assetsDir + "/robotModels/iiwa/link3.obj", FLAGS_assetsDir + "/robotModels/iiwa/link4.obj",
        FLAGS_assetsDir + "/robotModels/iiwa/link5.obj", FLAGS_assetsDir + "/robotModels/iiwa/link6.obj",
        FLAGS_assetsDir + "/robotModels/iiwa/link7.obj"};    // 7
    envConfigurator.setRobotBaseModelFile(FLAGS_assetsDir + "/robotModels/iiwa/link0.obj");
    envConfigurator.setRobotType(RobotType::Serial);

    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    std::vector<DofType> dofTypes = {DofType::joint, DofType::joint, DofType::joint, DofType::joint,
                                     DofType::joint, DofType::joint, DofType::joint};
    envConfigurator.setRobotBaseProperties(dim, dofTypes, std::make_pair(minRobotBound, maxRobotBound));
    std::vector<Vector6> linkOffsets(7, util::Vecd(0, 0, 0, 0, 0, 0));
    linkOffsets[0] = util::Vecd(0, 0, 150, 0, 0, 0);
    linkOffsets[1] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    linkOffsets[2] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[3] = util::Vecd(0, 0, 0, -util::halfPi(), 0, 0);
    linkOffsets[4] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[5] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    auto linkTransforms = util::convertPosesToTransforms(linkOffsets);
    envConfigurator.setSerialRobotProperties(dhParameters, jointModelFiles, Transform::Identity(), linkTransforms);
    envConfigurator.setWorkspaceProperties(AABB(Vector3(-10000, -10000, -10000), Vector3(10000, 10000, 10000)));
    envConfigurator.saveConfig("KukaEnvConfig.json");
    auto environment = envConfigurator.getEnvironment();

    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(environment->getRobot());

    //serialRobot->saveMeshConfig(util::Vecd(0, 0, 0, 0, 0, 0, 0));
    //serialRobot->saveMeshConfig(util::Vecd(0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7));
    //std::cout << "saved model" << std::endl;

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setCollisionType(CollisionType::FCL);

    RRT<dim> planner(environment, creator.getRRTOptions(30), creator.getGraph());
    Vector<dim> start = util::Vecd(0, 0, 0, 0, 0, 0, 0);
    Vector<dim> goal = util::Vecd(90, 90, 90, 90, 90, 90,90);
    start = util::degToRad<dim>(start);
    goal = util::degToRad<dim>(goal);

    // compute the tree
    clock_t begin = std::clock();
    bool connected = planner.computePath(start, goal, 200, 3);
    clock_t end = std::clock();
    printTime(begin, end);

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner.getGraphNodes();
    std::vector<Transform> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        auto path = planner.getPath(0.001, 0.001);

        auto json = jsonSerializer::serialize<dim>(path);
        ui::save("kukaPath.json", json);
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    Logging::setLogLevel(LogLevel::trace);

    simpleRRT();
    std::string string;
    std::cin >> string;

    return 0;
}
