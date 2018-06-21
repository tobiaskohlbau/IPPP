#include <ctime>

#include <gflags/gflags.h>

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");

void simpleRRT() {
    const unsigned int dim = 7;
    EnvironmentConfigurator envConfigurator;

    envConfigurator.setWorkspaceProperties(AABB(Vector3(-1000, -1000, -5), Vector3(1000, 1000, 2000)));
    for (double deg = -157.5; deg < 180; deg += 45) {
        double angle = util::toRad(deg);
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/plane.obj",
                                    util::Vecd(std::cos(angle) * 550, std::sin(angle) * 550, 320, 0, 0, angle));
    }
    // envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/obstacle400x400x800.obj",
    //                            util::Vecd(420, -400, 100, 0, 0, util::toRad(90)));

    Vector7 minRobotBound = util::Vecd(-170, -120, -170, -120, -170, -120, -175);
    Vector7 maxRobotBound = util::Vecd(170, 120, 170, 120, 170, 120, 175);
    minRobotBound = util::toRad<7>(minRobotBound);
    maxRobotBound = util::toRad<7>(maxRobotBound);
    std::vector<DhParameter> dhParameters({DhParameter(util::toRad(-90), 0, 340), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(util::toRad(90), 0, 400), DhParameter(util::toRad(-90), 0, 0),
                                           DhParameter(util::toRad(-90), 0, 400), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(0, 0, 126)});
    std::string iiwa = FLAGS_assetsDir + "/robotModels/iiwa/";
    std::vector<std::string> linkModelFiles = {iiwa + "link1.obj", iiwa + "link2.obj", iiwa + "link3.obj", iiwa + "link4.obj",
                                               iiwa + "link5.obj", iiwa + "link6.obj", iiwa + "link7.obj"};
    envConfigurator.setRobotBaseModelFile(iiwa + "link0.obj");
    envConfigurator.setRobotType(RobotType::Serial);

    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    std::vector<DofType> dofTypes(7, DofType::jointRot);
    envConfigurator.setRobotBaseProperties(dim, dofTypes, std::make_pair(minRobotBound, maxRobotBound));
    std::vector<Vector6> linkOffsets(7, util::Vecd(0, 0, 0, 0, 0, 0));
    linkOffsets[0] = util::Vecd(0, 0, 150, 0, 0, 0);
    linkOffsets[1] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    linkOffsets[2] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[3] = util::Vecd(0, 0, 0, -util::halfPi(), 0, 0);
    linkOffsets[4] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[5] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    auto linkTransforms = util::toTransform(linkOffsets);
    envConfigurator.setSerialRobotProperties(dhParameters, linkModelFiles, linkTransforms);    //, Transform::Identity(),
    //        util::toTransform(util::Vecd(0, 0, 205, 0, 0, 0)), util::toTransform(util::Vecd(0, 0, 205, 0, 0, 0)),
    //        FLAGS_assetsDir + "/robotModels/wesslingHand.obj");

    envConfigurator.saveConfig("KukaEnvConfig.json");

    auto environment = envConfigurator.getEnvironment();
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(environment->getRobot());

    Vector<dim> a = util::Vecd(-46, 54, 9, -81, 125, 48, 130);
    Vector<dim> b = util::Vecd(55, 46, 9, -53, 103, -25, 20);
    Vector<dim> c = util::Vecd(-75, 33, 83, -60, -25, 23, 68);
    Vector<dim> d = util::Vecd(-59, 64, 74, -116, -32, 96, 68);
    Vector<dim> e = util::Vecd(60, 71, 62, -112, -153, -105, -44);
    a = util::toRad<dim>(a);
    b = util::toRad<dim>(b);
    c = util::toRad<dim>(c);
    d = util::toRad<dim>(d);
    e = util::toRad<dim>(e);
    // Vector<dim> testConfig = util::Vecd(-90, 90, 170, 30, 90, 90, 30);
    // Vector<dim> testConfig = util::Vecd(45, 90, 170, 30, 10, 120, 0);
    // Vector<dim> testConfig = util::Vecd(0, 0, 0, 0, 0, 0, 0);
    // testConfig = util::toRad<dim>(testConfig);
    util::saveMeshes(*environment, a);

    double stepSize = util::toRad(70);
    ModuleConfigurator<dim> creator;
    creator.setEvaluatorType(EvaluatorType::TreeConnect);
    creator.setEvaluatorProperties(stepSize, 500);
    creator.setGraphSortCount(3000);
    creator.setEnvironment(environment);
    creator.setValidityCheckerType(ValidityCheckerType::FclSerial);
    creator.setSamplingType(SamplingType::NearObstacle);
    creator.setTrajectoryProperties(10, util::toRad(2));

    RRTStarConnect<dim> planner(environment, creator.getRRTOptions(stepSize), creator.getGraph(), creator.getGraphB());
    Vector<dim> start = util::Vecd(0, 0, 0, 0, 0, 0, 0);
    Vector<dim> goal = util::Vecd(-90, 90, 150, 30, 35, 114, 0);
    start = util::toRad<dim>(start);
    goal = util::toRad<dim>(goal);

    if (planner.computePath(a, b, 600, 24)) {
        std::cout << "Init and goal could be connected!" << std::endl;
        auto path = planner.getPath(0.001, 0.001);

        auto json = jsonSerializer::serialize<dim>(path);
        ui::save("kukaPath.json", json);
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    Logging::setLogLevel(LogLevel::debug);

    simpleRRT();
    Stats::writeData(std::cout);
    std::string string;
    std::cin >> string;

    return 0;
}
