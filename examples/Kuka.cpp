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

    envConfigurator.setWorkspaceProperties(AABB(Vector3(-3000, -3000, -5), Vector3(3000, 3000, 3000)));
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/tableOriginal.obj");
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/mannequin.obj");

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
    envConfigurator.setSerialRobotProperties(dhParameters, linkModelFiles, linkTransforms, Transform::Identity(),
            Transform::Identity(), Transform::Identity(),
            FLAGS_assetsDir + "/robotModels/carpetBeater.obj");
    envConfigurator.saveConfig("KukaEnvConfig.json");

    auto environment = envConfigurator.getEnvironment();

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
    Vector<dim> start = util::toRad<dim>(util::Vecd(-68.14, 54.24, -16.48, -19.41, -13.74, -17.03, -60.87));
    Vector<dim> goal = util::toRad<dim>(util::Vecd(-18.99, 70.54, 7.03, -57.85, -39.36, -18.57, -65.33));
    util::saveMeshes(*environment, goal, "test");

    if (planner.computePath(start, goal, 240, 24)) {
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
    //std::string string;
    //std::cin >> string;

    return 0;
}
