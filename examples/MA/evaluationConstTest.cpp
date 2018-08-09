#include <thread>

#include <ConfigurationMA.h>
#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../../assets/", "assets directory");
DEFINE_string(obstacle3dDir, FLAGS_assetsDir + "spaces/3D/", "obstacle directory");
DEFINE_string(obstacle2dDir, FLAGS_assetsDir + "spaces/2D/", "obstacle directory");

AABB iiwaWorkspace(Vector3(-2500, -2500, -200), Vector3(2500, 2500, 2500));
double stepSize = util::toRad(50);
bool drawImages = true;

ModuleConfigurator<7> getIiwaCreator(std::string seed, size_t obstacleType) {
    const unsigned int dim = 7;
    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(iiwaWorkspace);
    Vector<dim> minRobotBound = util::toRad<dim>(util::Vecd(-170, -120, -170, -120, -170, -120, -175));
    std::vector<DhParameter> dhParameters({DhParameter(util::toRad(-90), 0, 340), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(util::toRad(90), 0, 400), DhParameter(util::toRad(-90), 0, 0),
                                           DhParameter(util::toRad(-90), 0, 400), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(0, 0, 126)});
    std::string iiwa = FLAGS_assetsDir + "robotModels/iiwa/";
    std::vector<std::string> linkModelFiles = {iiwa + "link1.obj", iiwa + "link2.obj", iiwa + "link3.obj", iiwa + "link4.obj",
                                               iiwa + "link5.obj", iiwa + "link6.obj", iiwa + "link7.obj"};
    envConfigurator.setRobotBaseModelFile(iiwa + "link0.obj");
    envConfigurator.setRobotType(RobotType::Serial);
    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    std::vector<DofType> dofTypes(7, DofType::jointRot);
    envConfigurator.setRobotBaseProperties(dim, dofTypes, std::make_pair(minRobotBound, -minRobotBound));
    std::vector<Vector6> linkOffsets(7, util::Vecd(0, 0, 0, 0, 0, 0));
    linkOffsets[0] = util::Vecd(0, 0, 150, 0, 0, 0);
    linkOffsets[1] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    linkOffsets[2] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[3] = util::Vecd(0, 0, 0, -util::halfPi(), 0, 0);
    linkOffsets[4] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[5] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    envConfigurator.setSerialRobotProperties(dhParameters, linkModelFiles, util::toTransform(linkOffsets), Transform::Identity(),
                                             util::toTransform(util::Vecd(0, 0, 120, 0, 0, 0)), Transform::Identity(),
                                             iiwa + "handGuidingTool.obj");

    if (obstacleType == 0) {
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/maze.obj");
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/floor.obj");
    } else if (obstacleType == 1) {
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/rotationWalls.obj");
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/floor.obj");
    } else if (obstacleType == 2) {
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/backPlane.obj");
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/floor.obj");
    }

    ModuleConfigurator<7> creator;
    creator.setEnvironment(envConfigurator.getEnvironment());
    creator.setValidityCheckerType(ValidityCheckerType::FclSerialAndConstraint);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    creator.setEvaluatorProperties(stepSize, 180);
    creator.setSamplerType(SamplerType::Uniform);
    creator.setSamplingType(SamplingType::Berenson);
    creator.setPathModifierType(PathModifierType::Dummy);
    creator.setSamplerProperties(seed, 1);
    creator.setSamplingProperties(50, stepSize);

    envConfigurator.saveConfig("KukaEnvConfig.json");
    return creator;
}

void runIiwa(ModuleConfigurator<7>& iCtr, size_t type) {
    const unsigned dim = 7;
    auto env = iCtr.getEnvironment();

    Vector<dim> start;
    Vector<dim> goal;
    Vector6 Cmin;
    std::pair<Vector6, Vector6> C;
    Transform taskFrame;

    if (type == 0) {    // case 1: fixed z
        start = util::toRad<dim>(util::Vecd(135, 70, 0, -50, 0, 60, 0));
        goal = util::toRad<dim>(util::Vecd(45, 70, 0, -50, 0, 60, 0));
        Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -20, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
        C = std::make_pair(Cmin, -Cmin);
        taskFrame = util::toTransform(util::Vecd(0, 0, 50, 0, 0, 0));
    } else if (type == 1) {    // case 2: fixed rotation around x and y
        start = util::toRad<dim>(util::Vecd(-70, 90, 0, 90, -60, 0, 120));
        goal = util::toRad<dim>(util::Vecd(70, -90, 0, -90, 50, 0, -120));
        Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -util::toRad(5), -util::toRad(5), -IPPP_MAX);
        C = std::make_pair(Cmin, -Cmin);
        taskFrame = util::toTransform(util::Vecd(0, 0, 0, 0, 0, 0));
    } else if (type == 2) {    // case 3: fixed y
        start = util::toRad<dim>(util::Vecd(90, 90, 90, 45, 0, -45, 50));
        goal = util::toRad<dim>(util::Vecd(-90, 90, -90, 45, 0, -45, 110));
        Cmin = util::Vecd(-20, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
        C = std::make_pair(Cmin, -Cmin);
        taskFrame = util::toTransform(util::Vecd(528.84, 0, 0, 0, 0, 0));
    }

    iCtr.setConstraintProperties(C, taskFrame);
    auto planner = std::make_shared<RRTStarConnect<7>>(iCtr.getEnvironment(), iCtr.getRRTOptions(stepSize),
                                                       iCtr.getGraph(), iCtr.getGraphB());
    planner->computePath(start, goal, 2400, 24);
    planner->getPath();
    auto mesh = cad::mergeMeshes(util::createTcpMeshes<dim>(*iCtr.getGraph(), *env->getRobot()));
    cad::exportCad(cad::ExportFormat::OBJ, "tcps1.obj", mesh);
    mesh = cad::mergeMeshes(util::createTcpMeshes<dim>(*iCtr.getGraphB(), *env->getRobot()));
    cad::exportCad(cad::ExportFormat::OBJ, "tcps2.obj", mesh);

    if (drawImages) {
        ui::save("kukaPath.json", jsonSerializer::serialize<dim>(planner->getPath(0.001, 0.001)));

        //util::saveMeshes(*env, start, "start" + std::to_string(type));
        //util::saveMeshes(*env, goal, "goal" + std::to_string(type));
    }
}

void planningThread() {
    std::vector<std::string> seeds = {"234`r5fdsfda"};//, "23r54wedf",  "23894rhwef",  "092yu4re",   "0923ujrpiofesd",
                                      //"02u9r3jes",    "09243rjpef", "23refdss;wf", "2-3r0wepoj", "-243refdsaf"};
    for (auto& seed : seeds) {
        for (size_t i = 0; i < 1; ++i) {
            // iiwa
            Stats::initializeCollectors();
            auto iCtr = getIiwaCreator(seed, i);
            runIiwa(iCtr, i);
            ui::save("TestIiwa.json", Stats::serialize(), 4, true);
        }
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::debug);
    planningThread();
}
