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

AABB iiwaWorkspace(Vector3(-2500, -2500, -200), Vector3(2500, 2500, 2500));
double stepSize = util::toRad(75);

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
    envConfigurator.setSerialRobotProperties(dhParameters, linkModelFiles, util::toTransform(linkOffsets));

    if (obstacleType == 0) {
        envConfigurator.addObstacle(FLAGS_obstacle3dDir + "iiwaEvaluation/backPlane.obj");
        envConfigurator.addObstacle(FLAGS_obstacle3dDir + "iiwaEvaluation/frontPlane.obj");
    } else if (obstacleType == 1) {
        envConfigurator.addObstacle(FLAGS_obstacle3dDir + "iiwaEvaluation/backPlane.obj");
        envConfigurator.addObstacle(FLAGS_obstacle3dDir + "iiwaEvaluation/shelve.obj");
    }

    ModuleConfigurator<7> creator;
    creator.setEnvironment(envConfigurator.getEnvironment());
    creator.setValidityCheckerType(ValidityCheckerType::FclSerial);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    creator.setEvaluatorProperties(stepSize, 240);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplingType(SamplingType::Straight);
    creator.setSamplerProperties(seed);
    return creator;
}

void planningThread() {
    for (size_t numNodes = 50; numNodes <= 500; numNodes += 100) {
        for (size_t threads = 1; threads <= 24; threads++) {
            std::string nodesString = std::to_string(numNodes);
            std::string threadsString = std::to_string(threads);
            std::vector<std::string> seeds = {"234`r5fdsfda", "23r54wedf",  "23894rhwef",  "092yu4re",   "0923ujrpiofesd",
                                              "02u9r3jes",    "09243rjpef", "23refdss;wf", "2-3r0wepoj", "-243refdsaf"};
            for (auto& seed : seeds) {
                for (size_t i = 0; i < 3; ++i) {
                    Vector<7> start, goal;
                    if (i == 0) {
                        start = util::toRad<7>(util::Vecd(-46, 54, 9, -81, 125, 48, 130));
                        goal = util::toRad<7>(util::Vecd(55, 46, 9, -53, 103, -25, 20));
                    } else if (i == 1) {
                        start = util::toRad<7>(util::Vecd(-46, 54, 9, -81, 125, 48, 130));
                        goal = util::toRad<7>(util::Vecd(-75, 33, 83, -60, -25, 23, 68));
                    } else {
                        start = util::toRad<7>(util::Vecd(-59, 64, 74, -116, -32, 96, 68));
                        goal = util::toRad<7>(util::Vecd(60, 71, 62, -112, -153, -105, -44));
                    }

                    Stats::initializeCollectors();
                    auto iCtr = getIiwaCreator(seed, i);
                    auto iPlanner = std::make_shared<RRTStarConnect<7>>(iCtr.getEnvironment(), iCtr.getRRTOptions(stepSize),
                                                                        iCtr.getGraph(), iCtr.getGraphB());

                    iPlanner->computePath(start, goal, numNodes, threads);
                    iPlanner->getPath();
                    ui::save("IiwaT" + threadsString + "N" + nodesString + ".json", Stats::serialize(), 4, true);
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::debug);
    planningThread();
}
