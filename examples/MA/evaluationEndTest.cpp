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

ModuleConfigurator<7> getIiwaCreator(std::string seed) {
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
                                             util::toTransform(util::Vecd(0, 0, 205, 0, 0, 0)), Transform::Identity(),
                                             FLAGS_assetsDir + "/robotModels/rhmiGripper.obj");
    envConfigurator.addObstacle(FLAGS_obstacle3dDir + "tableOriginal.obj");
    envConfigurator.addObstacle(FLAGS_obstacle3dDir + "/iiwaEvaluation/shelves.obj");
    envConfigurator.saveConfig("KukaEnvConfig.json");

    ModuleConfigurator<7> creator;
    creator.setEnvironment(envConfigurator.getEnvironment());
    creator.setValidityCheckerType(ValidityCheckerType::FclSerial);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    creator.setEvaluatorProperties(util::toRad(60), 45);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplingType(SamplingType::Straight);
    creator.setSamplerProperties(seed, 1);
    return creator;
}

void planningThread() {
    std::vector<std::string> seeds = {"234`r5fdsfda", "23r54wedf",  "23894rhwef",  "092yu4re",   "0923ujrpiofesd",
                                      "02u9r3jes",    "09243rjpef", "23refdss;wf", "2-3r0wepoj", "-243refdsaf"};
    for (auto& seed : seeds) {
        Vector<7> vec[6];
        vec[0] = util::Vecd(0, 0, 0, 0, 0, 0, 0);
        vec[1] = util::toRad<7>(util::Vecd(-15.54, 103.18, 90.12, -72.3, -12.4, 73.02, -1.41));
        vec[2] = util::toRad<7>(util::Vecd(-1.54, 84.24, 120.0, -61.67, -11.47, 76.17, 30.92));
        vec[3] = util::toRad<7>(util::Vecd(17.26, 67.21, -6.02, -80.56, -84.32, 89.59, 63.82));
        vec[4] = util::toRad<7>(util::Vecd(10.84, 6.45, 1.11, -116.13, -88.17, 88.08, 31.11));
        vec[5] = util::Vecd(0, 0, 0, 0, 0, 0, 0);
        std::vector<Vector<7>> path;

        for (size_t i = 0; i < 5; ++i) {
            Stats::initializeCollectors();
            auto iCtr = getIiwaCreator(seed);
            auto iPlanner = std::make_shared<RRTStarConnect<7>>(
                iCtr.getEnvironment(), iCtr.getRRTOptions(util::toRad(90)), iCtr.getGraph(), iCtr.getGraphB());
            iPlanner->computePath(vec[i], vec[i + 1], 240, 24);
            //util::saveMeshes(*iCtr.getEnvironment(), vec[i], "vec" + std::to_string(i));
            auto tmpPath = iPlanner->getPath(1, util::toRad(1));
            if (!tmpPath.empty())
                path.insert(path.end(), tmpPath.begin(), tmpPath.end());
            ui::save("IiwaMain.json", Stats::serialize(), 4, true);
        }

        ui::save("kukaPath.json", jsonSerializer::serialize<7>(path));
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::info);
    planningThread();
}
