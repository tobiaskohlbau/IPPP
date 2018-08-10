#include <thread>

#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../../IPPP_assets/", "assets directory");
DEFINE_string(obstacle3dDir, FLAGS_assetsDir + "spaces/3D/", "obstacle directory");

AABB iiwaWorkspace(Vector3(-2500, -2500, -200), Vector3(2500, 2500, 2500));
bool drawImages = true;

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

    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/bottleEnv.obj");
    envConfigurator.saveConfig("KukaEnvConfig.json");

    ModuleConfigurator<7> creator;
    creator.setEnvironment(envConfigurator.getEnvironment());
    creator.setValidityCheckerType(ValidityCheckerType::FclSerialAndConstraint);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    creator.setEvaluatorProperties(util::toRad(90), 180);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplingType(SamplingType::Berenson);
    creator.setSamplerProperties(seed, 1);
    creator.setSamplingProperties(50, util::toRad(90));
    return creator;
}

std::vector<Vector<7>> runIiwa(ModuleConfigurator<7>& iCtr, Vector<7> start, Vector<7> goal) {
    const unsigned dim = 7;
    auto env = iCtr.getEnvironment();

    Vector6 Cmin;
    std::pair<Vector6, Vector6> C;
    Transform taskFrame;
    double epsOri = util::toRad(4);

    Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -epsOri, -epsOri, -IPPP_MAX);
    C = std::make_pair(Cmin, -Cmin);
    taskFrame = util::toTransform(util::Vecd(0, 0, 0, 0, 0, 0));

    iCtr.setConstraintProperties(C, taskFrame);
    auto planner = std::make_shared<RRTStarConnect<7>>(iCtr.getEnvironment(), iCtr.getRRTOptions(util::toRad(90)),
                                                       iCtr.getGraph(), iCtr.getGraphB());
    planner->computePath(start, goal, 480, 24);
    return planner->getPath();
}

void planningThread() {
    std::vector<std::string> seeds = {"234`r5fdsfda", "23r54wedf",  "23894rhwef",  "092yu4re",   "0923ujrpiofesd",
                                      "02u9r3jes",    "09243rjpef", "23refdss;wf", "2-3r0wepoj", "-243refdsaf"};
    for (auto& seed : seeds) {
        Vector<7> vec[5];
        vec[0] = util::toRad<7>(util::Vecd(5.81, -65.23, 24.92, -19.37, -31.01, 48.07, -3.24));
        vec[1] = util::toRad<7>(util::Vecd(106.68, -65.3, -11.96, -17.75, -166.16, -49.94, -35.80));
        vec[2] = util::toRad<7>(util::Vecd(21.97, 77.93, -7.23, 50.07, 14.27, -30.37, -149.95));
        vec[3] = util::toRad<7>(util::Vecd(5.81, -65.23, 24.92, -19.37, -31.01, 48.07, -3.24));
        //vec[0] = util::toRad<7>(util::Vecd(40.02, 92.09, -3.43, 90.27, 127.28, -4.92, 6.36));
        //vec[1] = util::toRad<7>(util::Vecd(-142.71, 69.46, 51.79, 43.58, 99.13, 48.75, 59.65));
        //vec[2] = util::toRad<7>(util::Vecd(-18.79, 89.47, 1.46, 72.04, -11.15, -17.13, 68.98));
        //vec[3] = util::toRad<7>(util::Vecd(9.68, 64.24, -10.29, 7.32, -165.63, 58.07,127.56));
        //vec[4] = util::toRad<7>(util::Vecd(40.02, 92.09, -3.43, 90.27, 127.28, -4.92, 6.36));
        std::vector<Vector<7>> path;

        for (size_t i = 0; i < 3; ++i) {
            auto iCtr = getIiwaCreator(seed);
            util::saveMeshes(*iCtr.getEnvironment(), vec[i], "vec" + std::to_string(i));
            Stats::initializeCollectors();
            auto tmpPath = runIiwa(iCtr,vec[i],vec[i+1]);
            if (!tmpPath.empty())
                path.insert(path.end(), tmpPath.begin(), tmpPath.end());
            ui::save("IiwaMainObs.json", Stats::serialize(), 4, true);
        }
        ui::save("kukaPath.json", jsonSerializer::serialize<7>(path));
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::debug);
    planningThread();
}
