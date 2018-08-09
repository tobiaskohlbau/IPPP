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
bool drawImages = true;

ModuleConfigurator<7> getIiwaCreatorConst(std::string seed) {
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
                                             util::toTransform(util::Vecd(0, 0, 205, -util::halfPi(), 0, 0)),
                                             Transform::Identity(), FLAGS_assetsDir + "/robotModels/rhmiGripper.obj");

    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/shelveAndHole.obj");
    envConfigurator.saveConfig("KukaEnvConfig.json");

    ModuleConfigurator<7> creator;
    creator.setEnvironment(envConfigurator.getEnvironment());
    creator.setValidityCheckerType(ValidityCheckerType::FclSerialAndConstraint);
    creator.setGraphSortCount(1000);
    creator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    creator.setEvaluatorProperties(util::toRad(90), 720);
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
    double eps = util::toRad(5);

    Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -eps, -eps, -IPPP_MAX);
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
        Vector<7> vec[2];
        vec[0] = util::toRad<7>(util::Vecd(3.4, 106.16, 73.61, 41.94, 21.82, -53.56, 1.89));
        vec[1] = util::toRad<7>(util::Vecd(5.68, 63.66, 68.72, 24.01, 19.5, -72.36, -34.37));

        auto iCtr = getIiwaCreatorConst(seed);
        //util::saveMeshes(*iCtr.getEnvironment(), vec[0], "vec0");
        //util::saveMeshes(*iCtr.getEnvironment(), vec[1], "vec1");

        Stats::initializeCollectors();
        auto tmpPath = runIiwa(iCtr, vec[0], vec[1]);
        ui::save("IiwaMainObs2.json", Stats::serialize(), 4, true);
        ui::save("kukaPath.json", jsonSerializer::serialize<7>(tmpPath));
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::debug);
    planningThread();
}
