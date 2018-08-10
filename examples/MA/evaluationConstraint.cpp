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
DEFINE_string(obstacle2dDir, FLAGS_assetsDir + "spaces/2D/", "obstacle directory");

AABB workspace(Vector3(0, 0, 0), Vector3(2000, 2000, 2000));
AABB iiwaWorkspace(Vector3(-2500, -2500, -200), Vector3(2500, 2500, 2500));

ModuleConfigurator<6> getSerialCreator(std::string seed, SamplingType samplingType, size_t obstacleType) {
    const unsigned int dim = 6;
    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(workspace);
    envConfigurator.setRobotType(RobotType::Serial);
    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    Vector6 minBound = util::Vecd(-util::pi(), -util::pi(), -util::pi(), -util::pi(), -util::pi(), -util::pi());
    envConfigurator.setRobotBaseProperties(dim, std::vector<DofType>(dim, DofType::jointRot),
                                           std::make_pair(minBound, -minBound));
    std::vector<DhParameter> dhParameters(dim, DhParameter(0, 83.3333333333));
    std::vector<std::string> jointModelFiles(dim, FLAGS_assetsDir + "robotModels/2D/2dLineDim6.obj");
    envConfigurator.setSerialRobotProperties(dhParameters, jointModelFiles);

    std::shared_ptr<Environment> env = envConfigurator.getEnvironment();
    env->getRobot()->setPose(util::Vecd(1000, 1000, 0, 0, 0, 0));

    ModuleConfigurator<6> creator;
    creator.setEnvironment(env);
    creator.setValidityCheckerType(ValidityCheckerType::FclSerialAndConstraint);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    creator.setEvaluatorProperties(util::toRad(90), 90);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplingType(samplingType);
    creator.setSamplerProperties(seed, 1);
    creator.setSamplingProperties(50, util::toRad(90));
    return creator;
}

ModuleConfigurator<7> getIiwaCreator(std::string seed, SamplingType samplingType, size_t obstacleType) {
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

    ModuleConfigurator<7> creator;
    creator.setEnvironment(envConfigurator.getEnvironment());
    creator.setValidityCheckerType(ValidityCheckerType::FclSerialAndConstraint);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    creator.setEvaluatorProperties(util::toRad(90), 90);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplingType(samplingType);
    creator.setSamplerProperties(seed, 1);
    creator.setSamplingProperties(50, util::toRad(90));
    return creator;
}

void runSerial(ModuleConfigurator<6>& sCtr, size_t type) {
    const unsigned dim = 6;
    const double epsPos = 2.5;
    const double epsOri = util::toRad(5);
    const double maxLength = 500;

    Vector<dim> start = Vector<dim>::Zero();
    Vector<dim> goal = Vector<dim>::Zero();
    Vector6 Cmin;
    std::pair<Vector6, Vector6> C;
    Transform taskFrame;
    double q1Angle = util::toRad(35);
    double length = 1000 + (maxLength * std::cos(q1Angle));

    if (type == 0) {    // case 1: fixed x
        Cmin = util::Vecd(-epsPos, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
        C = std::make_pair(Cmin, -Cmin);
        taskFrame = util::toTransform(util::Vecd(length, 0, 0, 0, 0, 0));
        start[0] = -q1Angle;
        goal[0] = q1Angle;
    } else if (type == 1) {    // case 2: fixed y
        q1Angle = util::toRad(10);
        Cmin = util::Vecd(-IPPP_MAX, -epsPos, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
        C = std::make_pair(Cmin, -Cmin);
        taskFrame = util::toTransform(util::Vecd(0, 1389.6, 0, 0, 0, 0));
        start[0] = q1Angle + util::toRad(90);
        goal[0] = -q1Angle + util::toRad(90);
        start[3] = util::toRad(45);
        goal[3] = - util::toRad(45);
    } else if (type == 2) {    // case 3: fixed orientation z
        q1Angle = util::toRad(90);
        Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -epsOri);
        C = std::make_pair(Cmin, -Cmin);
        taskFrame = util::toTransform(util::Vecd(0, 0, 0, 0, 0, 0));
        start[0] = -q1Angle;
        goal[0] = q1Angle;
        start[dim - 1] = q1Angle;
        goal[dim - 1] = -q1Angle;
    }

    sCtr.setConstraintProperties(C, taskFrame);
    auto planner = std::make_shared<RRTStarConnect<6>>(sCtr.getEnvironment(), sCtr.getRRTOptions(util::toRad(90)),
                                                       sCtr.getGraph(), sCtr.getGraphB());
    planner->computePath(start, goal, 480, 24);
    planner->getPath();
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
    auto planner = std::make_shared<RRTStarConnect<7>>(iCtr.getEnvironment(), iCtr.getRRTOptions(util::toRad(90)),
                                                       iCtr.getGraph(), iCtr.getGraphB());
    planner->computePath(start, goal, 480, 24);
    planner->getPath();
}

void planningThread() {
    std::vector<SamplingType> samplingTypes = {SamplingType::Berenson, SamplingType::TS, SamplingType::FOR};

    for (auto& samplingType : samplingTypes) {
        std::string samplingName = "Berenson";
        if (samplingType == SamplingType::TS)
            samplingName = "TS";
        else if (samplingType == SamplingType::FOR)
            samplingName = "FOR";

        std::vector<std::string> seeds = {"234`r5fdsfda", "23r54wedf",  "23894rhwef",  "092yu4re",   "0923ujrpiofesd",
                                          "02u9r3jes",    "09243rjpef", "23refdss;wf", "2-3r0wepoj", "-243refdsaf"};
        for (auto& seed : seeds) {
            for (size_t i = 0; i < 3; ++i) {
                // serial plane robot
                Stats::initializeCollectors();
                auto sCtr = getSerialCreator(seed, samplingType, i);
                runSerial(sCtr, i);
                ui::save(samplingName + "Serial.json", Stats::serialize(), 4, true);

                // iiwa
                Stats::initializeCollectors();
                auto iCtr = getIiwaCreator(seed, samplingType, i);
                //runIiwa(iCtr, i);
                ui::save(samplingName + "Iiwa.json", Stats::serialize(), 4, true);
            }
        }
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::debug);
    planningThread();
}
