#include <thread>

#include <ConfigurationMA.h>
#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../../assets", "assets directory");

AABB workspace(Vector3(0, 0, 0), Vector3(2000, 2000, 2000));
Vector2 startPoint(300, 300);
Vector2 goalPoint(1800, 1500);
Vector6 startSerial = util::Vecd(util::halfPi(), 0, 0, 0, 0, 0);
Vector6 goalSerial = util::Vecd(-util::halfPi(), -util::halfPi(), 0, util::halfPi(), 0, 0);

ModuleConfigurator<2> getPointCreator(std::string seed, EvaluatorType evalType, size_t obstacleType) {
    const unsigned int dim = 2;
    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(workspace);
    envConfigurator.setRobotType(RobotType::Point2D);

    Vector2 min(workspace.min()[0], workspace.min()[1]);
    Vector2 max(workspace.max()[0], workspace.max()[1]);
    std::string workspaceSeed = "2234wqdfsa]lksd";
    if (obstacleType == 1)
        workspaceSeed = "asdfeew234;6ufyrku;dsa";
    else if (obstacleType == 2)
        workspaceSeed = "4398;lkjfdsafwer23";
    cad::MapGenerator<dim> mapGenerator(workspace, std::make_shared<SamplerRandom<2>>(std::make_pair(min, max), workspaceSeed));
    auto meshes = mapGenerator.generateMap(400, Vector2(20, 20), Vector2(100, 100));
    for (auto& mesh : meshes)
        envConfigurator.addObstacle(mesh);

    ModuleConfigurator<2> creator;
    creator.setEnvironment(envConfigurator.getEnvironment());
    creator.setValidityCheckerType(ValidityCheckerType::Dim2);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(evalType);
    creator.setEvaluatorProperties(40, 45);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplingType(SamplingType::Straight);
    creator.setSamplerProperties(seed, 1);
    return creator;
}

ModuleConfigurator<6> getSerialCreator(std::string seed, EvaluatorType evalType, size_t obstacleType) {
    const unsigned int dim = 6;
    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(workspace);
    envConfigurator.setRobotType(RobotType::Serial);
    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    Vector6 minBound = util::Vecd(-util::pi(), -util::pi(), -util::pi(), -util::pi(), -util::pi(), -util::pi());
    envConfigurator.setRobotBaseProperties(dim, std::vector<DofType>(dim, DofType::jointRot),
                                           std::make_pair(minBound, -minBound));
    std::vector<DhParameter> dhParameters(dim, DhParameter(0, 83.3333333333));
    std::vector<std::string> jointModelFiles(dim, FLAGS_assetsDir + "/robotModels/2D/2dLineDim6.obj");
    envConfigurator.setSerialRobotProperties(dhParameters, jointModelFiles);
    std::shared_ptr<Environment> env = envConfigurator.getEnvironment();
    env->getRobot()->setPose(util::Vecd(1000, 1000, 0, 0, 0, 0));

    if (obstacleType == 1) {
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/100x100.obj", util::Vecd(1350, 1000, 0, 0, 0, 0));
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/100x100.obj", util::Vecd(1450, 1000, 0, 0, 0, 0));
    } else if (obstacleType == 2) {
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/100x100.obj", util::Vecd(1350, 900, 0, 0, 0, 0));
        envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/100x100.obj", util::Vecd(1350, 1100, 0, 0, 0, 0));
    }

    ModuleConfigurator<6> creator;
    creator.setEnvironment(env);
    creator.setValidityCheckerType(ValidityCheckerType::FclSerial);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(evalType);
    creator.setEvaluatorProperties(util::toRad(45), 45);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplingType(SamplingType::Straight);
    creator.setSamplerProperties(seed, 1);
    return creator;
}

void planningThread() {
    std::vector<std::string> seeds = {"234`r5fdsfda", "23r54wedf",  "23894rhwef",  "092yu4re",   "0923ujrpiofesd",
                                      "02u9r3jes",    "09243rjpef", "23refdss;wf", "2-3r0wepoj", "-243refdsaf"};
    for (auto& seed : seeds) {
        EvaluatorType evalType = EvaluatorType::TreeConfigOrTime;

        for (size_t i = 0; i < 3; ++i) {
            Stats::initializeCollectors();
            auto creator = getPointCreator(seed, evalType, i);
            auto planner = std::make_shared<RRTStar<2>>(creator.getEnvironment(), creator.getRRTOptions(40), creator.getGraph());
            planner->computePath(startPoint, goalPoint, 100, 1);
            planner->getPath();
            ui::save("evalRRTPointPoint.json", Stats::serialize(), 4, true);
        }

        for (size_t i = 0; i < 3; ++i) {
            Stats::initializeCollectors();
            auto creator = getSerialCreator(seed, evalType, i);
            auto planner = std::make_shared<RRTStar<6>>(creator.getEnvironment(), creator.getRRTOptions(util::toRad(90)), creator.getGraph());
            planner->computePath(startSerial, goalSerial, 100, 1);
            planner->getPath();
            ui::save("evalRRTSerialPoint.json", Stats::serialize(), 4, true);
        }
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::debug);
    planningThread();
}
