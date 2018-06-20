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

AABB workspace(Vector3(0, 0, 0), Vector3(2000, 2000, 2000));
AABB iiwaWorkspace(Vector3(-2500, -2500, -200), Vector3(2500, 2500, 2500));

Vector2 startPoint(300, 300);
Vector2 goalPoint(1800, 1500);
Vector6 startSerial = util::Vecd(util::halfPi()/2, 0.174, 0.436, 0.174, -util::halfPi(), -0.436);
Vector6 goalSerial = util::Vecd(-util::halfPi()/2, -util::halfPi(), -0.174, util::halfPi(), -0.174, -0.174);

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
    std::vector<std::string> jointModelFiles(dim, FLAGS_assetsDir + "robotModels/2D/2dLineDim6.obj");
    envConfigurator.setSerialRobotProperties(dhParameters, jointModelFiles);
    if (obstacleType == 1) {
        envConfigurator.addObstacle(FLAGS_obstacle2dDir + "100x100.obj", util::Vecd(1200, 1000, 0, 0, 0, 0));
        envConfigurator.addObstacle(FLAGS_obstacle2dDir + "100x100.obj", util::Vecd(1300, 1000, 0, 0, 0, 0));
    } else if (obstacleType == 2) {
        envConfigurator.addObstacle(FLAGS_obstacle2dDir + "100x100.obj", util::Vecd(1250, 900, 0, 0, 0, 0));
        envConfigurator.addObstacle(FLAGS_obstacle2dDir + "100x100.obj", util::Vecd(1250, 1100, 0, 0, 0, 0));
    }

    std::shared_ptr<Environment> env = envConfigurator.getEnvironment();
    env->getRobot()->setPose(util::Vecd(1000, 1000, 0, 0, 0, 0));

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

ModuleConfigurator<7> getIiwaCreator(std::string seed, EvaluatorType evalType, size_t obstacleType) {
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
    creator.setEvaluatorType(evalType);
    creator.setEvaluatorProperties(util::toRad(60), 45);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplingType(SamplingType::Straight);
    creator.setSamplerProperties(seed, 1);
    return creator;
}

void planningThread() {
    std::string seed = "234`r5fdsfda";
    EvaluatorType evalType = EvaluatorType::TreeConfigOrTime;

    for (size_t i = 0; i < 3; ++i) {
        // point plane robot
        auto pCtr = getPointCreator(seed, evalType, i);
        std::shared_ptr<Planner<2>> pPlanner =
            std::make_shared<RRTStar<2>>(pCtr.getEnvironment(), pCtr.getRRTOptions(40), pCtr.getGraph());

        pPlanner->computePath(startPoint, goalPoint, 10000, 24);
        auto workspace2D = cad::create2dspace(pCtr.getEnvironment()->getSpaceBoundary(), 255);
        cv::Mat imagePoint = drawing::eigenToCV(workspace2D.first);
        cv::cvtColor(imagePoint, imagePoint, CV_GRAY2BGR);
        for (const auto& obstacle : pCtr.getEnvironment()->getObstacles())
            drawing::drawPolygons(imagePoint, obstacle->model->m_mesh, obstacle->getPose(), workspace2D.second, Vector3i(50, 50, 50));
        drawing::drawPath2D(imagePoint, pPlanner->getPath(), Vector3i(255, 0, 0), 2);
        cv::imwrite("point" + std::to_string(i) + ".png", imagePoint);

        // serial plane robot
        auto sCtr = getSerialCreator(seed, evalType, i);
        std::shared_ptr<Planner<6>> sPlanner =
            std::make_shared<RRTStar<6>>(sCtr.getEnvironment(), sCtr.getRRTOptions(util::toRad(90)), sCtr.getGraph());

        //sPlanner->computePath(startSerial, goalSerial, 4000, 24);
        //sPlanner->getPath();
        auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(sCtr.getEnvironment()->getRobot());
        workspace2D = cad::create2dspace(sCtr.getEnvironment()->getSpaceBoundary(), 255);
        cv::Mat image = drawing::eigenToCV(workspace2D.first);
        cv::cvtColor(image, image, CV_GRAY2BGR);
        for (const auto& obstacle : sCtr.getEnvironment()->getObstacles())
            drawing::drawPolygons(image, obstacle->model->m_mesh, obstacle->getPose(), workspace2D.second, Vector3i(50, 50, 50));
        drawing::drawSerialRobot2D<6>(startSerial, *serialRobot, image, workspace2D.second, Vector3i(0, 0, 255));
        drawing::drawSerialRobot2D<6>(goalSerial, *serialRobot, image, workspace2D.second, Vector3i(0, 0, 255));
        cv::imwrite("serial" + std::to_string(i) + ".png", image);


        // serial iiwa robot
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

        auto iCtr = getIiwaCreator(seed, evalType, i);
        //util::saveMeshes(*iCtr.getEnvironment(), start, "start" + std::to_string(i));
        //util::saveMeshes(*iCtr.getEnvironment(), goal, "goal" + std::to_string(i));
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::info);
    planningThread();
}
