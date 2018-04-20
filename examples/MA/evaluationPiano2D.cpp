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

std::pair<Vector3, Vector3> setObstacle(EnvironmentConfigurator& envConfigurator, size_t type) {
    std::string space = FLAGS_assetsDir + "/spaces/2D/";
    if (type == 1) {
        envConfigurator.addObstacle(space + "100x1000x2.obj", util::Vecd(300, 400, 0, 0, 0, util::toRad(90)));
        envConfigurator.addObstacle(space + "100x1000x2.obj", util::Vecd(500, 600, 0, 0, 0, util::toRad(90)));
        envConfigurator.addObstacle(space + "100x1000x2.obj", util::Vecd(700, 400, 0, 0, 0, util::toRad(90)));
        return std::make_pair(Vector3(50, 50, 0), Vector3(950, 50, 0));
    } else if (type == 2) {
        envConfigurator.addObstacle(space + "800x200x2.obj", util::Vecd(500, 200, 0, 0, 0, util::toRad(90)));
        envConfigurator.addObstacle(space + "800x200x2.obj", util::Vecd(500, 800, 0, 0, 0, util::toRad(90)));
        envConfigurator.addObstacle(space + "800x200x2.obj", util::Vecd(800, 500, 0, 0, 0, 0));
        return std::make_pair(Vector3(620, 500, 0), Vector3(920, 500, 0));
    } else if (type == 3) {
        envConfigurator.addObstacle(space + "100x1000x2.obj", util::Vecd(400, -35, 0, 0, 0, util::toRad(90)));
        envConfigurator.addObstacle(space + "100x1000x2.obj", util::Vecd(500, -35, 0, 0, 0, util::toRad(90)));
        envConfigurator.addObstacle(space + "100x1000x2.obj", util::Vecd(400, 1035, 0, 0, 0, util::toRad(90)));
        envConfigurator.addObstacle(space + "100x1000x2.obj", util::Vecd(500, 1035, 0, 0, 0, util::toRad(90)));
        return std::make_pair(Vector3(50, 50, 0), Vector3(950, 50, 0));
    }
    return std::make_pair(Vector3(50, 50, 0), Vector3(950, 950, 0));
}

bool testTriangleRobot() {
    const unsigned int dim = 3;

    EnvironmentConfigurator envConfigurator;
    AABB workspaceBound(Vector3(0, 0, 0), Vector3(1000, 1000, 1));
    envConfigurator.setWorkspaceProperties(workspaceBound);
    envConfigurator.setRobotType(RobotType::Mobile2D);
    envConfigurator.setRobotBaseModelFile(FLAGS_assetsDir + "/robotModels/simpleTriangleRobot.obj");
    auto startGoal = setObstacle(envConfigurator, 1);
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setGraphSortCount(3000);
    creator.setValidityCheckerType(ValidityCheckerType::FclMobile);
    creator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    creator.setEvaluatorProperties(50, 30);

    std::shared_ptr<ippp::Planner<dim>> planner;
    // planner = std::make_shared<PRM<dim>>(environment, creator.getPRMOptions(30), creator.getGraph());
    // planner = std::make_shared<RRTStar<dim>>(environment, creator.getRRTOptions(40), creator.getGraph());
    planner =
        std::make_shared<RRTStarConnect<dim>>(environment, creator.getRRTOptions(40), creator.getGraph(), creator.getGraphB());
    // planner = std::make_shared<RRT<dim>>(environment, creator.getRRTOptions(50), creator.getGraph());
    // planner = std::make_shared<SRT<dim>>(environment, creator.getSRTOptions(20), creator.getGraph());

    bool connected = planner->computePath(startGoal.first, startGoal.second, 2000, 3);

    auto workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    for (const auto& obstacle : environment->getObstacles())
        drawing::drawPolygons(image, obstacle->model->m_mesh, obstacle->getPose(), workspace2D.second, Vector3i(50, 50, 50));

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();
    drawing::drawGraph2D<dim>(image, nodes, Vector3i(0, 0, 255), Vector3i(125, 125, 200), 1);
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        auto path = planner->getPath(100, util::toRad(120));
        drawing::drawTrianglePath(image, path, environment->getRobot()->getBaseModel()->m_mesh, workspace2D.second,
                                  Vector3i(0, 0, 255));
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::imwrite("result.png", image);
    cv::waitKey(0);
    return connected;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::trace);

    testTriangleRobot();
}
