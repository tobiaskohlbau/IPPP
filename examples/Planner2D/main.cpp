#include <chrono>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>
#include <ui/Drawing2D.hpp>

using namespace ippp;

Mesh generateMap() {
    const unsigned int dim = 2;
    Vector2 min(0, 0);
    Vector2 max(1000, 1000);
    std::shared_ptr<Sampler<dim>> sampler(new SamplerRandom<dim>(min, max));

    MapGenerator<dim> mapGenerator(min, max, sampler);
    auto meshes = mapGenerator.generateMap(80, Vector2(80, 80), Vector2(10, 10));
    auto mesh = cad::mergeMeshes(meshes);
    cad::exportCad(cad::ExportFormat::OBJ, "obstacle", mesh);
    return mesh;
}

bool testTriangleRobot() {
    const unsigned int dim = 3;

    Vector3 min(0.0, 0.0, 0.0);
    Vector3 max(1000, 1000, util::twoPi());

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(dim, AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)));
    envConfigurator.setRobotType(RobotType::Triangle2D, "models/robots/simpleTriangleRobot.obj");
    generateMap();
    envConfigurator.addObstaclePath("obstacle.obj");
    envConfigurator.saveConfig("envConfigTriangle.json");
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setGraphSortCount(3000);
    creator.setCollisionType(CollisionType::Dim2Triangle);
    creator.setEvaluatorType(EvaluatorType::QueryOrTime);
    creator.setEvaluatorProperties(50, 60);
    creator.setSamplingType(SamplingType::Straight);
    creator.saveConfig("moduleConfigTriangle.json");

    std::shared_ptr<ippp::Planner<dim>> planner;
    //    planner = std::shared_ptr<PRM<dim>>(new PRM<dim>(environment, creator.getPRMOptions(30), creator.getGraph()));
    planner = std::shared_ptr<RRTStar<dim>>(new RRTStar<dim>(environment, creator.getRRTOptions(40), creator.getGraph()));
    // planner = std::shared_ptr<RRT<dim>>(new RRT<dim>(environment, creator.getRRTOptions(50), creator.getGraph()));
    // planner = std::shared_ptr<SRT<dim>>(new SRT<dim>(environment, creator.getSRTOptions(20), creator.getGraph()));

    auto startTime = std::chrono::system_clock::now();
    Vector3 start(50, 50, 0);
    Vector3 goal(900, 900, 50 * util::toRad());
    bool connected = planner->computePath(start, goal, 5000, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    Eigen::MatrixXi workspace2D = cad::create2dspace(environment->getBoundary(), 255);
    std::vector<Mesh> meshes;
    for (auto obstacle : environment->getObstacles())
        meshes.push_back(obstacle->m_mesh);
    cad::drawTriangles(workspace2D, meshes, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D);
    cv::cvtColor(image, image, CV_GRAY2BGR);

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vector3> path = planner->getPath(80, 5);

        drawing::drawTrianglePath(path, environment->getRobot()->getBaseModel()->m_mesh, image, Eigen::Vector3i(0, 0, 255), 2);
        writer::writeVecsToFile<3>(path, "test.txt", 1);

        cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
        cv::imshow("pathPlanner", image);
        cv::imwrite("result.png", image);
        cv::waitKey(0);
        return true;
    }
    return false;
}

void testPointRobot() {
    const unsigned int dim = 2;
    Vector2 min(0.0, 0.0);
    Vector2 max(1000, 1000);

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(dim, AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)));
    envConfigurator.addObstaclePath("models/spaces/random2D.obj");
    envConfigurator.setRobotType(RobotType::Point);
    envConfigurator.saveConfig("envConfig.json");
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setCollisionType(CollisionType::Dim2);
    creator.setGraphSortCount(3000);
    creator.setEvaluatorType(EvaluatorType::QueryOrTime);
    creator.setEvaluatorProperties(50, 30);
    creator.setSamplingType(SamplingType::NearObstacle);
    creator.setSamplingProperties(10, 80);
    creator.setMetricWeightVec(Vector2(1.1, 1.1));
    creator.saveConfig("moduleConfig.json");
    creator.loadConfig("moduleConfig.json");

    std::shared_ptr<ippp::Planner<dim>> planner;
    // planner = std::shared_ptr<EST<dim>>(new EST<dim>(environment, creator.getPlannerOptions(), creator.getGraph()));
    // planner = std::shared_ptr<PRM<dim>>(new PRM<dim>(environment, creator.getPRMOptions(20), creator.getGraph()));
    planner = std::shared_ptr<RRTStar<dim>>(new RRTStar<dim>(environment, creator.getRRTOptions(35), creator.getGraph()));
    // planner = std::shared_ptr<RRT<dim>>(new RRT<dim>(environment, creator.getRRTOptions(50), creator.getGraph()));
    // planner = std::shared_ptr<SRT<dim>>(new SRT<dim>(environment, creator.getSRTOptions(20), creator.getGraph()));

    // compute the tree
    auto startTime = std::chrono::system_clock::now();
    Vector2 start(10.0, 10.0);
    Vector2 goal(990.0, 990.0);
    bool connected = planner->computePath(start, goal, 500, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();

    Eigen::MatrixXi workspace2D = cad::create2dspace(environment->getBoundary(), 255);
    std::vector<Mesh> meshes;
    for (auto obstacle : environment->getObstacles())
        meshes.push_back(obstacle->m_mesh);
    cad::drawTriangles(workspace2D, meshes, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D);
    cv::cvtColor(image, image, CV_GRAY2BGR);

    drawing::drawGraph2D(nodes, image, Eigen::Vector3i(125, 125, 200), Eigen::Vector3i(125, 125, 200), 1);
    drawing::drawTree2D(nodes, image, Eigen::Vector3i(0, 0, 255), Eigen::Vector3i(125, 125, 200), 1);

    if (connected) {
        std::vector<Vector2> pathPoints = planner->getPath();
        drawing::drawPath2D(pathPoints, image, Eigen::Vector3i(255, 0, 0), 2);
        writer::writeVecsToFile<2>(pathPoints, "test.txt", 1);
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::imwrite("result.png", image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    Logging::setLogLevel(LogLevel::debug);

    //while (1)
    //    if (testTriangleRobot())
    //        break;
     testPointRobot();
}
