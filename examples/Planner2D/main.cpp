#include <chrono>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ippp/Core>
#include <ippp/Environment>
#include <ippp/Planner>

#include <modelDirectory.h>
#include <ui/Drawing2D.hpp>
#include <ui/ModuleCreator.hpp>
#include <ui/Writer.hpp>

using namespace ippp;

void testTriangleRobot() {
    const unsigned int dim = 3;
    Vector3 min(0.0, 0.0, 0.0);
    Vector3 max(1000, 1000, util::twoPi());

    std::vector<Triangle2D> triangles;
    triangles.push_back(Triangle2D(Vector2(0, 0), Vector2(25, 0), Vector2(25, 50)));
    triangles.push_back(Triangle2D(Vector2(0, 0), Vector2(0, 50), Vector2(25, 50)));
    ModelFactoryTriangle2D factory;
    std::shared_ptr<ModelContainer> baseModel = factory.createModel(triangles);
    std::shared_ptr<TriangleRobot2D> robot(new TriangleRobot2D(baseModel, std::make_pair(min, max)));
    std::shared_ptr<Environment> environment(new Environment(2, AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)), robot));
    // std::shared_ptr<ModelContainer> model(new Model2D(mat));
    // robot->setWorkspace(model);

    std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetectionTriangleRobot(environment));
    ModuleCreator<dim> creator(environment, collision, MetricType::L2, NeighborType::KDTree, PathModifierType::NodeCut,
                               SamplerType::SamplerUniform, SamplingType::Straight, TrajectoryType::Linear);

    std::shared_ptr<ippp::Planner<dim>> planner;
    // planner = std::shared_ptr<PRM<dim>>(new PRM<dim>(environment, creator.getPRMOptions(40), creator.getGraph()));
    planner = std::shared_ptr<RRTStar<dim>>(new RRTStar<dim>(environment, creator.getRRTOptions(50), creator.getGraph()));
    // planner = std::shared_ptr<RRTStarContTraj<dim>>(new RRTStarContTraj<dim>(environment, creator.getRRTOptions(50),
    // creator.getGraph()));
    // planner = std::shared_ptr<RRT<dim>>(new RRT<dim>(environment, creator.getRRTOptions(50), creator.getGraph()));
    // planner = std::shared_ptr<SRT<dim>>(new SRT<dim>(environment, creator.getSRTOptions(20), creator.getGraph()));

    auto startTime = std::chrono::system_clock::now();
    Vector3 start(5, 5, 0);
    Vector3 goal(400.0, 930.0, 50 * util::toRad());
    bool connected = planner->computePath(start, goal, 20000, 4);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vector3> path = planner->getPath(80);

        // Writer::writeVecsToFile(path, "result.txt");

        Eigen::MatrixXi workspace2D = cad::create2dspace(environment->getBoundary(), 255);
        cv::Mat image = drawing::eigenToCV(workspace2D);
        // cv::Mat image = obstacleWorkspace.clone();
        cv::cvtColor(image, image, CV_GRAY2BGR);

        drawing::drawTrianglePath(path, triangles, image, Eigen::Vector3i(0, 0, 255), 2);
        writer::writeVecsToFile<3>(path, "test.txt", 1);

        cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
        cv::imshow("pathPlanner", image);
        cv::imwrite("result.png", image);
        cv::waitKey(0);
    }
}

void testPointRobot() {
    Vector2 min(0.0, 0.0);
    Vector2 max(1000, 1000);

    const unsigned int dim = 2;
    std::shared_ptr<PointRobot> robot(new PointRobot(std::make_pair(min, max)));
    std::shared_ptr<Environment> environment(new Environment(2, AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)), robot));
    ModelFactoryTriangle2D factory;
    auto workspace = factory.createModel(getModelDirectory() + "/spaces/easyMaze.obj");
    environment->addObstacle(workspace);
    // robot->setWorkspace(model);

    std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetection2D(environment));
    ModuleCreator<dim> creator(environment, collision, MetricType::L2, NeighborType::KDTree, PathModifierType::NodeCut,
                               SamplerType::SamplerUniform, SamplingType::Straight, TrajectoryType::Linear);

    std::shared_ptr<ippp::Planner<dim>> planner;
    planner = std::shared_ptr<PRM<dim>>(new PRM<dim>(environment, creator.getPRMOptions(40), creator.getGraph()));
    // planner = std::shared_ptr<RRTStar<dim>>(new RRTStar<dim>(environment, creator.getRRTOptions(50), creator.getGraph()));
    // planner = std::shared_ptr<RRTStarContTraj<dim>>(new RRTStarContTraj<dim>(environment, creator.getRRTOptions(50),
    // creator.getGraph()));
    // planner = std::shared_ptr<RRT<dim>>(new RRT<dim>(environment, creator.getRRTOptions(50), creator.getGraph()));
    // planner = std::shared_ptr<SRT<dim>>(new SRT<dim>(environment, creator.getSRTOptions(20), creator.getGraph()));

    // compute the tree
    auto startTime = std::chrono::system_clock::now();
    Vector2 start(50.0, 30.0);
    Vector2 goal(870.0, 870.0);
    bool connected = planner->computePath(start, goal, 6000, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();

    Eigen::MatrixXi workspace2D = cad::create2dspace(environment->getBoundary(), 255);
    cad::drawTriangles(workspace2D, std::dynamic_pointer_cast<ModelTriangle2D>(workspace)->m_triangles, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D);
    // cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    drawing::drawGraph2D(nodes, image, Eigen::Vector3i(125, 125, 125), Eigen::Vector3i(125, 125, 125), 1);
    drawing::drawTree2D(nodes, image, Eigen::Vector3i(0, 0, 255), Eigen::Vector3i(0, 0, 0), 1);

    if (connected) {
        std::vector<Vector2> pathPoints = planner->getPath();
        drawing::drawPath2D(pathPoints, image, Eigen::Vector3i(255, 0, 0), 3);
        writer::writeVecsToFile<2>(pathPoints, "test.txt", 1);
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::imwrite("result.png", image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    testTriangleRobot();

    testPointRobot();
}
