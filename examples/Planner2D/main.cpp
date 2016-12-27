#include <chrono>
#include <iostream>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Core>

#include <core/utility/Logging.h>
#include <core/utility/Utility.h>
#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/PRMPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/PointRobot.h>
#include <robot/TriangleRobot2D.h>

#include <ui/Drawing2D.h>
#include <ui/Writer.h>

using namespace rmpl;

cv::Mat obstacleWorkspace;

void testTriangleRobot(Eigen::Vector2f minimum, Eigen::Vector2f maximum, Eigen::MatrixXi mat) {
    Eigen::Vector3f min = utilVec::append(minimum, 0.0);
    Eigen::Vector3f max = utilVec::append(maximum, 360.0);

    std::vector<Triangle2D> triangles;
    triangles.push_back(Triangle2D(Eigen::Vector2f(0.0, 0.0), Eigen::Vector2f(0.0, 25.0), Eigen::Vector2f(25.0, 0.0), Eigen::Vector2f(25.0, 0.0)));
    triangles.push_back(Triangle2D(Eigen::Vector2f(0.0, 0.0), Eigen::Vector2f(0.0, 25.0), Eigen::Vector2f(25.0, 0.0)));
    triangles.push_back(Triangle2D(Eigen::Vector2f(0.0, 25.0), Eigen::Vector2f(25, 0.0), Eigen::Vector2f(25, 25)));
    triangles.push_back(Triangle2D(Eigen::Vector2f(0.0, 25.0), Eigen::Vector2f(25, 25), Eigen::Vector2f(25, 45)));
    std::shared_ptr<TriangleRobot2D> triangleRobot(new TriangleRobot2D(triangles, min, max));
    triangleRobot->set2DWorkspace(mat);

    PRMOptions prmOptions(30, 0.5, SamplingMethod::randomly);
    RRTOptions rrtOptions(30, 0.5, SamplingMethod::randomly);

    std::shared_ptr<rmpl::Planner> planner;
    // planner = std::shared_ptr<PRMPlanner>(new PRMPlanner(triangleRobot, prmOptions));
    planner = std::shared_ptr<StarRRTPlanner>(new StarRRTPlanner(triangleRobot, rrtOptions));
    // planner = std::shared_ptr<NormalRRTPlanner>(new NormalRRTPlanner(triangleRobot, rrtOptions));

    auto startTime = std::chrono::system_clock::now();
    Eigen::Vector3f start(5, 5, 0);
    Eigen::Vector3f goal(400.0, 930.0, 50);
    bool connected = planner->computePath(start, goal, 20000, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    std::vector<std::shared_ptr<Node>> nodes = planner->getGraphNodes();
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Eigen::VectorXf> path = planner->getPath(60, true);

        // Writer::writeVecsToFile(path, "result.txt");

        cv::Mat image = obstacleWorkspace.clone();
        cv::cvtColor(image, image, CV_GRAY2BGR);
        Drawing2D::drawTrianglePath(path, triangles, image, Eigen::Vector3i(0, 0, 255), 2);

        cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
        cv::imshow("pathPlanner", image);
        cv::imwrite("result.png", image);
        cv::waitKey(0);
    }
}

void testPointRobot(Eigen::Vector2f min, Eigen::Vector2f max, Eigen::MatrixXi mat) {
    std::shared_ptr<PointRobot> robot(new PointRobot(min, max));
    robot->set2DWorkspace(mat);

    PRMOptions prmOptions(40, 0.5, SamplingMethod::randomly, SamplingStrategy::nearObstacles);
    RRTOptions rrtOptions(50, 1, SamplingMethod::randomly, SamplingStrategy::nearObstacles);

    std::shared_ptr<rmpl::Planner> planner;
    // planner = std::shared_ptr<PRMPlanner>(new PRMPlanner(robot, prmOptions));
    planner = std::shared_ptr<StarRRTPlanner>(new StarRRTPlanner(robot, rrtOptions));
    // planner = std::shared_ptr<NormalRRTPlanner>(new NormalRRTPlanner(robot, rrtOptions));

    // compute the tree
    auto startTime = std::chrono::system_clock::now();
    Eigen::Vector2f start(50.0, 30.0);
    Eigen::Vector2f goal(870.0, 870.0);
    bool connected = planner->computePath(start, goal, 5200, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    std::vector<std::shared_ptr<Node>> nodes = planner->getGraphNodes();

    cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    Drawing2D::drawGraph2D(nodes, image, Eigen::Vector3i(125, 125, 125), Eigen::Vector3i(125, 125, 125), 2);
    Drawing2D::drawTree2D(nodes, image, Eigen::Vector3i(0, 0, 255), Eigen::Vector3i(0, 0, 0), 1);

    if (connected) {
        std::vector<Eigen::VectorXf> pathPoints = planner->getPath(1, true);
        Drawing2D::drawPath2D(pathPoints, image, Eigen::Vector3i(255, 0, 0), 3);
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::imwrite("result.png", image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    // obstacleWorkspace = cv::imread("spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    // obstacleWorkspace = cv::imread("spaces/labyrinth.png", CV_LOAD_IMAGE_GRAYSCALE);
    obstacleWorkspace = cv::imread("spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    Eigen::MatrixXi mat = Drawing2D::cvToEigen(obstacleWorkspace);

    Eigen::Vector2f minBoundary(0.0, 0.0);
    Eigen::Vector2f maxBoundary(mat.rows(), mat.cols());

    //testTriangleRobot(minBoundary, maxBoundary, mat);

    testPointRobot(minBoundary, maxBoundary, mat);
}
