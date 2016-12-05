#include <chrono>
#include <iostream>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Core>

#include <core/Logging.h>
#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/PRMPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/PointRobot.h>
#include <robot/TriangleRobot2D.h>

#include <ui/Drawing2D.h>
#include <ui/Writer.h>

using namespace rmpl;

cv::Mat obstacleWorkspace;

void testTriangleRobot(Vec<float> min, Vec<float> max, Eigen::MatrixXi mat) {
    min.append(0);
    max.append(359.999);
    std::vector<Triangle<Eigen::Vector2f>> triangles;
    triangles.push_back(Triangle<Eigen::Vector2f>(Eigen::Vector2f(0.0, 0.0), Eigen::Vector2f(0.0, 25), Eigen::Vector2f(25, 0.0)));
    triangles.push_back(Triangle<Eigen::Vector2f>(Eigen::Vector2f(0.0,25),   Eigen::Vector2f(25,0.0),  Eigen::Vector2f(25,25)));
    triangles.push_back(Triangle<Eigen::Vector2f>(Eigen::Vector2f(0.0,25),   Eigen::Vector2f(25,25),   Eigen::Vector2f(25,50)));
    std::shared_ptr<TriangleRobot2D> triangleRobot(new TriangleRobot2D(triangles, min, max));
    triangleRobot->set2DWorkspace(mat);

    std::shared_ptr<RRTOptions> options(new RRTOptions(30, 0.5, TrajectoryMethod::linear, SamplingMethod::randomly));
    StarRRTPlanner planner(triangleRobot, options);

    auto startTime = std::chrono::system_clock::now();
    Vec<float> start(5, 5, 0);
    Vec<float> goal(400.0, 950.0, 0);
    bool connected = planner.computePath(start, goal, 30000, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    std::vector<std::shared_ptr<Node>> nodes = planner.getGraphNodes();
    nodes = planner.getPathNodes();
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vec<float>> path = planner.getPath(40, true);

        //Writer::writeVecsToFile(path, "result.txt");

        cv::Mat image = obstacleWorkspace.clone();
        cv::cvtColor(image, image, CV_GRAY2BGR);
        Drawing2D::drawTrianglePath(path, triangles, image, Vec<uint8_t>(0, 0, 255), 2);

        cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
        cv::imshow("pathPlanner", image);
        cv::imwrite("result.png", image);
        cv::waitKey(0);
    }
}

void testPointRobot(Vec<float> min, Vec<float> max, Eigen::MatrixXi mat) {
    std::shared_ptr<PointRobot> robot(new PointRobot(min, max));
    robot->set2DWorkspace(mat);
    std::shared_ptr<RRTOptions> options(new RRTOptions(50, 0.5, TrajectoryMethod::linear, SamplingMethod::randomly));
    StarRRTPlanner planner(robot, options);
    NormalRRTPlanner planner1(robot, options);
    // compute the tree
    auto startTime = std::chrono::system_clock::now();
    Vec<float> start(50.0, 30.0);
    Vec<float> goal(870.0, 870.0);
    // planner.setInitNode(start);
    // planner.computeTree(4000, 2);
    // bool connected = planner.connectGoalNode(goal);
    bool connected = planner.computePath(start, goal, 4000, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    std::vector<std::shared_ptr<Node>> nodes = planner.getGraphNodes();

    cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    Drawing2D::drawTree2D(nodes, image, Vec<uint8_t>(0, 0, 255), Vec<uint8_t>(0, 0, 0), 1);

    if (connected) {
        std::vector<Vec<float>> pathPoints = planner.getPath(0.5, true);
        Drawing2D::drawPath2D(pathPoints, image, Vec<uint8_t>(255, 0, 0), 3);
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
    cv::Mat dst;
    obstacleWorkspace.convertTo(dst, CV_32SC1);

    int rows = dst.rows;
    int cols = dst.cols;
    Eigen::MatrixXi mat(rows, cols);
    std::vector<int> entries;
    int* temp;
    for (int i = 0; i < cols; ++i) {
        temp = dst.ptr<int>(i);
        for (int j = 0; j < rows; ++j) {
            entries.push_back(*temp);
            ++temp;
        }
    }
    mat = Eigen::MatrixXi::Map(&entries[0], rows, cols);

    Vec<float> minBoundary(0.0, 0.0);
    Vec<float> maxBoundary(rows, cols);

    testTriangleRobot(minBoundary, maxBoundary, mat);

    //testPointRobot(minBoundary, maxBoundary, mat);
}
