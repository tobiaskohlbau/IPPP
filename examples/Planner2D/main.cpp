#include <chrono>
#include <iostream>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Core>

#include <pathPlanner/PRMPlanner.h>
#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/PointRobot.h>

#include <ui/Drawing2D.h>

int main(int argc, char** argv) {
    cv::Mat freeWorkspace, obstacleWorkspace;
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

    rmpl::Vec<float> minBoundary(0.0, 0.0);
    rmpl::Vec<float> maxBoundary(rows, cols);
    std::shared_ptr<rmpl::PointRobot> robot(new rmpl::PointRobot(minBoundary, maxBoundary));
    robot->set2DWorkspace(mat);
    std::shared_ptr<rmpl::RRTOptions> options(
        new rmpl::RRTOptions(50, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly));
    rmpl::StarRRTPlanner planner(robot, options);
    rmpl::NormalRRTPlanner planner1(robot, options);
    // compute the tree
    auto startTime = std::chrono::system_clock::now();
    rmpl::Vec<float> start(50.0, 30.0);
    rmpl::Vec<float> goal(870.0, 870.0);
    // planner.setInitNode(start);
    // planner.computeTree(4000, 2);
    // bool connected = planner.connectGoalNode(goal);
    bool connected = planner.computePath(start, goal, 4000, 2);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();

    cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    Drawing2D::drawTree2D(nodes, image, rmpl::Vec<uint8_t>(0, 0, 255), rmpl::Vec<uint8_t>(0, 0, 0), 1);

    if (connected) {
        std::vector<rmpl::Vec<float>> pathPoints = planner.getPath(0.5, true);
        Drawing2D::drawPath2D(pathPoints, image, rmpl::Vec<uint8_t>(255, 0, 0), 3);
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::imwrite("result.png", image);
    cv::waitKey(0);
}
