#include <ctime>
#include <iostream>
#include <memory>

#include "opencv2/core/core.hpp"
#include <Eigen/Core>

#include <pathPlanner/PRMPlanner.h>
#include <robot/PointRobot.h>

#include <ui/Drawing.h>

void printTime(clock_t begin, clock_t end) {
    float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

int main(int argc, char** argv) {
    cv::Mat freeWorkspace, obstacleWorkspace;
    obstacleWorkspace = cv::imread("spaces/labyrinth.png", CV_LOAD_IMAGE_GRAYSCALE);
    // obstacleWorkspace = cv::imread("spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);

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

    std::shared_ptr<rmpl::PRMOptions> options(
        new rmpl::PRMOptions(30, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::uniform));
    rmpl::PRMPlanner planner(robot, options);

    rmpl::Vec<float> start(50, 30);
    rmpl::Vec<float> goal(870.0, 870.0);

    clock_t begin = std::clock();
    // planner.startSamplingPhase(4000, 2);
    // planner.startPlannerPhase(2);
    // bool connected = planner.queryPath(start, goal);
    bool connected = planner.computePath(start, goal, 6000, 2);
    clock_t end = std::clock();
    printTime(begin, end);

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();

    cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    // Drawing::drawGraph2D(nodes, image, rmpl::Vec<uint8_t>(0, 0, 255), rmpl::Vec<uint8_t>(0, 0, 0), 1);
    Drawing::drawTree2D(nodes, image, rmpl::Vec<uint8_t>(0, 0, 255), rmpl::Vec<uint8_t>(0, 0, 0), 1);

    if (connected) {
        std::vector<rmpl::Vec<float>> pathPoints = planner.getPath(0.5, true);
        Drawing::drawPath2D(pathPoints, image, rmpl::Vec<uint8_t>(255, 0, 0), 3);
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::waitKey(0);
}
