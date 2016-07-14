#include <iostream>
#include <cstdint>
#include <ctime>
#include <memory>

#include "opencv2/core/core.hpp"
#include <Eigen/Core>

#include <planner/NormalRRTPlanner.h>
#include <planner/StarRRTPlanner.h>
#include <robot/Jaco.h>
#include <robot/PointRobot.h>
#include <robot/CadFileLoader.h>

#include <vrep/Helper.h>

#include "ui/Drawing.h"

void planning2D() {
    cv::Mat freeWorkspace, obstacleWorkspace;
    freeWorkspace = cv::imread("spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    obstacleWorkspace = cv::imread("spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat dst;
    obstacleWorkspace.convertTo(dst, CV_32SC1);
    int rows = dst.rows;
    int cols = dst.cols;
    Eigen::MatrixXi mat(rows, cols);
    std::vector<int> entries;
    int *temp;
    for (int i = 0; i < rows; ++i) {
        temp = dst.ptr<int>(i);
        for (int j = 0; j < cols; ++j) {
            entries.push_back(*temp);
            ++temp;
        }
    }
    mat = Eigen::MatrixXi::Map(&entries[0], rows, cols);

    std::shared_ptr<rmpl::PointRobot> robot(new rmpl::PointRobot());
    rmpl::StarRRTPlanner planner(robot, 30.0, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    rmpl::NormalRRTPlanner planner2(robot, 30.0, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);

    // set properties to the planner
    rmpl::Vec<float> minBoundary(0.0,0.0);
    rmpl::Vec<float> maxBoundary(rows, cols);
    planner.setWorkspaceBoundaries(minBoundary, maxBoundary);
    planner.set2DWorkspace(mat); // only be used by 2D
    planner.setInitNode(rmpl::Node(10.0, 10.0));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(4000);
    clock_t end = std::clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "computation time: " << elapsed_secs << std::endl;

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();

    cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    rmpl::Node goal(650.0,750.0);
    bool connected = planner.connectGoalNode(goal);
    Drawing::drawTree2D(nodes, image, rmpl::Vec<uint8_t>(0,0,255), rmpl::Vec<uint8_t>(0,0,0), 1);

    if (connected) {
        std::vector<rmpl::Vec<float>> pathPoints = planner.getPath();
        Drawing::drawPath2D(pathPoints, image, rmpl::Vec<uint8_t>(255,0,0), 3);
    }

    cv::namedWindow("planner", CV_WINDOW_AUTOSIZE);
    cv::imshow("planner", image);
    cv::waitKey(0);
}

void planning6D() {
    std::shared_ptr<rmpl::Jaco> robot(new rmpl::Jaco());
    rmpl::NormalRRTPlanner planner(robot, 0.5, 80.0, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    std::shared_ptr<rmpl::Helper> vrep = planner.getVrep();

    // set properties to the planner
    rmpl::Vec<float> minBoundary(0, 42, 17, 0, 0, 0);
    rmpl::Vec<float> maxBoundary(360, 318, 343, 360, 360 ,360);
    planner.setWorkspaceBoundaries(minBoundary, maxBoundary);
    planner.setInitNode(rmpl::Node(180, 180, 180, 180, 180, 180));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(5000);
    clock_t end = std::clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "computation time: " << elapsed_secs << std::endl;

    rmpl::Node goal(170.0, 170.0, 200.0, 180.0, 160.0, 180.0);
    bool connected = planner.connectGoalNode(goal);

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();
    std::vector<rmpl::Vec<float>> graphPoints;
    std::cout << "Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getVec()));
    Drawing::writeVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        std::vector<rmpl::Vec<float>> pathAngles = planner.getPath();
        std::vector<rmpl::Vec<float>> pathPoints;
        for (int i = 0; i < pathAngles.size(); ++i)
            pathPoints.push_back(robot->directKinematic(pathAngles[i]));
        Drawing::appendVecsToFile(pathPoints, "example.ASC", 10);

        for (int i = 0; i < pathPoints.size(); ++i)
            vrep->setPos(pathAngles[i]);
    }
}

int main(int argc, char** argv)
{
    int dim = 6;

    if (dim == 2)
        planning2D();
    else if (dim == 6)
        planning6D();

    return 0;
}
