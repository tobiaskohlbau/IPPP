#include <cstdint>
#include <iostream>

#include "opencv2/core/core.hpp"

#include <memory>
#include <ctime>
#include <planner/NormalRRTPlanner.h>
#include <planner/StarRRTPlanner.h>
#include <robot/Jaco.h>
#include <robot/PointRobot.h>
#include <vrep/Helper.h>

#include "ui/Drawing.h"

void planning2D() {
    const unsigned int dim = 2;

    cv::Mat freeWorkspace, obstacleWorkspace;
    freeWorkspace = cv::imread("spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    obstacleWorkspace = cv::imread("spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);

    std::shared_ptr<rmpl::PointRobot> robot(new rmpl::PointRobot());
    rmpl::StarRRTPlanner planner(robot, 30.0, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    rmpl::NormalRRTPlanner planner2(robot, 30.0, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);

    // set properties to the planner
    rmpl::Vec<float> minBoundary(0.0,0.0);
    rmpl::Vec<float> maxBoundary(1000.0,1000.0);
    planner.setWorkspaceBoundaries(minBoundary, maxBoundary);
    planner.set2DWorkspace(obstacleWorkspace); // only be used by 2D
    planner.setInitNode(rmpl::Node(10.0, 10.0));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(4000);
    clock_t end = std::clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "computation time: " << elapsed_secs << std::endl;

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();

    // draw the result, if 2D is used
    cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    rmpl::Node goal(650.0,750.0);
    bool connected = planner.connectGoalNode(goal);
    if (connected) {
        Drawing::drawTree(nodes, image, rmpl::Vec<uint8_t>(0,0,255), rmpl::Vec<uint8_t>(0,0,0), 1);
        std::vector<rmpl::Vec<float>> pathPoints = planner.getPath();
        Drawing::drawPath(pathPoints, image, rmpl::Vec<uint8_t>(255,0,0), 3);

        //shared_ptr<Node> goalNode = planner.getGoalNode();
        //Drawing::drawPath(goalNode, image, Vec<uint8_t>(0,0,255), Vec<uint8_t>(0,255,0), 2);
    }
    else {
        Drawing::drawTree(nodes, image, rmpl::Vec<uint8_t>(0,0,255), rmpl::Vec<uint8_t>(0,0,0), 1);
    }
    cv::namedWindow("planner", CV_WINDOW_AUTOSIZE);
    cv::imshow("planner", image);
    cv::waitKey(0);
}

void planning3D() {
    const unsigned int dim = 3;

    std::shared_ptr<rmpl::PointRobot> robot(new rmpl::PointRobot());
    rmpl::StarRRTPlanner planner(robot, 50.0, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    rmpl::NormalRRTPlanner planner2(robot, 50.0, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);

    // set properties to the planner
    rmpl::Vec<float> minBoundary(0.0,0.0,0.0);
    rmpl::Vec<float> maxBoundary(1000.0,1000.0,1000.0);
    planner.setWorkspaceBoundaries(minBoundary, maxBoundary);
    planner.setInitNode(rmpl::Node(10.0, 10.0, 10.0));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(3000);
    clock_t end = std::clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "computation time: " << elapsed_secs << std::endl;

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();

    rmpl::Node goal(650.0,750.0,800.0);
    bool connected = planner.connectGoalNode(goal);
    cv::Mat image;
    if (connected) {
        Drawing::drawTree(nodes, image, rmpl::Vec<uint8_t>(0,0,255), rmpl::Vec<uint8_t>(0,0,0), 1);
        std::vector<rmpl::Vec<float>> pathPoints = planner.getPath();
        Drawing::drawPath(pathPoints, image, rmpl::Vec<uint8_t>(255,0,0), 3);
    }
    else {
        Drawing::drawTree(nodes, image, rmpl::Vec<uint8_t>(0,0,255), rmpl::Vec<uint8_t>(0,0,0), 1);
    }
}

void planning6D() {
    const unsigned int dim = 6;
    std::shared_ptr<rmpl::Jaco> robot(new rmpl::Jaco());
    rmpl::StarRRTPlanner planner(robot, 30.0, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    std::shared_ptr<rmpl::Helper> vrep = planner.getVrep();

    // set properties to the planner
    rmpl::Vec<float> minBoundary(0.0,0.0,0.0,0.0,0.0,0.0);
    rmpl::Vec<float> maxBoundary(360.0,360.0,360.0,360.0,360.0,360.0);
    planner.setWorkspaceBoundaries(minBoundary, maxBoundary);
    planner.setInitNode(rmpl::Node(180.0,180.0,180.0,180.0,180.0,180.0));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(10000);
    clock_t end = std::clock();

    rmpl::Node goal(170.0, 280.0, 240.0, 80.0, 100.0, 180.0);
    bool connected = false;
    connected = planner.connectGoalNode(goal);

    if (connected) {
        std::vector<rmpl::Vec<float>> pathPoints = planner.getPath();

        for (int i = 0; i < pathPoints.size(); ++i) {
            vrep->setPos(pathPoints[i]);
        }
    }

}

int main(int argc, char** argv)
{
    const unsigned int dim = 2;

    if (dim == 2)
        planning2D();
    else if (dim == 3)
        planning3D();
    else if (dim == 6)
        planning6D();

    return 0;
}
