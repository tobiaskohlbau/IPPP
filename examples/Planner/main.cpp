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

#include <vrep/Helper.h>

#include "ui/Drawing.h"

void printTime(clock_t begin, clock_t end) {
    float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Computation time: " << elapsed_secs << std::endl;
}

void planning2D() {
    cv::Mat freeWorkspace, obstacleWorkspace;
    freeWorkspace = cv::imread("/home/sascha/projects/Planner/spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    obstacleWorkspace = cv::imread("/home/sascha/projects/Planner/spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);

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
    robot->set2DWorkspace(mat);
    rmpl::Vec<rmpl::REAL> minBoundary(0.0,0.0);
    rmpl::Vec<rmpl::REAL> maxBoundary(rows, cols);
    robot->setBoundaries(minBoundary, maxBoundary);

    rmpl::StarRRTPlanner planner(robot, 30.0, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    rmpl::NormalRRTPlanner planner2(robot, 30.0, 0.5, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);

    // compute the tree
    clock_t begin = std::clock();
    planner.setInitNode(rmpl::Node(10.0, 10.0));
    planner.computeTree(10000);
    clock_t end = std::clock();
    printTime(begin, end);

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();

    cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    rmpl::Node goal(650.0,750.0);
    bool connected = planner.connectGoalNode(goal);
    Drawing::drawTree2D(nodes, image, rmpl::Vec<uint8_t>(0,0,255), rmpl::Vec<uint8_t>(0,0,0), 1);

    if (connected) {
        std::vector<rmpl::Vec<rmpl::REAL>> pathPoints = planner.getPath();
        Drawing::drawPath2D(pathPoints, image, rmpl::Vec<uint8_t>(255,0,0), 3);
    }

    cv::namedWindow("planner", CV_WINDOW_AUTOSIZE);
    cv::imshow("planner", image);
    cv::waitKey(0);
}

void simpleRRT() {
    std::shared_ptr<rmpl::Jaco> robot(new rmpl::Jaco());
    rmpl::Vec<rmpl::REAL> minBoundary(0, 42, 17, 0, 0, 0);
    rmpl::Vec<rmpl::REAL> maxBoundary(360, 318, 343, 360, 360 ,360);
    robot->setBoundaries(minBoundary, maxBoundary);

    rmpl::StarRRTPlanner planner(robot, 10, 0.2, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    //std::shared_ptr<rmpl::Helper> vrep = planner.getVrep();

    planner.setInitNode(rmpl::Node(180, 180, 180, 180, 180, 180));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(30000,2);
    clock_t end = std::clock();
    printTime(begin, end);

    bool connected = planner.connectGoalNode(rmpl::Node(275, 167.5, 57.4, 241, 82.7, 75.5));

    std::vector<std::shared_ptr<rmpl::Node>> nodes = planner.getGraphNodes();
    std::vector<rmpl::Vec<rmpl::REAL>> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getVec()));
    Drawing::writeVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        std::vector<rmpl::Vec<rmpl::REAL>> pathAngles = planner.getPath();

        std::vector<rmpl::Vec<rmpl::REAL>> pathPoints;
        for (int i = 0; i < pathAngles.size(); ++i)
            pathPoints.push_back(robot->directKinematic(pathAngles[i]));
        Drawing::appendVecsToFile(pathPoints, "example.ASC", 10);

        //for (int i = 0; i < pathPoints.size(); ++i)
        //    vrep->setPos(pathAngles[i]);
    }
}

void treeConnection() {
    std::shared_ptr<rmpl::Jaco> robot(new rmpl::Jaco());
    rmpl::Vec<rmpl::REAL> minBoundary(0, 42, 17, 0, 0, 0);
    rmpl::Vec<rmpl::REAL> maxBoundary(360, 318, 343, 360, 360 ,360);
    robot->setBoundaries(minBoundary, maxBoundary);

    // create two trees from init and from goal
    rmpl::StarRRTPlanner plannerInitNode(robot, 10, 0.2, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    rmpl::StarRRTPlanner plannerGoalNode(robot, 10, 0.2, rmpl::TrajectoryMethod::linear, rmpl::SamplingMethod::randomly);
    //std::shared_ptr<rmpl::Helper> vrep = planner.getVrep();

    // set properties to the plannerss
    plannerInitNode.setInitNode(rmpl::Node(180, 180, 180, 180, 180, 180));
    plannerGoalNode.setInitNode(rmpl::Node(275, 167.5, 57.4, 241, 82.7, 75.5));

    // compute the tree
    clock_t begin = std::clock();
    plannerInitNode.computeTree(20000,2);
    plannerGoalNode.computeTree(20000,2);
    clock_t end = std::clock();
    printTime(begin, end);

    // get random sample from the first planner and try to connect to both planners
    rmpl::Node goal;
    bool connected = false;
    rmpl::REAL minCost = std::numeric_limits<rmpl::REAL>::max();
    for (int i = 0; i < 10000; ++i){
        rmpl::Vec<rmpl::REAL> sample = plannerInitNode.getSamplePoint();
        //sample.print();
        rmpl::Node node(sample);
        bool planner1Connected = plannerInitNode.connectGoalNode(node);
        bool planner2Connected = plannerGoalNode.connectGoalNode(node);
        if (planner1Connected && planner2Connected) {
            rmpl::REAL cost = plannerInitNode.getGoalNode()->getCost() + plannerGoalNode.getGoalNode()->getCost();
            if (cost < minCost) {
                goal = node;
                connected = true;
            }
        }
    }

    std::vector<std::shared_ptr<rmpl::Node>> nodes = plannerInitNode.getGraphNodes();
    std::vector<rmpl::Vec<rmpl::REAL>> graphPoints;
    std::cout << "Init Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getVec()));
    Drawing::writeVecsToFile(graphPoints, "example.ASC", 10);

    nodes = plannerGoalNode.getGraphNodes();
    std::cout << "Goal Graph has: " << nodes.size() << "nodes" << std::endl;
    for (int i = 0; i < nodes.size(); ++i)
        graphPoints.push_back(robot->directKinematic(nodes[i]->getVec()));
    Drawing::appendVecsToFile(graphPoints, "example.ASC", 10);

    if (connected) {
        std::cout << "Init and goal could be connected!" << std::endl;
        plannerInitNode.connectGoalNode(goal);
        plannerGoalNode.connectGoalNode(goal);

        std::vector<rmpl::Vec<rmpl::REAL>> pathAngles = plannerInitNode.getPath();
        std::vector<rmpl::Vec<rmpl::REAL>> temp = plannerGoalNode.getPath();
        pathAngles.insert(pathAngles.end(), temp.begin(), temp.end());

        std::vector<rmpl::Vec<rmpl::REAL>> pathPoints;
        for (int i = 0; i < pathAngles.size(); ++i)
            pathPoints.push_back(robot->directKinematic(pathAngles[i]));
        Drawing::appendVecsToFile(pathPoints, "example.ASC", 10);

        //for (int i = 0; i < pathPoints.size(); ++i)
        //    vrep->setPos(pathAngles[i]);
    }
}

int main(int argc, char** argv)
{

    //planning2D();
    //treeConnection();
    simpleRRT();

    return 0;
}
