#include <cstdint>
#include <iostream>

#include "opencv2/core/core.hpp"

#include <memory>
#include <ctime>
#include <planner/NormalRRTPlanner.h>
#include <planner/StarRRTPlanner.h>
#include <ui/Drawing.h>
#include <vrep/Helper.h>

void planning2D() {
    const unsigned int dim = 2;

    cv::Mat freeWorkspace, obstacleWorkspace;
    freeWorkspace = cv::imread("spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    obstacleWorkspace = cv::imread("spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);

    StarRRTPlanner planner(dim, 50.0, TrajectoryMethod::linear, SamplingMethod::randomly);
    NormalRRTPlanner planner2(dim, 50.0, TrajectoryMethod::linear, SamplingMethod::randomly);

    // set properties to the planner
    Vec<float> minBoundary(0.0,0.0);
    Vec<float> maxBoundary(1000.0,1000.0);
    planner.setWorkspaceBoundaries(minBoundary, maxBoundary);
    planner.set2DWorkspace(obstacleWorkspace); // only be used by 2D
    planner.setInitNode(Node(10.0, 10.0));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(2000);
    clock_t end = std::clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "computation time: " << elapsed_secs << std::endl;

    std::vector<std::shared_ptr<Node>> nodes = planner.getTree();

    // draw the result, if 2D is used
    cv::Mat image = obstacleWorkspace.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    Node goal(650.0,750.0);
    bool connected = planner.connectGoalNode(goal);
    if (connected) {
        Drawing::drawTree(nodes, image, Vec<uint8_t>(0,0,255), Vec<uint8_t>(0,0,0), 1);
        std::vector<Vec<float>> pathPoints = planner.getPath();
        Drawing::drawPath(pathPoints, image, Vec<uint8_t>(255,0,0), 3);

        //shared_ptr<Node> goalNode = planner.getGoalNode();
        //Drawing::drawPath(goalNode, image, Vec<uint8_t>(0,0,255), Vec<uint8_t>(0,255,0), 2);
    }
    else {
        Drawing::drawTree(nodes, image, Vec<uint8_t>(0,0,255), Vec<uint8_t>(0,0,0), 1);
    }
    cv::namedWindow("planner", CV_WINDOW_AUTOSIZE);
    cv::imshow("planner", image);
    cv::waitKey(0);
}

void planning6D() {
    const unsigned int dim = 6;
    Helper vrep(dim);

    vrep.start();

    Vec<float> pos(90.0,90.0,90.0,90.0,90.0,90.0);
    vrep.setPos(pos);

    pos = Vec<float>(1.0,1.0,1.0,1.0,1.0,1.0);
    vrep.setPos(pos);
    pos = Vec<float>(-10.0, -10.0, -10.0, -10.0, -100.0, -100.0);
    vrep.setPos(pos);



}

int main(int argc, char** argv)
{
    const unsigned int dim = 2;

    if (dim == 2)
        planning2D();
    else if (dim == 6)
        planning6D();

    return 0;
}
