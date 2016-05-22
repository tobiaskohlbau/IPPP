#include <cstdint>
#include <iostream>

#ifdef __linux__
    #include "opencv2/imgproc/imgproc.hpp"
    #include "opencv2/highgui/highgui.hpp"
    //#include "opencv2/imgcodecs/imgcodecs.hpp"
    #define OSPROJECT "linux"
#else
    #include <opencv2/imgproc.hpp>
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgcodecs.hpp>
    #define OSPROJECT "windows"
#endif

#include <memory>
#include <ctime>
#include <NormalRRTPlanner.h>
#include <StarRRTPlanner.h>
#include "Drawing.h"
#include "VrepHelper.h"

void planning2D() {
    const unsigned int dim = 2;

    cv::Mat freeWorkspace, obstacleWorkspace;
    if (OSPROJECT == "windows") {
        freeWorkspace = cv::imread("../spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
        obstacleWorkspace = cv::imread("../spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else {
        freeWorkspace = cv::imread("spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
        obstacleWorkspace = cv::imread("spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    }

    StarRRTPlanner<dim> planner(70.0);
    NormalRRTPlanner<dim> planner2(70.0);

    // set properties to the planner
    Vec<dim, float> minBoundary(0,0);
    Vec<dim, float> maxBoundary(1000,1000);
    planner.setWorkspaceBoundaries(minBoundary, maxBoundary);
    planner.set2DWorkspace(obstacleWorkspace); // only be used by 2D
    planner.setInitNode(Node<dim>(10.0, 10.0));

    // compute the tree
    clock_t begin = std::clock();
    planner.computeTree(3000);
    clock_t end = std::clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "computation time: " << elapsed_secs << std::endl;


    std::vector<std::shared_ptr<Node<dim>>> nodes;
    planner.getTree(nodes);

    // draw the result, if 2D is used
    if (dim == 2) {
        cv::Mat image = obstacleWorkspace.clone();
        cv::cvtColor(image, image, CV_GRAY2BGR);

        std::shared_ptr<Node<dim>> goal(new Node<dim>(650,750));
        bool connected = planner.connectGoalNode(goal);
        if (connected) {
            Drawing::drawTree<dim>(nodes, image);
            Drawing::drawPath<dim>(goal, image);
        }
        else {
            Drawing::drawTree<dim>(nodes, image);
        }
        cv::namedWindow("planner", CV_WINDOW_AUTOSIZE);
        cv::imshow("planner", image);
        cv::waitKey(0);
    }
}

void planning6D() {
    const unsigned int dim = 6;
    VrepHelper<dim> vrep;

    vrep.startVrep();

    Vec<dim, float> pos(100,100,100,100,100,100);
    vrep.setPos(pos);

    pos = Vec<dim, float>(-10, -10, -10, -10, -100, -100);
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
