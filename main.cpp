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
#include "NormalRRTPlanner.h"
#include "StarRRTPlanner.h"
#include "Drawing.h"


int main(int argc, char** argv)
{
    cv::Mat freeWorkspace, obstacleWorkspace, obstacleFullWorkspace;
    if (OSPROJECT == "windows")
    {
        freeWorkspace = cv::imread("../spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
        obstacleWorkspace = cv::imread("../spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else
    {
        freeWorkspace = cv::imread("spaces/freeWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
        obstacleWorkspace = cv::imread("spaces/obstacleWorkspace.png", CV_LOAD_IMAGE_GRAYSCALE);
    }

    cv::Mat workspace = freeWorkspace;
    const uint16_t dim = 2;
    StarRRTPlanner<dim> planner(70.0);
    NormalRRTPlanner<dim> planner2(70.0);
    planner.setWorkspace(workspace);
    planner.setInitNode(Node<dim>(10.0, 10.0));
    //planner.setInitNode(Node<dim>(10, 10, 10 ,10, 10 ,10));
    planner.computeTree(1000);

    //cv::Mat image = cv::Mat(40, 40, CV_8U);
    std::vector<std::shared_ptr<Node<dim>>> nodes;
    planner.getTree(nodes);

    if (dim == 2) {
        cv::Mat image = workspace.clone();
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


    return 0;
}
