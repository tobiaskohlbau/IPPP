#ifndef DRAWING_H_
#define DRAWING_H_

#ifdef __linux__
    #include "opencv2/imgproc/imgproc.hpp"
    #include "opencv2/highgui/highgui.hpp"
#else
    #include <opencv2/imgproc.hpp>
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgcodecs.hpp>
#endif

#include <cstdint>
#include <memory>
#include "Graph.h"

class Drawing
{
public:

    template<uint16_t dim>
    static void drawTree(std::vector<std::shared_ptr<Node<dim>>> nodes, cv::Mat &image)
    {
        for (auto& elem : nodes) {
            cv::Point point(elem->getX(), elem->getY());
            cv::circle(image, point, 3, cv::Scalar(0, 0, 255), 2);
            if (elem->getParent() != NULL) {
                cv::Point point2(elem->getParent()->getX(), elem->getParent()->getY());
                cv::line(image, point, point2, cv::Scalar(0, 0, 0), 1);
            }
        }
    }

    template<uint16_t dim>
    static void drawPath(const std::shared_ptr<Node<dim>> goalNode, cv::Mat &image)
    {
        // build vector for Drawing
        std::vector<std::shared_ptr<Node<dim>>> nodes;
        nodes.push_back(goalNode);
        std::shared_ptr<Node<dim>> temp = goalNode->getParent();
        while(temp != NULL) {
            nodes.push_back(temp);
            temp = temp->getParent();
        }
        drawPath<dim>(nodes, image);
    }

    template<uint16_t dim>
    static void drawPath(std::vector<std::shared_ptr<Node<dim>>> nodes, cv::Mat &image)
    {
        for (auto& elem : nodes) {
            if (elem->getParent() != NULL) {
                cv::Point point(elem->getX(), elem->getY());
                cv::Point point2(elem->getParent()->getX(), elem->getParent()->getY());
                cv::line(image, point, point2, cv::Scalar(0, 255, 0), 2);
            }
        }

    }
};

#endif /* DRAWING_H_ */
