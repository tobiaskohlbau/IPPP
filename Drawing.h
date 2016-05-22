#ifndef DRAWING_H_
#define DRAWING_H_


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#include <assert.h>
#include <cstdint>
#include <memory>
#include "Graph.h"

using std::shared_ptr;

class Drawing
{
public:

    static void drawTree(std::vector<shared_ptr<Node>> nodes, cv::Mat &image)
    {
        assert(nodes[0]->getDim() == 2);
        for (auto& elem : nodes) {
            cv::Point point(elem->getX(), elem->getY());
            cv::circle(image, point, 3, cv::Scalar(0, 0, 255), 2);
            if (elem->getParent() != NULL) {
                cv::Point point2(elem->getParent()->getX(), elem->getParent()->getY());
                cv::line(image, point, point2, cv::Scalar(0, 0, 0), 1);
            }
        }
    }

    static void drawPath(const shared_ptr<Node> goalNode, cv::Mat &image)
    {
        assert(goalNode->getDim() == 2);
        // build vector for Drawing
        std::vector<shared_ptr<Node>> nodes;
        nodes.push_back(goalNode);
        shared_ptr<Node> temp = goalNode->getParent();
        while(temp != NULL) {
            nodes.push_back(temp);
            temp = temp->getParent();
        }
        drawPath(nodes, image);
    }

    static void drawPath(std::vector<shared_ptr<Node>> nodes, cv::Mat &image)
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
