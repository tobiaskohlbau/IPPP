#include "Drawing.h"

void Drawing::drawTree(std::vector<shared_ptr<Node>> nodes, cv::Mat &image)
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

void Drawing::drawPath(const shared_ptr<Node> goalNode, cv::Mat &image)
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

void Drawing::drawPath(std::vector<shared_ptr<Node>> nodes, cv::Mat &image)
{
    for (auto& elem : nodes) {
        if (elem->getParent() != NULL) {
            cv::Point point(elem->getX(), elem->getY());
            cv::Point point2(elem->getParent()->getX(), elem->getParent()->getY());
            cv::line(image, point, point2, cv::Scalar(0, 255, 0), 2);
        }
    }

}
