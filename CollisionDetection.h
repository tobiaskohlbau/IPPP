#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include <cstdint>
#include <memory>

#ifdef __linux__
    #include "opencv2/core/core.hpp"
#else
    #include <opencv2/core.hpp>
#endif

#include "Node.h"

class CollisionDetection
{
public:
    bool controlCollision(const std::shared_ptr<Node> node);
    bool controlCollision2D(const float x, const float y);

    void set2DWorkspace(cv::Mat space);
    cv::Mat get2DWorkspace() const;
private:
    cv::Mat m_workspace;
};

bool CollisionDetection::controlCollision(const std::shared_ptr<Node> node) {
    if (node->getDim() == 2)
        return controlCollision2D(node->getX(), node->getY());
    else
        return false;
}

bool CollisionDetection::controlCollision2D(const float x, const float y) {
    if (m_workspace.empty())
        return false;

    uint8_t value = m_workspace.at<uint8_t>((int)y, (int)x);
    if (value != 0)
        return false;
    else
        return true;
}

void CollisionDetection::set2DWorkspace(cv::Mat space) {
    if (space.empty())
        return;
    m_workspace = space;
}

cv::Mat CollisionDetection::get2DWorkspace() const {
    return m_workspace;
}

#endif /* COLLISIONDETECTION_H_ */
