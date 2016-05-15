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
    template<uint16_t dim>
    bool controlCollision(const std::shared_ptr<Node<dim>> node);

    bool controlCollision2D(const float x, const float y);
    void setWorkspace(cv::Mat space);
    cv::Mat getWorkspace();
private:
    cv::Mat m_workspace;
};

void CollisionDetection::setWorkspace(cv::Mat space) {
    if (space.empty())
        return;
    m_workspace = space;
}

cv::Mat CollisionDetection::getWorkspace() {
    return m_workspace;
}

template<uint16_t dim>
bool CollisionDetection::controlCollision(const std::shared_ptr<Node<dim>> node) {
    if (dim == 2)
        return controlCollision2D(node->getX(), node->getY());
    else
        return false;
}

bool CollisionDetection::controlCollision2D(const float x, const float y) {
    if (m_workspace.empty())
        return false;

    if (x < 0 || x > m_workspace.rows || y < 0 || y > m_workspace.cols)
        return false;

    uint8_t value = m_workspace.at<uint8_t>((int)y, (int)x);
    if (value != 0)
        return false;
    else
        return true;
}

#endif /* COLLISIONDETECTION_H_ */
