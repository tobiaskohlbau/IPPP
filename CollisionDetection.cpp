#include "CollisionDetection.h"

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
