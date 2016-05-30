#include "CollisionDetection.h"

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[out] possibility of collision
*  \date       2016-05-25
*/
bool CollisionDetection::controlCollision(const std::shared_ptr<Node> &node) {
    if (node->getDim() == 2)
        return controlCollision2D(node->getX(), node->getY());
    else
        return false;
}

/*!
*  \brief      Check for 2D collision
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[out] possibility of collision
*  \date       2016-05-25
*/
bool CollisionDetection::controlCollision2D(const float &x, const float &y) {
    if (m_workspace.empty())
        return false;

    uint8_t value = m_workspace.at<uint8_t>((int)y, (int)x);
    if (value != 0)
        return false;
    else
        return true;
}

/*!
*  \brief      Set workspace for 2D collision check
*  \author     Sascha Kaden
*  \param[in]  workspace
*  \date       2016-05-25
*/
void CollisionDetection::set2DWorkspace(cv::Mat &space) {
    if (space.empty())
        return;
    m_workspace = space.clone();
}

/*!
*  \brief      Return workspace for 2D collision
*  \author     Sascha Kaden
*  \param[out] workspace
*  \date       2016-05-25
*/
cv::Mat CollisionDetection::get2DWorkspace() const {
    return m_workspace;
}
