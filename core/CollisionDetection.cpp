#include <core/CollisionDetection.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  VREP Helper
*  \date       2016-06-02
*/
CollisionDetection::CollisionDetection(const std::shared_ptr<Helper> &vrep)
        : Base("CollisionDetection") {
    m_vrep = vrep;
}

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
    else if (node->getDim() == 6)
        return controlCollision6D(node->getVec());
    else
        return false;
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  vec
*  \param[out] possibility of collision
*  \date       2016-05-25
*/
bool CollisionDetection::controlCollision(const Vec<float> &vec) {
    if (vec.getDim() == 2)
        return controlCollision2D(vec[0], vec[1]);
    else if (vec.getDim() == 6)
        return controlCollision6D(vec);
    else
        return false;
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  vector of points
*  \param[out] possibility of collision
*  \date       2016-05-25
*/
bool CollisionDetection::controlCollision(const std::vector<Vec<float>> &vecs) {
    if (vecs.size() == 0)
        return false;

    if (vecs[0].getDim() == 2)
        for (int i = 0; i < vecs.size(); ++i)
            if (controlCollision2D(vecs[i][0], vecs[i][1]))
                return true;
    else if (vecs[0].getDim() == 6)
        return controlCollision6D(vecs);
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

    uint8_t value = m_workspace.at<uint8_t>((unsigned int)y, (unsigned int)x);
    if (value != 0)
        return false;
    else
        return true;
}

bool CollisionDetection::controlCollision6D(const Vec<float> &vec) {
    assert(vec.getDim() == 6);
    return m_vrep->checkCollision(vec);
}

bool CollisionDetection::controlCollision6D(const std::vector<Vec<float>> &vecs) {
    assert(vecs[0].getDim() == 6);
    return m_vrep->checkCollision(vecs);
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
