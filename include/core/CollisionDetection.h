#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include <cstdint>
#include <memory>
#include "opencv2/core/core.hpp"

#include <core/Node.h>

/*!
* \brief   Class CollisionDetection checks the configuration on collision and return boolean value
* \author  Sascha Kaden
* \date    2016-05-25
*/
class CollisionDetection
{
public:
    bool controlCollision(const std::shared_ptr<Node> &node);
    bool controlCollision(const Vec<float> &vec);
    bool controlCollision2D(const float &x, const float &y);

    void set2DWorkspace(cv::Mat &space);
    cv::Mat get2DWorkspace() const;
private:
    cv::Mat m_workspace;
};

#endif /* COLLISIONDETECTION_H_ */
