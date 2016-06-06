#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include "opencv2/core/core.hpp"

#include <core/Node.h>
#include <core/Base.h>
#include <vrep/Helper.h>

namespace rmpl {

/*!
* \brief   Class CollisionDetection checks the configuration on collision and return boolean value
* \author  Sascha Kaden
* \date    2016-05-25
*/
class CollisionDetection : public Base
{
public:
    CollisionDetection(const std::shared_ptr<Helper> &vrep);
    bool controlCollision(const std::shared_ptr<Node> &node);
    bool controlCollision(const Vec<float> &vec);
    bool controlCollision(const std::vector<Vec<float>> &vec);
    bool controlCollision2D(const float &x, const float &y);
    bool controlCollision6D(const Vec<float> &vec);
    bool controlCollision6D(const std::vector<Vec<float>> &vec);

    void set2DWorkspace(cv::Mat &space);
    cv::Mat get2DWorkspace() const;
private:
    cv::Mat m_workspace;
    std::shared_ptr<Helper>  m_vrep;
};

} /* namespace rmpl */

#endif /* COLLISIONDETECTION_H_ */
