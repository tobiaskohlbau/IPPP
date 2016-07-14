#ifndef ROBOTBASE_H_
#define ROBOTBASE_H_

#include <vector>
#include <string>

#include <Eigen/Core>
#include <PQP/include/PQP.h>

#include <core/Base.h>
#include <core/Vec.hpp>
#include <robot/CadFileLoader.h>

namespace rmpl {

enum RobotType
{
    JACO,
    POINT_ROBOT
};

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
class RobotBase : public Base
{
public:
    RobotBase(std::string name, RobotType type, unsigned int dim, unsigned int numberJoints);

    virtual Vec<float> directKinematic(const Vec<float> &angles) = 0;
    virtual std::vector<Eigen::Matrix4f> getTransformations(const Vec<float> &angles) = 0;

    unsigned int getDim() const;
    unsigned int getNbJoints() const;
    std::string getName() const;
    RobotType getType() const;

    Eigen::Matrix4f getTrafo(float alpha, float a, float d, const float q);
    Vec<float> getTcpPosition(const std::vector<Eigen::Matrix4f> &trafos, const Vec<float> basis);

    bool setCadModels(const std::vector<std::string> &files);
    std::shared_ptr<PQP_Model> getCadModel(unsigned int index);

    Vec<float> degToRad(const Vec<float> deg);
    Eigen::ArrayXf VecToEigen(const Vec<float> &vec);
    Vec<float> EigenToVec(const Eigen::ArrayXf &eigenVec);

protected:
    std::shared_ptr<CadFileLoader> m_fileLoader;

    std::string  m_robotName;
    RobotType    m_robotType;
    unsigned int m_nbJoints;
    unsigned int m_dim;

    Vec<float> m_alpha;
    Vec<float> m_a;
    Vec<float> m_d;
    std::vector<std::string> m_cadFiles;
    std::vector<std::shared_ptr<PQP_Model>> m_cadModels;

    float        m_pi = 3.1416;
};

} /* namespace rmpl */

#endif /* ROBOTBASE_H_ */
