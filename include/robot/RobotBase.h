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

enum CollisionType
{
    vrep,
    pqp,
    twoD
};

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
class RobotBase : public Base
{
public:
    RobotBase(std::string name, CollisionType type, unsigned int dim, unsigned int numberJoints);

    virtual Vec<float> directKinematic(const Vec<float> &angles) = 0;
    virtual std::vector<Eigen::Matrix4f> getTransformations(const Vec<float> &angles) = 0;
    Eigen::Matrix4f getTrafo(float alpha, float a, float d, float q);
    Vec<float> getTcpPosition(const std::vector<Eigen::Matrix4f> &trafos, const Vec<float> basis);

    void setBoundaries(const Vec<float> &minBoundary, const Vec<float> &maxBoundary);
    Vec<float> getMinBoundary();
    Vec<float> getMaxBoundary();

    bool setCadModels(const std::vector<std::string> &files);
    std::shared_ptr<PQP_Model> getCadModel(unsigned int index);
    bool setWorkspace(const std::string &workspaceFile);
    std::shared_ptr<PQP_Model> getWorkspace();
    bool set2DWorkspace(const Eigen::MatrixXi &space);
    Eigen::MatrixXi& get2DWorkspace();

    unsigned int getDim();
    unsigned int getNbJoints();
    std::string getName();
    CollisionType getCollisionType();

    Vec<float> degToRad(const Vec<float> deg);
    Eigen::ArrayXf VecToEigen(const Vec<float> &vec);
    Vec<float> EigenToVec(const Eigen::ArrayXf &eigenVec);

protected:
    std::shared_ptr<CadFileLoader> m_fileLoader;

    std::string   m_robotName;
    CollisionType m_collisionType;
    unsigned int  m_nbJoints;
    unsigned int  m_dim;

    Vec<float> m_minBoundary;
    Vec<float> m_maxBoundary;
    Vec<float> m_alpha;
    Vec<float> m_a;
    Vec<float> m_d;

    std::vector<std::string> m_cadFiles;
    std::vector<std::shared_ptr<PQP_Model>> m_cadModels;
    std::string m_workspaceFile;
    std::shared_ptr<PQP_Model> m_workspaceCad;
    Eigen::MatrixXi m_2DWorkspace;

    float m_pi = 3.1416;
};

} /* namespace rmpl */

#endif /* ROBOTBASE_H_ */
