#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <core/Base.h>
#include <core/Vec.hpp>
#include <robot/RobotBase.h>

namespace rmpl{

enum SamplingMethod
{
    randomly,
    uniform,
    hammersley,
    halton
};

/*!
* \brief   Class Sampling creates samples of new vectors by a given method
* \author  Sascha Kaden
* \date    2016-05-23
*/
class Sampling : public Base
{
public:
    Sampling(const std::shared_ptr<RobotBase> &robot, SamplingMethod method = SamplingMethod::randomly);
    Vec<float> getSample(unsigned int dim, int index, int nbSamples);

private:
    bool checkBoudaries();

    Vec<float> m_minBoundary;
    Vec<float> m_maxBoundary;
    SamplingMethod m_method;
    std::shared_ptr<RobotBase> m_robot;
};

} /* namespace rmpl */

#endif /* SAMPLING_H_ */
