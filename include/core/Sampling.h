#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <cstdint>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include <core/Node.h>
#include <core/Vec.hpp>

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
class Sampling
{
public:
    Sampling(const SamplingMethod &method = SamplingMethod::randomly);
    Vec<float> getSample(const unsigned int &dim, const int index, const int nbSamples);
    void setBoundaries(const Vec<float> &maxBoundary, const Vec<float> &minBoundary);

private:
    bool checkBoudaries();

    Vec<float> m_maxBoundary;
    Vec<float> m_minBoundary;
    SamplingMethod m_method;
};

} /* namespace rmpl */

#endif /* SAMPLING_H_ */
