#include <core/Sampling.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  SamplingMethod
*  \date       2016-05-24
*/
Sampling::Sampling(const SamplingMethod &method)
    : Base("Sampling") {
    m_method = method;
    srand(time(NULL));
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[in]  index from loop of samples
*  \param[in]  absolute number of samples, which will be created
*  \param[out] Vec
*  \date       2016-05-24
*/
Vec<float> Sampling::getSample(const unsigned int &dim, const int &index, const int &nbSamples) {
    Vec<float> vec(dim);
    if (!checkBoudaries())
        return vec;

    if (true)//m_method == SamplingMethod::randomly)
    {
        for (unsigned int i = 0; i < dim; ++i)
            vec[i] = m_minBoundary[i] + (float)(rand() % (int)(m_maxBoundary[i]-m_minBoundary[i]));
        return vec;
    }
    return vec;
}

/*!
*  \brief      Set minimum and maximum boundaries of configuration space
*  \author     Sascha Kaden
*  \param[in]  minimum boundaries
*  \param[in]  maximum boundaries
*  \date       2016-05-24
*/
void Sampling::setBoundaries(const Vec<float> &minBoundary, const Vec<float> &maxBoundary) {
    if (minBoundary.getDim() != maxBoundary.getDim()) {
        this->sendMessage("Boudaries have different dimensions");
        return;
    }
    for (unsigned int i = 0; i < minBoundary.getDim(); ++i) {
        if (minBoundary[i] > maxBoundary[i]) {
            this->sendMessage("Min boundary is larger than max boundary");
            return;
        }
    }
    m_maxBoundary = maxBoundary;
    m_minBoundary = minBoundary;
}

/*!
*  \brief      Check boundary existence
*  \author     Sascha Kaden
*  \param[out] result of check
*  \date       2016-05-24
*/
bool Sampling::checkBoudaries() {
    if (m_maxBoundary.empty() || m_minBoundary.empty()) {
        this->sendMessage("Boundaries are empty");
        return false;
    }
    return true;
}
