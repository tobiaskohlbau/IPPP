#include <core/Sampling.h>

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  SamplingMethod
*  \date       2016-05-24
*/
Sampling::Sampling(const SamplingMethod &method) {
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
Vec<float> Sampling::getSample(const unsigned int &dim, const int index, const int nbSamples) {
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
        std::cout << "Boudaries have different dimensions" << std::endl;
        return;
    }
    for (unsigned int i = 0; i < minBoundary.getDim(); ++i) {
        if (minBoundary[i] > maxBoundary[i]) {
            std::cout << "Min boundary is larger than max boundary" << std::endl;
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
        std::cout << "Boundaries are empty" << std::endl;
        return false;
    }
    return true;
}
