#include "Sampling.h"

Vec<float> Sampling::getSample(const unsigned int &dim, const int index, const int nbSamples) {
    Vec<float> vec(dim);
    if (!controlBoudaries())
        return vec;

    if (true)//m_method == SamplingMethod::randomly)
    {
        for (unsigned int i = 0; i < dim; ++i)
            vec[i] = m_minBoundary[i] + (float)(rand() % (int)(m_maxBoundary[i]-m_minBoundary[i]));
        return vec;
    }
    return vec;
}

void Sampling::setBoundaries(const Vec<float> &minBoundary, const Vec<float> &maxBoundary) {
    m_maxBoundary = maxBoundary;
    m_minBoundary = minBoundary;
}

bool Sampling::controlBoudaries() {
    if (m_maxBoundary.empty() || m_minBoundary.empty()) {
        std::cout << "No boundaries set" << std::endl;
        return false;
    }
    return true;
}
