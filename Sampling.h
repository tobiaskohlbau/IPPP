#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <cstdint>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include "Node.h"
#include "Vec.h"

enum SamplingMethod
{
    randomly,
    uniform,
    hammersley,
    halton
};

template<unsigned int dim>
class Sampling
{
public:
    Sampling(const SamplingMethod method = SamplingMethod::randomly);
    Vec<dim, float> getSample(const int index, const int nbSamples);
    void setBoundaries(const Vec<dim, float> &maxBoundary, const Vec<dim, float> &minBoundary);

private:
    Vec<dim, float> m_maxBoundary;
    Vec<dim, float> m_minBoundary;
    SamplingMethod m_method;
};

template<unsigned int dim>
Sampling<dim>::Sampling(const SamplingMethod method) {
    m_method = method;
    srand(time(NULL));
}

template<unsigned int dim>
Vec<dim, float> Sampling<dim>::getSample(const int index, const int nbSamples) {
    Vec<dim, float> vec;
    if (m_method == SamplingMethod::randomly)
    {
        for (unsigned int i = 0; i < dim; ++i)
            vec[i] = m_minBoundary[i] + (float)(rand() % (int)(m_maxBoundary[i]-m_minBoundary[i]));
        return vec;
    }
    return vec;
}

template<unsigned int dim>
void Sampling<dim>::setBoundaries(const Vec<dim, float> &maxBoundary, const Vec<dim, float> &minBoundary) {
    m_maxBoundary = maxBoundary;
    m_minBoundary = minBoundary;
}

#endif /* SAMPLING_H_ */
