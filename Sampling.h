#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <cstdint>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include "Node.h"
#include "Vec.h"

class Sampling
{
public:
    enum SamplingMethod
    {
        random,
        uniform,
        hammersley,
        halton,
    };

    Sampling(const int maxSize = 0, const SamplingMethod method = SamplingMethod::random);
    template<uint16_t dim>
    Vec<dim, float> getSample(const int index, const int nbSamples);
    void setMaxWorkspaceSize(const int max);

private:
    int m_maxSize;
    SamplingMethod m_method;
};

Sampling::Sampling(const int maxSize, const SamplingMethod method) {
    m_method = method;
    m_maxSize = maxSize;
    srand(time(NULL));
}

template<uint16_t dim>
Vec<dim, float> Sampling::getSample(const int index, const int nbSamples) {
    Vec<dim, float> vec;
    if (m_method == SamplingMethod::random)
    {
        for (uint16_t i = 0; i < dim; ++i)
            vec[i] = (float)(rand() % m_maxSize);
        return vec;
    }
    return vec;
}

void Sampling::setMaxWorkspaceSize(const int max) {
    m_maxSize = max;
}

#endif /* SAMPLING_H_ */
