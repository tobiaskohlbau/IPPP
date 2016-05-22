#ifndef VEC_H_
#define VEC_H_

#include <assert.h>
#include <cstdint>
#include <math.h>

using std::shared_ptr;

template<unsigned int dim, typename T>
class Vec
{
public:
    Vec();
    Vec(const T &x);
    Vec(const T &x, const T &y);
    Vec(const T &x, const T &y, const T &z);
    Vec(const T &x, const T &y, const T &z, const T &rx);
    Vec(const T &x, const T &y, const T &z, const T &rx, const T &ry);
    Vec(const T &x, const T &y, const T &z, const T &rx, const T &ry, const T &rz);
    Vec(const T data[]);

    void setAllTo(const T &value);

    T getValue(unsigned int index);
    bool empty() const;
    T norm() const;
    Vec<dim, T> abs() const;
    T getDist(const Vec<dim, T> &vec) const;
    T getSqDist(const Vec<dim, T> &vec) const;

    inline T& operator [] (const unsigned int index) {
        assert (index < dim);
        return m_data[index];
    }

    inline const T& operator [] (const unsigned int index) const {
        assert (index < dim);
        return m_data[index];
    }

    Vec<dim, T>& operator += (const Vec<dim, T> &vec) {
        for (unsigned int i = 0; i < dim; ++i)
            m_data[i] += vec[i];
        return *this;
    }

    Vec<dim, T>& operator += (const T scalar) {
        for (unsigned int i = 0; i < dim; ++i)
            m_data[i] += scalar;
        return *this;
    }

    Vec operator + (const Vec<dim, T> &v) const {
        Vec result = Vec<dim, T>::zeros();
        for (unsigned int i = 0; i < dim; ++i)
            result[i] = (*this)[i] + v[i];
        return result;
    }

    Vec& operator -= (const Vec<dim, T> &v) {
        for (unsigned int i = 0; i < dim; ++i)
            (*this)[i] -= v[i];
        return *this;
    }

    Vec& operator -= (const T scalar) {
        for (unsigned int i = 0; i < dim; ++i)
            (*this)[i] -= scalar;
        return *this;
    }

    Vec operator - (const Vec<dim, T> &v) const {
        Vec result = Vec<dim, T>::zeros();
        for (unsigned int i = 0; i < dim; ++i)
            result[i] = (*this)[i] - v[i];
        return result;
    }

    Vec operator + (const T &s) const {
        Vec<dim, T> result(*this);
        for (unsigned int i = 0; i < dim; ++i)
            result[i] += s;
        return result;
    }

    Vec operator - (const T &s) const {
        Vec<dim, T> result(*this);
        for (unsigned int i = 0; i < dim; ++i)
            result[i] -= s;
        return result;
    }

    Vec& operator *= (const T s) {
        for (unsigned int i = 0; i < dim; ++i)
            (*this)[i] *= s;
        return *this;
    }

    Vec operator * (const T &s) const {
        Vec result = Vec<dim, T>::zeros();
        for (unsigned int i = 0; i < dim; ++i)
            result[i] = (*this)[i] * s;
        return result;
    }

    T operator * (const Vec<dim, T> &v) const {
        T result = 0;
        for (unsigned int i = 0; i < dim; ++i)
            result += (*this)[i] * v[i];
        return result;
    }

    Vec& operator /= (const T &s) {
        for (unsigned int i = 0; i < dim; ++i)
            (*this)[i] /= s;
        return *this;
    }

    Vec operator / (const T &s) const {
        Vec result = Vec<dim, T>::zeros();
        for (unsigned int i = 0; i < dim; ++i)
            result[i] = (*this)[i] / s;
        return result;
    }

    static Vec<dim, T> zeros() {
        Vec<dim, T> vec;
        vec.setAllTo(0);
        return vec;
    }

    static Vec<dim, T> ones() {
        Vec<dim, T> vec;
        vec.setAllTo(1);
        return vec;
    }

private:
    T m_data[dim];
};

template<unsigned int dim, typename T>
Vec<dim, T>::Vec() {
    for (unsigned int i = 0; i < dim; ++i)
        m_data[i] = NAN;
}

template<unsigned int dim, typename T>
Vec<dim, T>::Vec(const T &x) {
    assert(dim == 1);
    m_data[0] = x;
}

template<unsigned int dim, typename T>
Vec<dim, T>::Vec(const T &x, const T &y) {
    assert(dim == 2);
    m_data[0] = x;
    m_data[1] = y;
}

template<unsigned int dim, typename T>
Vec<dim, T>::Vec(const T &x, const T &y, const T &z) {
    assert(dim == 3);
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

template<unsigned int dim, typename T>
Vec<dim, T>::Vec(const T &x, const T &y, const T &z, const T &rx) {
    assert(dim == 4);
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
    m_data[3] = rx;
}

template<unsigned int dim, typename T>
Vec<dim, T>::Vec(const T &x, const T &y, const T &z, const T &rx, const T &ry) {
    assert(dim == 5);
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
    m_data[3] = rx;
    m_data[4] = ry;
}

template<unsigned int dim, typename T>
Vec<dim, T>::Vec(const T &x, const T &y, const T &z, const T &rx, const T &ry, const T &rz) {
    assert(dim == 6);
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
    m_data[3] = rx;
    m_data[4] = ry;
    m_data[5] = rz;
}

template<unsigned int dim, typename T>
Vec<dim, T>::Vec(const T data[]) {
    for (unsigned int i = 0; i < dim; ++i)
        (*this)[i] = data[i];
}

template<unsigned int dim, typename T>
void Vec<dim, T>::setAllTo(const T &value) {
    for (unsigned int i = 0; i < dim; ++i)
        (*this)[i] = value;
}

template<unsigned int dim, typename T>
T Vec<dim, T>:: getValue(unsigned int index){
    return m_data[index];
}

template<unsigned int dim, typename T>
bool Vec<dim, T>::empty() const{
    for (unsigned int i = 0; i < dim; ++i)
        if ((*this)[i] == NAN)
            return true;
    return false;
}

template<unsigned int dim, typename T>
T Vec<dim, T>::norm() const {
    T norm = 0;
    for (unsigned int i = 0; i < dim; ++i)
        norm += (*this)[i] * (*this)[i];
    return sqrtf(norm);
}

template<unsigned int dim, typename T>
Vec<dim, T> Vec<dim, T>::abs() const {
    Vec<dim, T> result;
    for (unsigned int i = 0; i < dim; ++i)
        result[i] = fabs((*this)[i]);
    return result;
}

template<unsigned int dim, typename T>
T Vec<dim, T>::getDist(const Vec<dim, T> &vec) const{
    return sqrtf(getSqDist(vec));
}

template<unsigned int dim, typename T>
T Vec<dim, T>::getSqDist(const Vec<dim, T> &vec) const{
    T dist = 0;
    for (unsigned int i = 0; i < dim; ++i)
        dist += ((*this)[i] - vec[i]) * ((*this)[i] - vec[i]);
    return dist;
}

#endif /* VEC_H_ */
