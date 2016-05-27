#ifndef VEC_H_
#define VEC_H_

#include <assert.h>
#include <cstdint>
#include <math.h>
#include <iostream>

using std::shared_ptr;

/*!
* \brief   Class Vec present a normal vector
* \author  Sascha Kaden
* \date    2016-05-23
*/
template<typename T>
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
    Vec(const unsigned int &dim);
    Vec(const unsigned int &dim, const T data[]);
    Vec(const Vec<T> &vec);

    void setAllTo(const T &value);

    unsigned int getDim() const;
    bool empty() const;
    T norm() const;
    Vec<T> abs() const;
    T getDist(const Vec<T> &vec) const;
    T getSqDist(const Vec<T> &vec) const;
    void print();

    inline T& operator [] (const unsigned int index) {
        assert (index < m_dim);
        return m_data[index];
    }

    inline const T& operator [] (const unsigned int index) const {
        assert (index < m_dim);
        return m_data[index];
    }

    Vec<T> &operator = (const Vec<T> &vec) {
        if (this == &vec)      // Same object?
              return *this;
        m_dim = vec.getDim();
        this->m_data = std::unique_ptr<T[]>(new T[m_dim]);
        for (unsigned int i = 0; i < m_dim; ++i) {
            this->m_data[i] = vec[i];
        }
        return *this;
    }

    Vec<T>& operator += (const Vec<T> &vec) {
        assert(m_dim == vec.getDim());
        for (unsigned int i = 0; i < m_dim; ++i)
            m_data[i] += vec[i];
        return *this;
    }

    Vec<T>& operator += (const T scalar) {
        for (unsigned int i = 0; i < m_dim; ++i)
            m_data[i] += scalar;
        return *this;
    }

    Vec operator + (const Vec<T> &vec) const {
        assert(m_dim == vec.getDim());
        Vec result = Vec<T>::zeros(m_dim);
        for (unsigned int i = 0; i < m_dim; ++i)
            result[i] = (*this)[i] + vec[i];
        return result;
    }

    Vec& operator -= (const Vec<T> &vec) {
        assert(m_dim == vec.getDim());
        for (unsigned int i = 0; i < m_dim; ++i)
            (*this)[i] -= vec[i];
        return *this;
    }

    Vec& operator -= (const T scalar) {
        for (unsigned int i = 0; i < m_dim; ++i)
            (*this)[i] -= scalar;
        return *this;
    }

    Vec operator - (const Vec<T> &vec) const {
        assert(m_dim == vec.getDim());
        Vec result = Vec<T>::zeros(m_dim);
        for (unsigned int i = 0; i < m_dim; ++i)
            result[i] = (*this)[i] - vec[i];
        return result;
    }

    Vec operator + (const T &s) const {
        Vec<T> result(*this);
        for (unsigned int i = 0; i < m_dim; ++i)
            result[i] += s;
        return result;
    }

    Vec operator - (const T &s) const {
        Vec<T> result(*this);
        for (unsigned int i = 0; i < m_dim; ++i)
            result[i] -= s;
        return result;
    }

    Vec& operator *= (const T s) {
        for (unsigned int i = 0; i < m_dim; ++i)
            (*this)[i] *= s;
        return *this;
    }

    Vec operator * (const T &s) const {
        Vec result = Vec<T>::zeros(m_dim);
        for (unsigned int i = 0; i < m_dim; ++i)
            result[i] = (*this)[i] * s;
        return result;
    }

    T operator * (const Vec<T> &vec) const {
        assert(m_dim == vec.getDim());
        T result = 0;
        for (unsigned int i = 0; i < m_dim; ++i)
            result += (*this)[i] * vec[i];
        return result;
    }

    Vec& operator /= (const T &s) {
        for (unsigned int i = 0; i < m_dim; ++i)
            (*this)[i] /= s;
        return *this;
    }

    Vec operator / (const T &s) const {
        Vec result = Vec<T>::zeros(m_dim);
        for (unsigned int i = 0; i < m_dim; ++i)
            result[i] = (*this)[i] / s;
        return result;
    }

    static Vec<T> zeros(unsigned int dim) {
        Vec<T> vec(dim);
        vec.setAllTo(0);
        return vec;
    }

    static Vec<T> ones(unsigned int dim) {
        Vec<T> vec(dim);
        vec.setAllTo(1);
        return vec;
    }

private:
    std::unique_ptr<T[]> m_data;
    unsigned int m_dim;
};

/*!
*  \brief      Constructor of the class Vec, set dimension to 0
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec() {
    m_dim = 0;
}

/*!
*  \brief      Constructor of the class Vec
*  \author     Sascha Kaden
*  \param[in]  dimension of the Vec
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const unsigned int &dim) {
    m_dim = dim;
    m_data = std::unique_ptr<T[]>(new T[m_dim]);
    for (unsigned int i = 0; i < m_dim; ++i)
        (*this)[i] = NAN;
}

/*!
*  \brief      Constructor of the class Vec (1D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const T &x) {
    m_dim = 1;
    m_data = std::unique_ptr<T[]>(new T[m_dim]);
    m_data[0] = x;
}

/*!
*  \brief      Constructor of the class Vec (2D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const T &x, const T &y) {
    m_dim = 2;
    m_data = std::unique_ptr<T[]>(new T[m_dim]);
    m_data[0] = x;
    m_data[1] = y;
}

/*!
*  \brief      Constructor of the class Vec (3D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const T &x, const T &y, const T &z) {
    m_dim = 3;
    m_data = std::unique_ptr<T[]>(new T[m_dim]);
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

/*!
*  \brief      Constructor of the class Vec (4D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \param[in]  rx
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const T &x, const T &y, const T &z, const T &rx) {
    m_dim = 4;
    m_data = std::unique_ptr<T[]>(new T[m_dim]);
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
    m_data[3] = rx;
}

/*!
*  \brief      Constructor of the class Vec (5D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \param[in]  rx
*  \param[in]  ry
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const T &x, const T &y, const T &z, const T &rx, const T &ry) {
    m_dim = 5;
    m_data = std::unique_ptr<T[]>(new T[m_dim]);
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
    m_data[3] = rx;
    m_data[4] = ry;
}

/*!
*  \brief      Constructor of the class Vec (6D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \param[in]  rx
*  \param[in]  ry
*  \param[in]  rz
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const T &x, const T &y, const T &z, const T &rx, const T &ry, const T &rz) {
    m_dim = 6;
    m_data = std::unique_ptr<T[]>(new T[m_dim]);
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
    m_data[3] = rx;
    m_data[4] = ry;
    m_data[5] = rz;
}

/*!
*  \brief      Constructor of the class Vec (6D)
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[in]  data array
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const unsigned int &dim, const T data[]) {
    m_dim = m_dim;
    for (unsigned int i = 0; i < m_dim; ++i)
        (*this)[i] = data[i];
}

/*!
*  \brief      Copy operator
*  \author     Sascha Kaden
*  \param[in]  Vec
*  \date       2016-05-24
*/
template<typename T>
Vec<T>::Vec(const Vec<T> &vec) {
    *this = vec;
}

/*!
*  \brief      Set all elements to a given value
*  \author     Sascha Kaden
*  \param[in]  value
*  \date       2016-05-24
*/
template<typename T>
void Vec<T>::setAllTo(const T &value) {
    for (unsigned int i = 0; i < m_dim; ++i)
        (*this)[i] = value;
}

/*!
*  \brief      Return dimension of Vec
*  \author     Sascha Kaden
*  \param[out] dimension
*  \date       2016-05-24
*/
template<typename T>
unsigned int Vec<T>::getDim() const {
    return m_dim;
}

/*!
*  \brief      Return true, if Vec is empty
*  \author     Sascha Kaden
*  \param[out] state
*  \date       2016-05-24
*/
template<typename T>
bool Vec<T>::empty() const{
    if (m_dim == 0)
        return true;
    for (unsigned int i = 0; i < m_dim; ++i)
        if ((*this)[i] == NAN)
            return true;
    return false;
}

/*!
*  \brief      Return norm 2 of Vec
*  \author     Sascha Kaden
*  \param[out] norm 2
*  \date       2016-05-24
*/
template<typename T>
T Vec<T>::norm() const {
    T norm = 0;
    for (unsigned int i = 0; i < m_dim; ++i)
        norm += (*this)[i] * (*this)[i];
    return sqrtf(norm);
}

/*!
*  \brief      Return absolute Vec of itself
*  \author     Sascha Kaden
*  \param[out] absolute Vec
*  \date       2016-05-24
*/
template<typename T>
Vec<T> Vec<T>::abs() const {
    Vec<T> result;
    for (unsigned int i = 0; i < m_dim; ++i)
        result[i] = fabs((*this)[i]);
    return result;
}

/*!
*  \brief      Return distance to given Vec
*  \author     Sascha Kaden
*  \param[in]  Vec
*  \param[out] distance
*  \date       2016-05-24
*/
template<typename T>
T Vec<T>::getDist(const Vec<T> &vec) const{
    assert(m_dim == vec.getDim());
    return sqrtf(getSqDist(vec));
}

/*!
*  \brief      Return squared distance to given Vec
*  \author     Sascha Kaden
*  \param[in]  Vec
*  \param[out] squared distance
*  \date       2016-05-24
*/
template<typename T>
T Vec<T>::getSqDist(const Vec<T> &vec) const{
    assert(m_dim == vec.getDim());
    T dist = 0;
    for (unsigned int i = 0; i < m_dim; ++i)
        dist += ((*this)[i] - vec[i]) * ((*this)[i] - vec[i]);
    return dist;
}

/*!
*  \brief      Print values of Vec to the console
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template<typename T>
void Vec<T>::print() {
    std::cout << "Dim: " << m_dim << " | ";
    for (unsigned int i = 0; i < m_dim; ++i)
        std::cout << "Value" << i << ": " << (*this)[i] << "  ";
    std::cout << std::endl;
}

#endif /* VEC_H_ */
