//
// Created by robert on 15.02.17.
//

#ifndef ROBOTMOTIONPLANNING_FRAME_HPP
#define ROBOTMOTIONPLANNING_FRAME_HPP

#include <cstddef>

#include <Eigen/Dense>

#include <include/core/types.h>

namespace rmpl { namespace math {

/*
template<typename T, std::size_t Dim>
class Vector<T, Dim> : public Eigen::Matrix<T, Dim, 1>
{
public:
    Vector();
    virtual ~Vector();
};
*/

template<typename T>
class Frame : public Eigen::Matrix<T, 4, 4>
{
public:

    static const T	EPSILON;	///< frame epsilon

public: // instance methods
    Frame(const T &a = 1);
    Frame(const Eigen::Matrix<T, 4, 1>& &n, const Eigen::Matrix<T, 4, 1>& &o, const Eigen::Matrix<T, 4, 1>& &a, const Eigen::Matrix<T, 4, 1>& &p);
    Frame(const Eigen::Matrix<T, 3, 1> &n_, const Eigen::Matrix<T, 3, 1> &o_, const Eigen::Matrix<T, 3, 1> &a_, const Eigen::Matrix<T, 3, 1> &p_);

    Eigen::Matrix<T, 4, 1>& N() { return this->row; };
    Eigen::Matrix<T, 4, 1>& O();
    Eigen::Matrix<T, 4, 1>& A();
    Eigen::Matrix<T, 4, 1>& P();
    const Eigen::Matrix<T, 4, 1>&& N() const {return m_data[0];}
    const Eigen::Matrix<T, 4, 1>&& O() const {return m_data[1];}
    const Eigen::Matrix<T, 4, 1>&& A() const {return m_data[2];}
    const Eigen::Matrix<T, 4, 1>&& P() const {return m_data[3];}

private:
    Eigen::Matrix<T, 4, 4>
};

};
};

#endif //ROBOTMOTIONPLANNING_FRAME_HPP
