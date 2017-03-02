//
// Created by robert on 15.02.17.
//

#include <include/core/math/Frame.hpp>

namespace rmpl {
namespace math {

/// Constructor.
/// \param a Diagonal elements (except P().W()) are initiated with 'a'.
Frame::Frame(const T &a = 1) {
    N().X() = a;
    O().Y() = a;
    A().Z() = a;
    P().W() = 1;
}

/// Constructor.
/// \param n Normal vector.
/// \param o Open vector.
/// \param a Approach vector.
/// \param p Position vector.
Frame(const Vec<4, T> &n, const Vec<4, T> &o, const Vec<4, T> &a, const Vec<4, T> &p) {
    N() = n;
    O() = o;
    A() = a;
    P() = p;
}

/// Constructor. Missing elements of input vectors are added.
/// \param n Normal vector.
/// \param o Open vector.
/// \param a Approach vector.
/// \param p Position vector.
Frame(const Vec<3, T> &n_, const Vec<3, T> &o_, const Vec<3, T> &a_, const Vec<3, T> &p_) {
    N().X() = n_.X();
    N().Y() = n_.Y();
    N().Z() = n_.Z();
    N().W() = 0;
    O().X() = o_.X();
    O().Y() = o_.Y();
    O().Z() = o_.Z();
    O().W() = 0;
    A().X() = a_.X();
    A().Y() = a_.Y();
    A().Z() = a_.Z();
    A().W() = 0;
    P().X() = p_.X();
    P().Y() = p_.Y();
    P().Z() = p_.Z();
    P().W() = 1;
}

/// Copy constructor.
/// \param f Source frame.
Frame(const Frame &f) {
    memcpy(GetPointer(), f.GetConstPointer(), sizeof(T) * 16);
}

/// Direct access to columns.
Vec<4, T> &N() {
    return m_data[0];
}
Vec<4, T> &O() {
    return m_data[1];
}
Vec<4, T> &A() {
    return m_data[2];
}
Vec<4, T> &P() {
    return m_data[3];
}
const Vec<4, T> &N() const {
    return m_data[0];
}
const Vec<4, T> &O() const {
    return m_data[1];
}
const Vec<4, T> &A() const {
    return m_data[2];
}
const Vec<4, T> &P() const {
    return m_data[3];
}

/// Returns inverted frame.
/// \return An inverted frame.
Frame GetInv() const {
    Frame f(*this);
    f.Inv();
    return f;
}

/// Returns a pointer to the data.
/// Data is stored in the order n,o,a,p.
/// \return Data pointer.
T *GetPointer() {
    return m_data[0].GetPointer();
}

/// Returns a pointer to the data.
/// Data is stored in the order n,o,a,p.
/// \return Data pointer.
const T *GetConstPointer() const {
    return m_data[0].GetConstPointer();
}

/// Is matrix ortho-normalized?
/// \param eps Epsilon for check of orthonomality (standard 1E-8)
/// \return True, if yes.
bool IsOrthoNorm(T eps = EPSILON) const {
    if (fabs(!N() - 1) > eps)
        return false;
    if (fabs(!O() - 1) > eps)
        return false;
    if (fabs(!A() - 1) > eps)
        return false;
    if (fabs(N() * O()) > eps)
        return false;
    if (fabs(N() * A()) > eps)
        return false;
    if (fabs(O() * A()) > eps)
        return false;

    return true;
}

bool IsEqual(const Frame &f, T eps = EPSILON) const {
    if ((P().X() > (f.P().X() + eps)) || (P().X() < (f.P().X() - eps)))
        return false;
    if ((P().Y() > (f.P().Y() + eps)) || (P().Y() < (f.P().Y() - eps)))
        return false;
    if ((P().Z() > (f.P().Z() + eps)) || (P().Z() < (f.P().Z() - eps)))
        return false;
    if ((N().X() > (f.N().X() + eps)) || (N().X() < (f.N().X() - eps)))
        return false;
    if ((N().Y() > (f.N().Y() + eps)) || (N().Y() < (f.N().Y() - eps)))
        return false;
    if ((N().Z() > (f.N().Z() + eps)) || (N().Z() < (f.N().Z() - eps)))
        return false;
    if ((O().X() > (f.O().X() + eps)) || (O().X() < (f.O().X() - eps)))
        return false;
    if ((O().Y() > (f.O().Y() + eps)) || (O().Y() < (f.O().Y() - eps)))
        return false;
    if ((O().Z() > (f.O().Z() + eps)) || (O().Z() < (f.O().Z() - eps)))
        return false;
    if ((A().X() > (f.A().X() + eps)) || (A().X() < (f.A().X() - eps)))
        return false;
    if ((A().Y() > (f.A().Y() + eps)) || (A().Y() < (f.A().Y() - eps)))
        return false;
    if ((A().Z() > (f.A().Z() + eps)) || (A().Z() < (f.A().Z() - eps)))
        return false;
    return true;
}

/// Inverts the frame inplace.
/// \return Reference to the frame.
Frame &Inv()    // caution! only for orthonormed frames
{
    assert(this->IsOrthoNorm());

    Vec<4, T> old_pos(this->P());

    P().X() = -old_pos * N();
    P().Y() = -old_pos * O();
    P().Z() = -old_pos * A();

    T old_ny = N().Y(), old_nz = N().Z(), old_oz = O().Z();

    // nx
    N().Y() = O().X();
    N().Z() = A().X();

    O().X() = old_ny;
    O().Z() = A().Y();

    A().X() = old_nz;
    A().Y() = old_oz;

    return *this;
}

Frame &setQuat(double tx, double ty, double tz, double x, double y, double z, double w) {
    N().X() = w * w + x * x - y * y - z * z;
    O().X() = 2. * (x * y - w * z);
    A().X() = 2. * (x * z + w * y);
    N().Y() = 2. * (y * x + w * z);
    O().Y() = w * w - x * x + y * y - z * z;
    A().Y() = 2. * (y * z - w * x);
    N().Z() = 2. * (z * x - w * y);
    O().Z() = 2. * (z * y + w * x);
    A().Z() = w * w - x * x - y * y + z * z;
    P().X() = tx;
    P().Y() = ty;
    P().Z() = tz;
    P().W() = 1;
    return *this;
}

template <class T2>
Vec<3, T2> MultP(const Vec<3, T2> &v) const {
    Vec<4, T> vRes((T)v.X(), (T)v.Y(), (T)v.Z(), 1.0);
    vRes = *this * vRes;
    return Vec<3, T2>((T2)vRes.X(), (T2)vRes.Y(), (T2)vRes.Z());
}

template <class T2>
Vec<3, T2> MultV(const Vec<3, T2> &v) const {
    Vec<4, T> vRes((T)v.X(), (T)v.Y(), (T)v.Z(), 0.0);
    vRes = *this * vRes;
    return Vec<3, T2>((T2)vRes.X(), (T2)vRes.Y(), (T2)vRes.Z());
}

Vec<4, T> operator*(const Vec<4, T> &v) const {
    Vec<4, T> result;
    result.X() = N().X() * v.X() + O().X() * v.Y() + A().X() * v.Z() + P().X() * v.W();
    result.Y() = N().Y() * v.X() + O().Y() * v.Y() + A().Y() * v.Z() + P().Y() * v.W();
    result.Z() = N().Z() * v.X() + O().Z() * v.Y() + A().Z() * v.Z() + P().Z() * v.W();
    result.W() = N().W() * v.X() + O().W() * v.Y() + A().W() * v.Z() + P().W() * v.W();
    return result;
}

template <class T2>
Frame operator*(const Frame<T2> &f) const {
    int col, row, k;

    Frame<T> result(0.0);

    // rotation
    for (col = 0; col <= 2; ++col)
        for (row = 0; row <= 2; ++row)
            for (k = 0; k <= 3; k++)
                result[col][row] += (*this)[k][row] * f[col][k];

    // translation
    for (row = 0; row <= 2; ++row)
        for (k = 0; k <= 3; k++)
            result[3][row] += (*this)[k][row] * f[3][k];

    return result;
}

template <class T2>
Frame &operator*=(const Frame<T2> &f) {
    int col, row, k;

    Frame<T> result(0.0);

    // rotation
    for (col = 0; col <= 2; ++col)
        for (row = 0; row <= 2; ++row)
            for (k = 0; k <= 3; k++)
                result[col][row] += (*this)[k][row] * f[col][k];

    // translation
    for (row = 0; row <= 2; ++row)
        for (k = 0; k <= 3; k++)
            result[3][row] += (*this)[k][row] * f[3][k];

    (*this) = result;
    return *this;
}

Vec<4, T> &operator[](int i) {
    assert(i >= 0 && i <= 3);

    return m_data[i];
}

const Vec<4, T> &operator[](int i) const {
    assert(i >= 0 && i <= 3);

    return m_data[i];
}

/// Sets the frame from a pose vector.
/// \param pose Pose-vector containing three translation coordinates and three rotation angles in rad.
Frame &SetPoseRPY(const Vec<6, T> &pose) {
    Frame<T> res = FTrans(pose[0], pose[1], pose[2]) * FRotX(pose[3]) * FRotY(pose[4]) * FRotZ(pose[5]);

    N() = res.N();
    O() = res.O();
    A() = res.A();
    P() = res.P();

    /*
     *  Z Y X (gamma,beta,alpha)
     *
        const T Sgamma = (sin(pose[3]));
            const T Cgamma = (cos(pose[3]));
            const T Sbeta = (sin(pose[4]));
            const T Cbeta = (cos(pose[4]));
            const T Salpha = (sin(pose[5]));
            const T Calpha = (cos(pose[5]));

            N().X() = Cbeta*Cgamma;
            N().Y() = Sapha *Sbeta *Cgamma+Calpha*Sgamma;
            N().Z() = -Calpha*Sbeta*Cgamma+Salpha*Sgamma;

            O().X() = -Cbeta*Sgamma;
            O().Y() = -Salpha*Sbeta*Sgamma+Calpha*Cgamma;
            O().Z() = Calpha*Sbeta*Sgamma+Salpha*Cgamma;

            A().X() = Sbeta;
            A().Y() = -Salpha*Cbeta;
            A().Z() = Calpha*Cbeta;

            P().X() = pose[0];
            P().Y() = pose[1];
            P().Z() = pose[2];
*/
    return *this;
}

/// Sets the frame from a pose vector. (x,y,z,rz,ry',rx'')
/// \param pose Pose-vector containing three translation coordinates and three rotation angles in rad.
Frame &SetPoseYPR(const Vec<6, T> &pose) {
    Frame<T> res = FTrans(pose[0], pose[1], pose[2]) * FRotZ(pose[3]) * FRotY(pose[4]) * FRotX(pose[5]);

    N() = res.N();
    O() = res.O();
    A() = res.A();
    P() = res.P();

    return *this;
}
/// Sets the frame from a pose vector - ZY'Z'' convention
/// \param pose Pose-vector containing three translation coordinates and three rotation angles in rad.
Frame &SetPoseZYZ(const Vec<6, T> &pose) {
    Frame<T> res = FTrans(pose[0], pose[1], pose[2]) * FRotZ(pose[3]) * FRotY(pose[4]) * FRotZ(pose[5]);

    N() = res.N();
    O() = res.O();
    A() = res.A();
    P() = res.P();
    return *this;
}

/// Set the frame to the identity frame
Frame &SetIdentity() {
    N() = Vec<4, T>(1, 0, 0, 0);
    O() = Vec<4, T>(0, 1, 0, 0);
    A() = Vec<4, T>(0, 0, 1, 0);
    P() = Vec<4, T>(0, 0, 0, 1);

    return *this;
}
/// Sets the frame as a rotation frame.
/// \param angle Angle of x-rotation in rad.
Frame &SetRotX(const T &angle) {
    N() = Vec<4, T>(1, 0, 0, 0);
    O() = Vec<4, T>(0, cos(angle), sin(angle), 0);
    A() = Vec<4, T>(0, -sin(angle), cos(angle), 0);
    P() = Vec<4, T>(0, 0, 0, 1);

    return *this;
}

/// Sets the frame as a rotation frame.
/// \param angle Angle of y-rotation in rad.
Frame &SetRotY(const T &angle) {
    N() = Vec<4, T>(cos(angle), 0, -sin(angle), 0);
    O() = Vec<4, T>(0, 1, 0, 0);
    A() = Vec<4, T>(sin(angle), 0, cos(angle), 0);
    P() = Vec<4, T>(0, 0, 0, 1);
    return *this;
}

/// Sets the frame as a rotation frame.
/// \param angle Angle of z-rotation in rad.
Frame &SetRotZ(const T &angle) {
    N() = Vec<4, T>(cos(angle), sin(angle), 0, 0);
    O() = Vec<4, T>(-sin(angle), cos(angle), 0, 0);
    A() = Vec<4, T>(0, 0, 1, 0);
    P() = Vec<4, T>(0, 0, 0, 1);
    return *this;
}

/// Sets the frame as a rotation frame (XY'Z'').
/// \param phi Roll angle in rad.
/// \param theta Pitch angle in rad.
/// \param psi Yaw angle in rad.
Frame &SetRotRPY(const T &rx, const T &ry, const T &rz) {
    Frame<T> res = FRotX(rx) * FRotY(ry) * FRotZ(rz);
    N() = res.N();
    O() = res.O();
    A() = res.A();
    P() = res.P();
    return *this;
}

/// Sets the frame as a rotation frame (ZY'X'').
/// \param rz Yaw angle in rad.
/// \param ry Pitch angle in rad.
/// \param rx Roll angle in rad.
Frame &SetRotYPR(const T &rz, const T &ry, const T &rx) {
    Frame<T> res = FRotZ(rz) * FRotY(ry) * FRotX(rx);
    N() = res.N();
    O() = res.O();
    A() = res.A();
    P() = res.P();
    return *this;
}
/// Sets the frame as a rotation frame. (Z,Y',Z'')
/// \param phi Yaw angle in rad.
/// \param theta Pitch angle in rad.
/// \param psi Roll angle in rad.
Frame &SetRotZYZ(const T &z1, const T &y, const T &z2) {
    Frame<T> res = FRotZ(z1) * FRotY(y) * FRotZ(z2);
    N() = res.N();
    O() = res.O();
    A() = res.A();
    P() = res.P();
    return *this;
}

/// Sets the frame as a rotation frame.
/// See Fu, Gonzalez, Lee, ROBOTICS, p. 21
/// \param v Rotation axis.
/// \param angle Angle in rad.
Frame &SetRotVector(const Vec<4, T> &v, const T &angle) {
    if (fabs(angle) <= EPSILON)
        return (*this) = Frame();
    Vec<4, T> vn(v);
    vn.W() = 0;
    T vnLength = vn.Length();
    if (vnLength <= EPSILON)
        return (*this) = Frame();

    vn /= vnLength;

    const T Sa = sin(angle);
    const T Ca = cos(angle);
    const T Va = (T)1 - Ca;

    N().X() = vn[0] * vn[0] * Va + Ca;
    N().Y() = vn[0] * vn[1] * Va + vn[2] * Sa;
    N().Z() = vn[0] * vn[2] * Va - vn[1] * Sa;

    O().X() = vn[0] * vn[1] * Va - vn[2] * Sa;
    O().Y() = vn[1] * vn[1] * Va + Ca;
    O().Z() = vn[1] * vn[2] * Va + vn[0] * Sa;

    A().X() = vn[0] * vn[2] * Va + vn[1] * Sa;
    A().Y() = vn[1] * vn[2] * Va - vn[0] * Sa;
    A().Z() = vn[2] * vn[2] * Va + Ca;

    P().X() = 0.0;
    P().Y() = 0.0;
    P().Z() = 0.0;

    return *this;
}
/// Sets the frame for a scalation
/// \param sx Scale in x-direction
/// \param sx Scale in y-direction
/// \param sx Scale in y-direction
Frame &SetScale(const T &sx, const T &sy, const T &sz) {
    N() = Vec<4, T>(sx, 0, 0, 0);
    O() = Vec<4, T>(0, sy, 0, 0);
    A() = Vec<4, T>(0, 0, sz, 0);
    P() = Vec<4, T>(0, 0, 0, 1);

    return *this;
}
/// Sets the frame for projection
/// \param d projection
Frame &SetProjection(const T d) {
    N() = Vec<4, T>(1, 0, 0, 0);
    O() = Vec<4, T>(0, 1, 0, 0);
    A() = Vec<4, T>(0, 0, 1, 1 / d);
    P() = Vec<4, T>(0, 0, 0, 0);
    return *this;
}

/// Sets the frame as a translation frame.
/// \param dx Translation in x-direction.
/// \param dy Translation in y-direction.
/// \param dz Translation in z-direction.
Frame &SetTrans(const T &dx, const T &dy, const T &dz) {
    N() = Vec<4, T>(1, 0, 0, 0);
    O() = Vec<4, T>(0, 1, 0, 0);
    A() = Vec<4, T>(0, 0, 1, 0);
    P() = Vec<4, T>(dx, dy, dz, 1);
    return *this;
}

/// Sets the frame as a translation frame.
/// \param t Translation vector.
Frame &SetTrans(const Vec<4, T> &t) {
    N() = Vec<4, T>(1, 0, 0, 0);
    O() = Vec<4, T>(0, 1, 0, 0);
    A() = Vec<4, T>(0, 0, 1, 0);
    P() = t;
    P().W() = 1;
    return *this;
}

void GetRotVector(Vec<4, T> &v, T &angle) const {
    // calculate rotation HVECTOR and angle
    // see J.Opt.Soc.Am. A/Vol.4,No.4/April 87; B.K.P.Horn, page 641ff
    // we use internal a quaternion representation

    // because the accuracy of arccos is critical near 0, we
    // do some special calculations to achieve a higher numerical
    // accuracy

    Frame f = *this;    // orthonormed frame

    f.OrthoNormalize();

    T FourQ0Q0, Q0 = 0;    // Quaternion components
    T FourQxQx, Qx = 0;
    T FourQyQy, Qy = 0;
    T FourQzQz, Qz = 0;

    FourQ0Q0 = 1 + f.N().X() + f.O().Y() + f.A().Z();
    FourQxQx = 1 + f.N().X() - f.O().Y() - f.A().Z();
    FourQyQy = 1 - f.N().X() + f.O().Y() - f.A().Z();
    FourQzQz = 1 - f.N().X() - f.O().Y() + f.A().Z();

    if (FourQ0Q0 >= FourQxQx && FourQ0Q0 >= FourQyQy && FourQ0Q0 >= FourQzQz) {
        // FourQ0Q0 is largest value
        Q0 = sqrt(FourQ0Q0) / (T)2;
        Qx = (f.O().Z() - f.A().Y()) / ((T)4 * Q0);
        Qy = (f.A().X() - f.N().Z()) / ((T)4 * Q0);
        Qz = (f.N().Y() - f.O().X()) / ((T)4 * Q0);
    } else if (FourQxQx >= FourQ0Q0 && FourQxQx >= FourQyQy && FourQxQx >= FourQzQz) {
        // FourQxQx is largest value
        Qx = sqrt(FourQxQx) / (T)2;
        Q0 = (f.O().Z() - f.A().Y()) / ((T)4 * Qx);
        Qy = (f.N().Y() + f.O().X()) / ((T)4 * Qx);
        Qz = (f.A().X() + f.N().Z()) / ((T)4 * Qx);
    } else if (FourQyQy >= FourQ0Q0 && FourQyQy >= FourQxQx && FourQyQy >= FourQzQz) {
        // FourQyQy is largest value
        Qy = sqrt(FourQyQy) / (T)2;
        Q0 = (f.A().X() - f.N().Z()) / ((T)4 * Qy);
        Qx = (f.N().Y() + f.O().X()) / ((T)4 * Qy);
        Qz = (f.O().Z() + f.A().Y()) / ((T)4 * Qy);
    } else if (FourQzQz >= FourQ0Q0 && FourQzQz >= FourQxQx && FourQzQz >= FourQyQy) {
        // FourQzQz is largest value
        Qz = sqrt(FourQzQz) / (T)2;
        Q0 = (f.N().Y() - f.O().X()) / ((T)4 * Qz);
        Qx = (f.A().X() + f.N().Z()) / ((T)4 * Qz);
        Qy = (f.O().Z() + f.A().Y()) / ((T)4 * Qz);
    } else {
        // Mit dieser Zeile kommt der Linker nicht zurecht -
        // SCHEISS WINDOWS NT + VISUAL C++ !
        std::cerr << "FRAME :: GetRotHVECTOR() internal error\n";
        // PRO0 ("FRAME::GetRotHVECTOR() internal error\n")
        assert(0);
    }

    // calculate angle and rotation HVECTOR
    Vec<4, T> rv(0, 0, 0, 0);

    angle = (T)2 * acos(Q0);
    if (fabs(angle) > EPSILON && !(Qx == 0 && Qy == 0 && Qz == 0)) {
        rv.X() = Qx;
        rv.Y() = Qy;
        rv.Z() = Qz;
        assert(fabs(sin(acos(Q0))) > EPSILON);
        v = rv * ((T)1 / sin(acos(Q0)));
    } else {
        angle = 0;
        v = rv;
    }
}

/// Returns the RPY-values as a vector. (XY'Z'' convention)
/// See #CalcRotRPY.
/// \return Vector containing the RPY-values.
Vec<3, T> GetRPY() const {
    throw std::logic_error("GetRPY() is not implemented yet.");
    T rx, ry, rz;
    CalcRotRPY(rx, ry, rz);

    return Vec<3, T>(rx, ry, rz);
}

/// Returns the translational components together with the RPY-values as a vector. (XY'Z'' convention)
/// See #CalcRotRPY.
/// \return Vector containing the RPY-values.
Vec<6, T> GetPoseRPY() const {
    throw std::logic_error("GetRPY() is not implemented yet.");
    T rx, ry, rz;
    CalcRotRPY(rx, ry, rz);

    return Vec<6, T>(P().X(), P().Y(), P().Z(), rx, ry, rz);
}

/// Returns the YPR-values as a vector. (ZY'X'' convention)
/// See #CalcRotYPR.
/// \return Vector containing the YPR-values.
Vec<3, T> GetYPR() const {
    T rz, ry, rx;
    CalcRotYPR(rz, ry, rx);
    return Vec<3, T>(rz, ry, rx);
}
/// Returns the translational components together with the YPR-values as a vector.
/// See #CalcRotYPR.
/// \return Vector containing the YPR-values.
Vec<6, T> GetPoseYPR() const {
    T rz, ry, rx;
    CalcRotYPR(rz, ry, rx);

    return Vec<6, T>(P().X(), P().Y(), P().Z(), rz, ry, rx);
}

/// Returns the YPR-values as a vector. (ZY'Z'' convention)
/// See #CalcRotYPR.
/// \return Vector containing the YPR-values.
Vec<3, T> GetZYZ() const {
    T z1, y, z2;
    CalcRotZYZ(z1, y, z2);

    return Vec<3, T>(z1, y, z2);
}
/// Returns the translational components together with the YPR-values as a vector.
/// See #CalcRotYPR.
/// \return Vector containing the YPR-values.
Vec<6, T> GetPoseZYZ() const {
    T z1, y, z2;
    CalcRotZYZ(z1, y, z2);

    return Vec<6, T>(P().X(), P().Y(), P().Z(), z1, y, z2);
}

/// Makes the axis orthogonal.
/// \return *this
Frame &OrthoNormalize() {
    // Check before normalizing.
    if (IsOrthoNorm())
        return *this;

    // OrthoNormalize.
    O().Normalize();
    A().Normalize();
    N() = ~(O() ^ A());
    O() = A() ^ N();

    return *this;
}
/// Comment
Frame &getTransformation4D(Vec<3, T> &p1, Vec<3, T> &p2, Vec<3, T> &q1, Vec<3, T> &q2, bool &valid) {
    math::Vec<3, T> diff;
    math::Vec<3, T> q2_prime;
    double angle;
    math::Vec<3, T> direction1, direction2;

    diff = p1 - q1;
    q2_prime = q2 + diff;
    valid = false;
    direction1 = p2 - p1;
    direction2 = q2_prime - p1;
    if (fabs((direction1.Z() - direction2.Z())) < 0.1) {
        angle = direction1 % direction2;
        valid = true;
        this->SetRotZ(angle);
        P().X() = diff.X();
        P().Y() = diff.Y();
        P().Z() = diff.Z();
    }
    return *this;
}
/// Comment
void getSerilized(T *current) {
    current[0] = N().X();
    current[1] = N().Y();
    current[2] = N().Z();
    current[3] = 0;

    current[4] = O().X();
    current[5] = O().Y();
    current[6] = O().Z();
    current[7] = 0;

    current[8] = A().X();
    current[9] = A().Y();
    current[10] = A().Z();
    current[11] = 0;

    current[12] = P().X();
    current[13] = P().Y();
    current[14] = P().Z();
    current[15] = 1;
}
/// Special Feature Frames
Frame &getFeatureFrame(Vec<3, T> &p1, Vec<3, T> &p2, Vec<3, T> &n1, Vec<3, T> &n2) {
    /*	math::Vec<3,T>   p12,x,y,z,p;

            p12 = p2-p1;

            x = p12;
            x.Normalize();
            N().X() = x.X();
            N().Y() = x.Y();
            N().Z() = x.Z();
            N().W() = 0.0;

            y = x^n1;
            y.Normalize();
            O().X() = y.X();
            O().Y() = y.Y();
            O().Z() = y.Z();
            O().W() = 0.0;

            z = x^y;
            z = z.Normalize();

            A().X() = z.X();
            A().Y() = z.Y();
            A().Z() = z.Z();
            A().W() = 0.0;


            P().X() = p1.X();
            P().Y() = p1.Y();
            P().Z() = p1.Z();
            P().W() = 1.0;

            return *this;
 */
    math::Vec<3, T> p12, n12;
    math::Vec<3, T> x, y, z, p;

    p12 = p2 - p1;
    n12 = n1 + n2;

    x = p12 ^ n12;
    x = x.Normalize();

    y = p12;
    y = y.Normalize();

    z = p12 ^ n12 ^ p12;
    z = z.Normalize();

    p = (p1 * 0.5f) + (p2 * 0.5f);

    N().X() = x.X();
    N().Y() = x.Y();
    N().Z() = x.Z();
    N().W() = 0.0;

    O().X() = y.X();
    O().Y() = y.Y();
    O().Z() = y.Z();
    O().W() = 0.0;

    A().X() = z.X();
    A().Y() = z.Y();
    A().Z() = z.Z();
    A().W() = 0.0;

    P().X() = p.X();
    P().Y() = p.Y();
    P().Z() = p.Z();
    P().W() = 1.0;

    return *this;
}
/// Special Feature Frames
Frame &getFeatureFrameTriple(Vec<3, T> &p1, Vec<3, T> &p2, Vec<3, T> &p3) {
    math::Vec<3, T> p21, p23, x, y, z, p;

    p21 = p1 - p2;
    p23 = p3 - p2;

    x = p21 ^ p23;
    x.Normalize();
    N().X() = x.X();
    N().Y() = x.Y();
    N().Z() = x.Z();
    N().W() = 0.0;

    y = p21;
    y.Normalize();
    O().X() = y.X();
    O().Y() = y.Y();
    O().Z() = y.Z();
    O().W() = 0.0;

    z = x ^ y;
    z = z.Normalize();

    A().X() = z.X();
    A().Y() = z.Y();
    A().Z() = z.Z();
    A().W() = 0.0;

    P().X() = p2.X();
    P().Y() = p2.Y();
    P().Z() = p2.Z();
    P().W() = 1.0;

    return *this;
}
/// Comment
void setZeroRxRy() {
    Vec6d pose;

    pose = this->GetPoseYPR();
    if (pose[5] < 0) {
        pose[5] = pose[5] + math::PI;
    } else if (pose[5] > math::PI) {
        pose[5] = pose[5] - math::PI;
    }
    pose[3] = 0;
    pose[4] = 0;
    this->SetPoseYPR(pose);
}
/// Comment
void setCraig(double a, double alpha, double d, double theta) {
    this->N().X() = cos(theta);
    this->N().Y() = sin(theta) * cos(alpha);
    this->N().Z() = sin(theta) * sin(alpha);
    this->N().W() = 0;

    this->O().X() = -sin(theta);
    this->O().Y() = cos(theta) * cos(alpha);
    this->O().Z() = cos(theta) * sin(alpha);
    this->O().W() = 0;

    this->A().X() = 0;
    this->A().Y() = -sin(alpha);
    this->A().Z() = cos(alpha);
    this->A().W() = 0;

    this->P().X() = a;
    this->P().Y() = d * (-sin(alpha));
    this->P().Z() = cos(alpha) * d;
    this->P().W() = 1;
}
/// Comment
void setDH(double theta, double d, double a, double alpha) {
    double c_th = cos(theta);
    double c_al = cos(alpha);
    double s_th = sin(theta);
    double s_al = sin(alpha);

    c_th = fabs(c_th) > 0.000001 ? c_th : 0.0;
    c_al = fabs(c_al) > 0.000001 ? c_al : 0.0;
    s_th = fabs(s_th) > 0.000001 ? s_th : 0.0;
    s_al = fabs(s_al) > 0.000001 ? s_al : 0.0;

    this->N().X() = c_th;
    this->N().Y() = s_th;
    this->N().Z() = 0;
    this->N().W() = 0;

    this->O().X() = -s_th * c_al;
    this->O().Y() = c_th * c_al;
    this->O().Z() = s_al;
    this->O().W() = 0;

    this->A().X() = s_th * s_al;
    this->A().Y() = -c_th * s_al;
    this->A().Z() = c_al;
    this->A().W() = 0;

    this->P().X() = a * c_th;
    this->P().Y() = a * s_th;
    this->P().Z() = d;
    this->P().W() = 1;
}

private:
/// Calculates the roll, pitch, yaw values. (ZY'X'' Convention)
/// \param phi Roll.
/// \param theta Pitch.
/// \param psi Yaw.
void CalcRotRPY(double &rx, double &ry, double &rz) const {
    rx = 99999999;
    ry = 99999999;
    rz = 99999999;
    std::cerr << "CalcRotRPY() is not implemented yet." << std::endl;

    //                        // calculate roll, pitch, and yaw from FRAME4
    //                        // there are a lot of different formulas: this is from
    //                        // Ho, Sriwattanathamma: ROBOT KINEMATICS, p. 115
    //                        // only one(of two or more) solutions is calculated
    //                        //
    //                        // if the angle phi is undefined it is set to 0.0

    //                        // with MS C++ Standard-Lib the atan2(x,y)-function is
    //                        // well defined even if x = 0, and y <> 0. If x = 0 and
    //                        // y = 0 then atan2(x,y) is 0.0. This behaviour differs
    //                        // from library to library. PAY ATTENTION to this fact!

    //                        if (fabs(N().Y()) > Frame::EPSILON || fabs(N().X()) > Frame::EPSILON)
    //                                phi = atan2(N().Y(), N().X());
    //                        else
    //                                phi=PI/2;

    //                        // the other solution is: atan2(-N().Y(),-N().X());
    //                        // which is equivalent to *phi +/- PI
    //                        // if A().X() = 0 and A().X() = 0 then *phi
    //                        // can have an arbitrary value, we choose *phi = 0.0 (because of the
    //                        // behaviour of atan2)

    //                        if (fabs(N().Z()) > Frame::EPSILON || fabs(cos(phi)*N().X() + sin(phi)*N().Y()) > Frame::EPSILON)
    //                                        theta = atan2 ( -N().Z(),
    //                                                                        cos(phi)*N().X() + sin(phi)*N().Y() );
    //                        else
    //                                theta=PI/2;

    //                        if (fabs(sin(phi)*A().X() - cos(phi)*A().Y()) > Frame::EPSILON ||
    //                                fabs(- sin(phi)*O().X() + cos(phi)*O().Y()) > Frame::EPSILON)
    //                                        psi = atan2 (  sin(phi)*A().X() - cos(phi)*A().Y(),
    //                                                                  -sin(phi)*O().X() + cos(phi)*O().Y() );
    //                        else
    //                                psi=PI/2;
}

/// Calculates the roll, pitch, yaw values. (ZY'X'' Convention)
/// \param phi Roll.
/// \param theta Pitch.
/// \param psi Yaw.
void CalcRotYPR(double &rz, double &ry, double &rx) const {
    // calculate roll, pitch, and yaw from FRAME4
    // there are a lot of different formulas: this is from
    // Ho, Sriwattanathamma: ROBOT KINEMATICS, p. 115
    // only one(of two or more) solutions is calculated
    //
    // if the angle rz is undefined it is set to 0.0

    // with MS C++ Standard-Lib the atan2(x,y)-function is
    // well defined even if x = 0, and y <> 0. If x = 0 and
    // y = 0 then atan2(x,y) is 0.0. This behaviour differs
    // from library to library. PAY ATTENTION to this fact!

    if (fabs(N().Y()) > Frame::EPSILON || fabs(N().X()) > Frame::EPSILON)
        rz = atan2(N().Y(), N().X());
    else
        rz = PI / 2;

    // the other solution is: atan2(-N().Y(),-N().X());
    // which is equivalent to *rz +/- PI
    // if A().X() = 0 and A().X() = 0 then *rz
    // can have an arbitrary value, we choose *rz = 0.0 (because of the
    // behaviour of atan2)

    if (fabs(N().Z()) > Frame::EPSILON || fabs(cos(rz) * N().X() + sin(rz) * N().Y()) > Frame::EPSILON)
        ry = atan2(-N().Z(), cos(rz) * N().X() + sin(rz) * N().Y());
    else
        ry = PI / 2;

    if (fabs(sin(rz) * A().X() - cos(rz) * A().Y()) > Frame::EPSILON ||
        fabs(-sin(rz) * O().X() + cos(rz) * O().Y()) > Frame::EPSILON)
        rx = atan2(sin(rz) * A().X() - cos(rz) * A().Y(), -sin(rz) * O().X() + cos(rz) * O().Y());
    else
        rx = PI / 2;
}

/// Calculates the yaw, pitch, roll values. (ZY'Z'' Convention)
/// \param Yaw. umZ
/// \param Pitch. umY'
/// \param Roll. umZ''
void CalcRotZYZ(double &yaw, double &pitch, double &roll) const {
    // with MS C++ Standard-Lib the atan2(x,y)-function is
    // well defined even if x = 0, and y <> 0. If x = 0 and
    // y = 0 then atan2(x,y) is 0.0. This behaviour differs
    // from library to library. PAY ATTENTION to this fact!
    double eps = 0.001;

    pitch = atan2(sqrt(N().Z() * N().Z() + O().Z() * O().Z()), A().Z());
    if (fabs(pitch) < eps) {
        yaw = 0.0;
        roll = atan2(-O().X(), N().X());
    } else {
        if (fabs(pitch - 180.0) <= eps) {
            yaw = 0.0;
            roll = atan2(O().X(), -N().X());
        } else {
            yaw = atan2(A().Y(), A().X());
            roll = atan2(O().Z(), -N().Z());
        }
    }
}

public:
/// Loads the frame from a file.
/// \param filename Name of the file.
/// \param eps Epsilon for check of orthonomality
/// \return True, if successful.
bool LoadFromFile(const std::string &filename) {
    std::ifstream f;    //(filename, std::ios::in);

    f.open(filename.c_str(), std::ios::in);

    if (f.is_open()) {
        for (int row = 0; row < 4; row++) {
            f >> this->N()[row] >> this->O()[row] >> this->A()[row] >> this->P()[row];
        }
        f.close();
        return true;
    }
    return false;
}
/// Saves the frame to a file.
/// \param filename Name of the file.
/// \return True, if successful.
bool SaveToFile(const std::string &filename) {
    std::fstream f;

    f.open(filename.c_str(), std::ios::out);

    if (f.is_open() == true) {
        for (int row = 0; row < 4; row++) {
            f << N()[row] << " " << O()[row] << " " << A()[row] << " " << P()[row] << "\n";
        }
        f.close();
        return true;
    }
    return false;
}
//    template <class T>
static Frame<T> FTrans(const T &x, const T &y, const T &z) {
    return Frame<T>().SetTrans(x, y, z);
}

//    template <class T>
static Frame<T> FTransX(const T &x) {
    return Frame<T>().SetTrans(x, 0, 0);
}

//   template <class T>
static Frame<T> FTransY(const T &y) {
    return Frame<T>().SetTrans(0, y, 0);
}

//  template <class T>
static Frame<T> FTransZ(const T &z) {
    return Frame<T>().SetTrans(0, 0, z);
}

//  template <class T>
static Frame<T> FTrans(const Vec<4, T> &v) {
    return Frame<T>().SetTrans(v);
}

//    template <class T>
static Frame<T> FRotX(const T &angle) {
    return Frame<T>().SetRotX(angle);
}

//    template <class T>
static Frame<T> FRotY(const T &angle) {
    return Frame<T>().SetRotY(angle);
}

//   template <class T>
static Frame<T> FRotZ(const T &angle) {
    return Frame<T>().SetRotZ(angle);
}

//    template <class T>
static Frame<T> FRotRPY(const T &phi, const T &theta, const T &psi) {
    return Frame<T>().SetRotRPY(phi, theta, psi);
}

//    template <class T>
static Frame<T> FRotRPY(const Vec<6, T> &pose) {
    return Frame<T>().SetPoseRotRPY(pose);
}

//    template <class T>
static Frame<T> FRotYPR(const T &phi, const T &theta, const T &psi) {
    return Frame<T>().SetRotYPR(phi, theta, psi);
}

//    template <class T>
static Frame<T> FRotYPR(const Vec<6, T> &pose) {
    return Frame<T>().SetPoseRotYPR(pose);
}
}
}
