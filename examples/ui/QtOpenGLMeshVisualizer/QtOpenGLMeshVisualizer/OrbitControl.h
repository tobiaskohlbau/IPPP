#ifndef ORBITCONTROL_H
#define ORBITCONTROL_H

#include <GL/glut.h>
#include <Eigen/Dense>

//#include <MathLib/mathVec.h>
//#include <MathLib/mathFrame.h>

typedef Eigen::Matrix<float, 4, 4> Frame4f;
typedef Eigen::Matrix<float, 4, 1> Vector4f;

class OrbitControl{

public:
    OrbitControl();
    OrbitControl(Frame4f &camera,double radius);
    ~OrbitControl();
    /*
     * using the orbit calls gluLookAt according to calculated camera frame with near = 1.0
     * and far = radius
     */
    void use();
    /*
     *  mouse Moved
     *  \param x position
     *  \param y position
     */
    void mouseMove(int x,int y);
    /*!
     *  Mouse pressed or released
     *  \param button 0 = left 1= right 2= mid
     *  \param press = 1 release = 0
     *  \param x position of mouse
     *  \param y position of mouse
     */
    void mousePress(int button,int press_release,int x,int y);
    /*!
     *  zoom in and out
     */

    void zoom(float value);
    void reset();
    void setRadius(float radius);
    void upsidedown();

protected:

    Frame4f m_camera;
    Eigen::Vector4f m_look_at;
    int         		  m_old_x;
    int                   m_old_y;
    float				  m_radius;
    enum motionStateEnum {FLY,ROTATE,MOVE,NOTHING,ZOOM};
    motionStateEnum		  m_motion;
};

#endif // ORBITCONTROL_H
