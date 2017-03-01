#include "OrbitControl.h"

static void SetRotX(Frame4f &frame, const float angleRad)
{

    frame.col(0) = Eigen::Vector4f(1, 0, 0, 0);
    frame.col(1) = Eigen::Vector4f(0, cos(angleRad), sin(angleRad), 0);
    frame.col(2) = Eigen::Vector4f(0, -sin(angleRad), cos(angleRad), 0);
    frame.col(3) = Eigen::Vector4f(0, 0, 0, 1);
}

/// Sets the frame as a rotation frame.
/// \param angle Angle of y-rotation in rad.
static void SetRotY(Frame4f &frame, const float angleRad)
{
    frame.col(0) = Eigen::Vector4f(cos(angleRad), 0, -sin(angleRad), 0);
    frame.col(1) = Eigen::Vector4f(0, 1, 0, 0);
    frame.col(2) = Eigen::Vector4f(sin(angleRad), 0, cos(angleRad), 0);
    frame.col(3) = Eigen::Vector4f(0, 0, 0, 1);
}

/// Sets the frame as a rotation frame.
/// \param angle Angle of z-rotation in rad.
static void SetRotZ(Frame4f &frame, const float angleRad)
{
    frame.col(0) = Eigen::Vector4f(cos(angleRad), sin(angleRad), 0, 0);
    frame.col(1) = Eigen::Vector4f(-sin(angleRad), cos(angleRad), 0, 0);
    frame.col(2) = Eigen::Vector4f(0, 0, 1, 0);
    frame.col(3) = Eigen::Vector4f(0, 0, 0, 1);
}

OrbitControl::OrbitControl()
{
    const float r = 200.0;

    this->m_radius = r;
    m_camera(0, 3) = 0;
    m_camera(1, 3) = 0;
    m_camera(2, 3) = -r;
    m_camera(3, 3) = 1.0;
    //this->m_camera.P()=Vector4f(0.0,0.0,-r,1.0);
    this->m_old_x=0.0;
    this->m_old_y=0.0;
    this->m_look_at = this->m_camera.col(3) + this->m_camera.col(2);
}

OrbitControl::OrbitControl(Frame4f &camera,double radius)
{

    this->m_radius=radius;
    this->m_camera=camera;
    this->m_old_x=0.0;
    this->m_old_y=0.0;
    m_look_at = m_camera.col(3) + m_camera.col(2);
}

OrbitControl::~OrbitControl()
{

}

void OrbitControl::use()
{

    gluLookAt(m_camera(0, 3), m_camera(1, 3), m_camera(2, 3),
               m_look_at(0), m_look_at(1), m_look_at(2),
               m_camera(0, 1), m_camera(1, 1), m_camera(2, 1));

}

void OrbitControl::mouseMove(int x,int y)
{
       float delta_x = x-m_old_x;
       float delta_y = y-m_old_y;
       Frame4f  pitch, yaw, move;

       if(m_motion == ROTATE){
           // pitch and yaw
           SetRotX(pitch, -(delta_y * M_PI)/m_radius);
           //pitch.SetRotX(-(delta_y*math::PI)/m_radius);
           SetRotY(yaw, -(delta_x * M_PI)/m_radius);
           m_camera=pitch*m_camera;
           m_camera=yaw*m_camera;
           m_look_at = m_camera.col(3) + m_camera.col(2);
           m_old_x=x;
           m_old_y=y;
       }
       if (m_motion == FLY){ // todo: not implemented (R.Andre 20150609)
           SetRotX(pitch, (delta_x * 2.0 * M_PI)/m_radius);
           //pitch.SetRotX(();
           SetRotY(yaw, (delta_y * 2 * M_PI)/m_radius);
           //yaw.SetRotY();
           m_camera = m_camera * pitch;
           m_camera=m_camera*yaw;
           m_look_at = m_camera.col(3) + m_camera.col(2);
           m_old_x=x;
           m_old_y=y;
       }
       else if(m_motion == MOVE){
           move(0, 3) += delta_x;
           move(1, 3) += delta_y;
           this->m_camera=m_camera*move;
           m_look_at = m_camera.col(3) + m_camera.col(2);
           m_old_x=x;
           m_old_y=y;
       }
       else if(m_motion == ZOOM){
           this->zoom(delta_y/100);
       }
       m_old_x=x;
       m_old_y=y;

}

void OrbitControl::mousePress(int button,int state,int x,int y)
{
    m_old_x = x;
    m_old_y = y;

    if (button == 0) {
        if (state == 1) {
          m_motion = ROTATE;
        }
      }
      else if (button == 1) {
        if (state == 1) {
          m_motion = MOVE;
        }
      }
      else if (button == 2){
         if(state == 1)
            m_motion = ZOOM;
      }
      else{
          m_motion = NOTHING;
      }

}

void OrbitControl::zoom(float value)
{
    Frame4f move;
    move(2, 3) = value * 0.1;
    m_camera = m_camera*move;
    if( m_camera(2, 3) > m_radius)
    {
        m_camera(2, 3) = m_radius;
    }
    if (m_camera(2, 3) > (m_radius * (1 - 0 ))) // lol <- arob
    {
        m_camera(2, 3) =- m_radius;
    }
    m_look_at = m_camera.col(3) + m_camera.col(2);
}

void OrbitControl::reset()
{
    Frame4f move;
    this->m_camera=move;
    this->m_camera.col(3) = Vector4f(0.0f, 0.0f, -m_radius, 1.0f);
    this->m_old_x=0.0;
    this->m_old_y=0.0;
    m_look_at = m_camera.col(3) + m_camera.col(2);
}

void OrbitControl::setRadius(float radius)
{
    this->m_radius=radius;
}

void OrbitControl::upsidedown()
{
    Frame4f rot;
    SetRotZ(rot, M_PI);
    //rot.SetRotZ(math::PI);
    m_camera = rot * m_camera;
    m_look_at = m_camera.col(3) + m_camera.col(2);
}
