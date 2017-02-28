#ifndef TRACKBALL_H
#define TRACKBALL_H

#include <cmath>
#include <string>

#ifdef __APPLE__
 #include <GLUT/glut.h>
#else
  #include <GL/glut.h>
#endif
//#include <MathLib/mathVec.h>
#include <Eigen/Dense>

typedef Eigen::Matrix< float, 3, 1 > Vector3f;

struct TrackballState
{
  float dist;
  float theta;
  float phi;
  float oldX;
  float oldY;
  Vector3f pos;
  Vector3f lookAt;
  Vector3f up;
  float zoomSpeed;
};

class Trackball
{
private:


public:
  Trackball(float t = M_PI / 2.0f, float p = M_PI/2.0f, float d = 2.5f);

  void mouseMotion(int x, int y);
  void mouse(int button, int state, int x, int y);

  Vector3f getPosition() const;
  Vector3f getLookAt() const { return pos.lookAt; }
  void getPolar(float& dist, float& theta, float& phi) const;
  float getPhi() const{ return pos.phi; }
  float getTheta() const{ return pos.theta; }
  float getDistance() const{return pos.dist; }

  void setDistance(float distance);
  void setZoomSpeed(float speed);

  void setLookAt(const Vector3f &lookAt){ pos.lookAt = lookAt; }
  void setLookAt(const Vector3f &posE, const Vector3f &lookAt) { pos.pos=posE; pos.lookAt = lookAt; }
  void setLookAt(const Vector3f &posE, const Vector3f &lookAt, const Vector3f &up){ pos.pos=posE; pos.lookAt = lookAt; pos.up=up;}
  void use() const;

  void zoom(const float z);
  void scaledZoom(const float z);
  void rotateLeftRight(const float r);
  void rotateUpDown(const float r);
  void strafe(const float right, const float up);

  void save();
  void load();

  void saveToFile(const char* filename) const;
  void loadFromFile(const char* filename);
  void printScreen() const;

  void loadGoal(const char* filename);
  void interpolateToGoal(float time);

 protected:
  void polar2xyz();
  /**
   * \brief Enumeration for depicting the current motion state
   */
  enum motionStateEnum {ROTATE, /*!< Camera is in rotation mode */
                        MOVE,   /*!< Camera is in move mode */
                        STRAFE};

  /**
   * \brief State variable depicting the current motion state
   */
  motionStateEnum mMotionState;

  /**
   * \brief Current position
   */
  TrackballState pos;

  /**
   * \brief saves the old position
   */
  TrackballState mOldPos;

  /**
   * \brief target position
   */
  TrackballState mGoal;
};

#endif // TRACKBALL_H
