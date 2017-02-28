#include "Trackball.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include <Eigen/Geometry>

using std::ifstream;
using std::ofstream;
using std::stringstream;


/**
 * \brief Constructor
 *
 * \param t - initial theta
 * \param p - initial phi
 * \param d - initail distance
 */
Trackball::Trackball(float t, float p, float d)
{
  pos.theta = t;
  pos.phi = p;
  pos.dist = d;
  pos.oldX = 0;
  pos.oldY = 0;
  pos.zoomSpeed = 0.01f;
  mMotionState = ROTATE;

  pos.pos[0] = 0.0f;
  pos.pos[1] = 0.0f;
  pos.pos[2] = -1.0f;

  pos.lookAt[0] = 0.0f;
  pos.lookAt[1] = 0.0f;
  pos.lookAt[2] = 0.0f;

  pos.up[0] = 0.0f;
  pos.up[1] = 1.0f;
  pos.up[2] = 0.0f;
}


/**
 * \brief Tracks the motion from the last position and calcs changes
 *
 * \param x - mouse x position in window coordinates
 * \param y - mouse y position in window coordinates
 */
void Trackball::mouseMotion(int x, int y)
{
  float deltaX = x - pos.oldX;
  float deltaY = y - pos.oldY;

  if (mMotionState == ROTATE) {
    pos.theta -= 0.01 * deltaY;

    if (pos.theta < 0.01) pos.theta = 0.01;
    else if (pos.theta > M_PI - 0.01) pos.theta = M_PI - 0.01;

    pos.phi += 0.01 * deltaX;
    if (pos.phi < 0) pos.phi += 2*M_PI;
    else if (pos.phi > 2*M_PI) pos.phi -= 2*M_PI;
  }
  else if (mMotionState == MOVE) {
    pos.dist += pos.zoomSpeed * deltaY;
  }
  else if(mMotionState == STRAFE){
    strafe(-0.01*deltaX, -0.01*deltaY);
  }

  pos.oldX = x;
  pos.oldY = y;

  polar2xyz(); // update data
}


/**
 * \brief activated function when a mouse button is pressed or released
 *
 * \param button - button pressed 0=left 1=right 2=middle
 * \param state - button pressed (1) or released (0)
 * \param x - mouse x position in window coordinates
 * \param y - mouse y position in window coordinates
 */
void Trackball::mouse(int button, int state, int x, int y)
{
  pos.oldX = x;
  pos.oldY = y;

  if (button == 0) {
    if (state == 1) {
      mMotionState = ROTATE;
    }
  }
  else if (button == 1) {
    if (state == 1) {
      mMotionState = MOVE;
    }
  }
  else if (button == 2) {
    if (state == 1) {
      mMotionState = STRAFE;
    }
  }
}


/**
 * \brief Returns the currently saved position
 *
 * \param xyz - Reference to the xyz position
 */
Vector3f Trackball::getPosition() const
{
  return pos.pos + pos.lookAt;
}

/**
 * \brief Returns the currently saved parameters als polar coordinates, only valid if lookAt is (0,0,0)
 */
void Trackball::getPolar(float& dist, float& theta, float& phi) const
{
  dist  = pos.dist;
  theta = pos.theta;
  phi   = pos.phi;
}


/**
 * \brief Calculates the world coordinates the position of the camera from its polar coordinates, later also the lookAt an up vector should be added
 *
 * \param pos - Handle to the position of the camera in world coordinates
 */
void Trackball::polar2xyz()
{
  pos.pos[0] = pos.dist * sin(pos.theta) * cos(pos.phi);
  pos.pos[1] = pos.dist * cos(pos.theta);
  pos.pos[2] = pos.dist * sin(pos.theta) * sin(pos.phi);

  pos.pos = pos.pos + pos.lookAt; // so the camera rotates around the lookAt point
}


/**
 * \brief Change the distance
 */
void Trackball::setDistance(float distance)
{
  pos.dist = distance;
}


/**
 * \brief Change the zoom speed
 */
void Trackball::setZoomSpeed(float speed)
{
  pos.zoomSpeed = speed;
}


/**
 * \brief Use the current trackball parameters for gluLookAt
 */
void Trackball::use() const
{
 // printScreen();
  gluLookAt(pos.pos[0],pos.pos[1],pos.pos[2], pos.lookAt[0], pos.lookAt[1], pos.lookAt[2], pos.up[0], pos.up[1], pos.up[2]);
}


/**
 * \brief Zoom in or out
 */
void Trackball::zoom(const float z)
{
  pos.dist += z; //2*z;
  //if(pos.dist < 0.0f){ pos.dist = 0.0f; }

  polar2xyz();
}


/**
 * \brief Zoom in or out relative to current distance from lookAt point
 *
 * \param z - Scale factor for the distance
 */
void Trackball::scaledZoom(const float z)
{
  pos.dist *= z;
  if(pos.dist < 0.0f){ pos.dist = 0.0f; }

  polar2xyz();
}


/**
 * \brief Rotate left or right
 */
void Trackball::rotateLeftRight(const float r)
{
  pos.phi += r;
  if (pos.phi < 0) pos.phi += 2*M_PI;
  else if (pos.phi > 2*M_PI) pos.phi -= 2*M_PI;

  polar2xyz();
}


/**
 * \brief Rotate left or right
 */
void Trackball::rotateUpDown(const float r)
{
  pos.theta += r;
  if (pos.theta < 0.01) pos.theta = 0.01;
  else if (pos.theta > M_PI - 0.01) pos.theta = M_PI - 0.01;

  polar2xyz();
}


/**
 * \brief Strafes to the left, right, top or down
 */
void Trackball::strafe(const float right, const float up)
{
    Vector3f diff(pos.lookAt - pos.pos);
    Vector3f r(diff.cross(pos.up));
    r.normalize();
    diff = -(pos.lookAt - pos.pos);

    Vector3f u(diff.cross(r));
    u.normalize();
    pos.lookAt += (r * right);
    pos.lookAt += (u * up);

    polar2xyz();
}


/**
 * \brief save current state
 */
void Trackball::save()
{
  mOldPos.dist = pos.dist;
  mOldPos.theta = pos.theta;
  mOldPos.phi = pos.phi;
  mOldPos.oldX = pos.oldX;
  mOldPos.oldY = pos.oldY;
  mOldPos.pos = pos.pos;
  mOldPos.lookAt = pos.lookAt;
  mOldPos.up = pos.up;
  mOldPos.zoomSpeed = pos.zoomSpeed;
}


/**
 * \brief load old state
 */
void Trackball::load()
{
  pos.dist = mOldPos.dist;
  pos.theta = mOldPos.theta;
  pos.phi = mOldPos.phi;
  pos.oldX = mOldPos.oldX;
  pos.oldY = mOldPos.oldY;
  pos.pos = mOldPos.pos;
  pos.lookAt = mOldPos.lookAt;
  pos.up = mOldPos.up;
  pos.zoomSpeed = mOldPos.zoomSpeed;
}


/**
 * \brief Saves the current state to file
 */
void Trackball::saveToFile(const char* filename) const
{
  ofstream file(filename);

  if(file.fail()){ throw "Couldn't open file for saving current trackball."; }

  file << pos.dist << " "
       << pos.theta << " "
       << pos.phi << " "
       << pos.oldX << " "
       << pos.oldY << " "
       << pos.pos[0] << " " << pos.pos[1] << " " << pos.pos[2] << " "
       << pos.lookAt[0] << " " << pos.lookAt[1] << " " << pos.lookAt[2] << " "
       << pos.up[0] << " " << pos.up[1] << " " << pos.up[2] << " "
       << pos.zoomSpeed;
}
void Trackball::printScreen() const{

    std::cout << " Dis" << pos.dist << "Theta "
               << pos.theta << "Phi "
               << pos.phi << "OldX "
               << pos.oldX << "OldY "
               << pos.oldY << "Pos[ "
               << pos.pos[0] << ", " << pos.pos[1] << ", " << pos.pos[2] << "] LA[ "
               << pos.lookAt[0] << "," << pos.lookAt[1] << ", " << pos.lookAt[2] << "] Up[ "
               << pos.up[0] << "," << pos.up[1] << ", " << pos.up[2] << "] "
               << pos.zoomSpeed << std::endl;
}


/**
 * \brief Load a state from file
 */
void Trackball::loadFromFile(const char* filename)
{
  ifstream file(filename);

  if(file.fail()){ throw "Couldn't open file for loading trackball."; }

  file >> pos.dist
       >> pos.theta
       >> pos.phi
       >> pos.oldX
       >> pos.oldY
       >> pos.pos[0] >> pos.pos[1] >> pos.pos[2]
       >> pos.lookAt[0] >> pos.lookAt[1] >> pos.lookAt[2]
       >> pos.up[0] >> pos.up[1] >> pos.up[2]
       >> pos.zoomSpeed;
}


/**
 * \brief Loads a goal target from file
 */
void Trackball::loadGoal(const char* filename)
{
  // load goal from file //
  ifstream file(filename);

  if(file.fail()){ throw "Couldn't open file for loading trackball."; }

  file >> mGoal.dist
       >> mGoal.theta
       >> mGoal.phi
       >> mGoal.oldX
       >> mGoal.oldY
       >> mGoal.pos[0] >> mGoal.pos[1] >> mGoal.pos[2]
       >> mGoal.lookAt[0] >> mGoal.lookAt[1] >> mGoal.lookAt[2]
       >> mGoal.up[0] >> mGoal.up[1] >> mGoal.up[2]
       >> mGoal.zoomSpeed;

  // save old position //
  mOldPos = pos;
}


/**
 * \brief increments the current values towards the goal
 */
void Trackball::interpolateToGoal(float t)
{
  float tm = 1.0f - t;

  pos.dist     = mOldPos.dist * tm    + mGoal.dist * t;
  pos.theta    = mOldPos.theta * tm   + mGoal.theta * t;
  pos.phi      = mOldPos.phi * tm     + mGoal.phi * t;
  pos.oldX     = mOldPos.oldX * tm    + mGoal.oldX * t;
  pos.oldY     = mOldPos.oldY * tm     + mGoal.oldY * t;
  pos.pos      = mOldPos.pos * tm     + mGoal.pos * t ;
  pos.lookAt   = mOldPos.lookAt * tm   + mGoal.lookAt * t;
  pos.up       = mOldPos.up *tm       + mGoal.up *t ;
  pos.zoomSpeed= mOldPos.zoomSpeed * tm + mGoal.zoomSpeed *t;

}
