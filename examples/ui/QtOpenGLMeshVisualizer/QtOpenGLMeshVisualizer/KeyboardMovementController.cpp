#include "KeyboardMovementController.h"

#include "VisualisationComponents/include/control/IMovementControllable.h"
#include "VisualisationComponents/include/control/MovementController.h"

static const viz::control::MovementParameters_t NullMovement;

static inline void initMovementParams(viz::control::MovementParameters &mp)
{
    mp.x = mp.y = mp.z = mp.phi = mp.theta = mp.psi = 0.0;
}

KeyboardMovementController::KeyboardMovementController(viz::control::IMovementControllable &controllable)
    : MovementController(controllable), _movementFactor(0.2), _rotationFactor(1), _cw(1.0)
{
    clearHistory();
}

KeyboardMovementController::~KeyboardMovementController()
{
    _history.clear();
}

void KeyboardMovementController::processKeyEvent(QKeyEvent *event)
{
    viz::control::MovementParameters mp;
    initMovementParams(mp);
    //const math::Vec3d t(0.1, 0.0, 0.0);
    switch(event->key())
    {
    case Qt::Key_Left:
    case Qt::Key_A:
        left();
        break;
    case Qt::Key_Right:
    case Qt::Key_D:
        right();
        break;
    case Qt::Key_Up:
    case Qt::Key_W:
        up();
        break;
    case Qt::Key_Down:
    case Qt::Key_S:
        down();
        break;
    case Qt::Key_9:
    case Qt::Key_PageUp:
        forward();
        break;
    case Qt::Key_3:
    case Qt::Key_PageDown:
        backward();
        break;
    default:
        return;
    }
}

void KeyboardMovementController::moveTo(const double x, const double y, const double z)
{
    viz::control::MovementParameters mp;
    initMovementParams(mp);
    mp.x = x;
    mp.y = y;
    mp.z = z;
    moveTo(mp);
}

void KeyboardMovementController::up()
{
    up(_movementFactor);
}

void KeyboardMovementController::up(const double y)
{
    moveTo(0.0, y);
}

void KeyboardMovementController::down()
{
    down(-1.0 * _movementFactor);
}

void KeyboardMovementController::down(const double y)
{
    moveTo(0.0, y);
}

void KeyboardMovementController::left()
{
    left(_movementFactor);
}

void KeyboardMovementController::left(const double x)
{
    moveTo(x);
}

void KeyboardMovementController::right()
{
    right(-1.0 * _movementFactor);
}

void KeyboardMovementController::right(const double x)
{
    moveTo(x);
}

void KeyboardMovementController::backward()
{
    backward(-1.0 * _movementFactor);
}

void KeyboardMovementController::backward(const double z)
{
    moveTo(0.0, 0.0, z);
}

void KeyboardMovementController::forward()
{
    forward(_movementFactor);
}

void KeyboardMovementController::forward(const double z)
{
    moveTo(0.0, 0.0, z);
}

void KeyboardMovementController::roll()
{
    roll(_cw * _rotationFactor);
}

void KeyboardMovementController::roll(const double phi)
{
    rotate(phi, 0.0, 0.0);
}

void KeyboardMovementController::pitch()
{
    pitch(_cw * _rotationFactor);
}

void KeyboardMovementController::pitch(const double theta)
{
    rotate(0.0, theta, 0.0);
}

void KeyboardMovementController::yaw()
{
    yaw(_cw * _rotationFactor);
}

void KeyboardMovementController::yaw(const double psi)
{
    rotate(0.0, 0.0, psi);
}

void KeyboardMovementController::rotate(const double phi, const double theta, const double psi)
{
    viz::control::MovementParameters mp;
    initMovementParams(mp);
    mp.phi = phi;
    mp.theta = theta;
    mp.psi = psi;
    moveTo(mp);
}

double KeyboardMovementController::getMovementFactor() const
{
    return _movementFactor;
}

double KeyboardMovementController::getRotationFactor() const
{
    return _rotationFactor;
}

void KeyboardMovementController::setMovementFactor(const double factor)
{
    if (factor > 0.0)
    {
        _movementFactor = factor;
    }

}

void KeyboardMovementController::setRotationFactor(const double factor)
{
    if (factor > 0.0)
    {
        _rotationFactor = factor;
    }
}

void KeyboardMovementController::setRotateClockwise()
{
    _cw = 1.0;
}

void KeyboardMovementController::setRotateCounterClockwise()
{
    _cw = -1.0;
}


viz::control::MovementParameters_t& KeyboardMovementController::getPreviousMovement()
{
    if (_history.size())
    {
        return _history.back();
    }
    throw NullMovement;
}

const viz::control::MovementParameters_t& KeyboardMovementController::getConstPreviousMovement() const
{
    if (_history.size())
    {
        return _history.back();
    }
    throw NullMovement;
}

std::vector<viz::control::MovementParameters_t>& KeyboardMovementController::getMovementHistory()
{
    return _history;
}
const std::vector<viz::control::MovementParameters_t>& KeyboardMovementController::getMovementHistory() const
{
    return _history;
}

void KeyboardMovementController::clearHistory()
{
    _history.clear();
    _history.push_back(NullMovement);
}

// protected

void KeyboardMovementController::moveTo(const viz::control::MovementParameters_t &mp)
{
    _controllableObject.move(mp);
    _history.push_back(mp);
}
