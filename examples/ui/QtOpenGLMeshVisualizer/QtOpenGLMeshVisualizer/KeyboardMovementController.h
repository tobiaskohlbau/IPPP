#ifndef KEYBOARDMOVEMENTCONTROLLER_H
#define KEYBOARDMOVEMENTCONTROLLER_H

#include <vector>

#include "VisualisationComponents/include/control/MovementController.h"
#include "VisualisationComponents/include/control/IMovementHistory.h"
#include <QKeyEvent>

namespace viz {
    namespace control {
        struct MovementParameters;
    }
}

class KeyboardMovementController : viz::control::MovementController, viz::control::IMovementHistory
{
public:
    KeyboardMovementController(viz::control::IMovementControllable &controllable);
    virtual ~KeyboardMovementController();

    void processKeyEvent(QKeyEvent *event);

    virtual void moveTo(const double x = 0.0, const double y = 0.0, const double z = 0.0) override;
    void up();
    virtual void up(const double y) override;
    void down();
    virtual void down(const double y) override;
    void left();
    virtual void left(const double x) override;
    void right();
    virtual void right(const double x) override;
    void backward();
    virtual void backward(const double z) override;
    void forward();
    virtual void forward(const double z) override;

    void roll();
    virtual void roll(const double phi) override;
    void pitch();
    virtual void pitch(const double theta) override;
    void yaw();
    virtual void yaw(const double psi) override;
    virtual void rotate(const double phi = 0.0, const double theta = 0.0, const double psi = 0.0) override;

    double getMovementFactor() const;
    double getRotationFactor() const;

    void setMovementFactor(const double factor);
    void setRotationFactor(const double factor);
    void setRotateClockwise();
    void setRotateCounterClockwise();

    //IMovementHistory
    viz::control::MovementParameters_t& getPreviousMovement() override;
    const viz::control::MovementParameters_t& getConstPreviousMovement() const override;
    std::vector<viz::control::MovementParameters_t>& getMovementHistory() override;
    const std::vector<viz::control::MovementParameters_t>& getMovementHistory() const override;
    void clearHistory() override;

protected:
    void moveTo(const viz::control::MovementParameters_t &mp);

private:
    double _movementFactor;
    double _rotationFactor;
    double _cw;

    std::vector<viz::control::MovementParameters_t> _history;
};

#endif // KEYBOARDMOVEMENTCONTROLLER_H
