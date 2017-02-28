/*
 * IMovementController.h
 *
 *  Created on: 08.10.2015
 *      Author: robert
 */

#ifndef VISUALISATIONLIB_SRC_CONTROL_MOVEMENTCONTROLLER_H_
#define VISUALISATIONLIB_SRC_CONTROL_MOVEMENTCONTROLLER_H_

namespace viz
{
namespace control
{

class IMovementControllable;

class MovementController
{
public:
	MovementController(IMovementControllable &controllableObject);
	virtual ~MovementController();

	void setControllableObject(IMovementControllable &controllableObject);
	IMovementControllable& getControllableObject();

	virtual void moveTo(const double x, const double y, const double z) = 0;
	virtual void up(const double y) = 0;
	virtual void down(const double y) = 0;
	virtual void left(const double x) = 0;
	virtual void right(const double x) = 0;
	virtual void backward(const double z) = 0;
	virtual void forward(const double z) = 0;

	virtual void roll(const double phi) = 0;
	virtual void pitch(const double theta) = 0;
	virtual void yaw(const double psi) = 0;
	virtual void rotate(const double phi, const double theta, const double psi) = 0;
protected:
	IMovementControllable &_controllableObject;
};

} /* namespace control */
} /* namespace viz */

#endif /* VISUALISATIONLIB_SRC_CONTROL_MOVEMENTCONTROLLER_H_ */
