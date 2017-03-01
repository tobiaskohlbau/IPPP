/*
 * Test.cpp
 *
 *  Created on: 08.10.2015
 *      Author: robert
 */

#include "../../include/control/MovementController.h"
#include "../../include/control/IMovementControllable.h"

namespace viz
{
namespace control
{

MovementController::MovementController(IMovementControllable &controllableObject)
	: _controllableObject(controllableObject)
{
}

MovementController::~MovementController()
{
}

void MovementController::setControllableObject(IMovementControllable &controllableObject)
{
	_controllableObject = controllableObject;
}

IMovementControllable& MovementController::getControllableObject()
{
	return _controllableObject;
}

} /* namespace control */
} /* namespace viz */
