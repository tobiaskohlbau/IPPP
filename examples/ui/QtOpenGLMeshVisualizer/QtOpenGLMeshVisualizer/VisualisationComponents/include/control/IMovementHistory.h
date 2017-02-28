/*
 * IMovementHistory.h
 *
 *  Created on: 12.02.2016
 *      Author: robert
 */

#ifndef INCLUDE_CONTROL_IMOVEMENTHISTORY_H_
#define INCLUDE_CONTROL_IMOVEMENTHISTORY_H_

#include <vector>

#include "IMovementControllable.h"

namespace viz
{
namespace control
{

class IMovementHistory
{
public:
	IMovementHistory() { }
	virtual ~IMovementHistory() { }

	virtual MovementParameters_t& getPreviousMovement() = 0;
	virtual const MovementParameters_t& getConstPreviousMovement() const = 0;
	virtual std::vector<MovementParameters_t>& getMovementHistory() = 0;
	virtual const std::vector<MovementParameters_t>& getMovementHistory() const = 0;

	virtual void clearHistory() = 0;
};

} /* namespace control */
} /* namespace viz */

#endif /* INCLUDE_CONTROL_IMOVEMENTHISTORY_H_ */
