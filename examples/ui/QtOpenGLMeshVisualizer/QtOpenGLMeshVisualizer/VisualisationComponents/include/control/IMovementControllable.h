/*
 * IMovementControllable.h
 *
 *  Created on: 08.10.2015
 *      Author: robert
 */

#ifndef VISUALISATIONLIB_SRC_CONTROL_IMOVEMENTCONTROLLABLE_H_
#define VISUALISATIONLIB_SRC_CONTROL_IMOVEMENTCONTROLLABLE_H_

#include <limits>

namespace viz
{
namespace control
{

static const double inf = std::numeric_limits<double>::infinity();
static const double null = 0.0;
static const double D = null; // default

typedef struct MovementParameters
{
	MovementParameters() : x(D), y(D), z(D), phi(D), theta(D), psi(D) { }
	double x, y, z;
	double phi, theta, psi;
} MovementParameters_t;

class IMovementControllable
{
public:
	IMovementControllable() { }
	virtual ~IMovementControllable() { }

	virtual void move(const MovementParameters_t &movementParams) = 0;
	virtual void move(const double x, const double y, const double z) = 0;
	virtual void rotate(const double phi, const double theta, const double psi) = 0;
};

} /* namespace control */
} /* namespace viz */

#endif /* VISUALISATIONLIB_SRC_CONTROL_IMOVEMENTCONTROLLABLE_H_ */
