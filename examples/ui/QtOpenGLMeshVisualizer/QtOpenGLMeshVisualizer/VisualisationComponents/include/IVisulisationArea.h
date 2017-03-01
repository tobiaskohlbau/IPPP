/*
 * IVisulisationArea.h
 *
 *  Created on: 18.12.2015
 *      Author: robert
 */

#ifndef INCLUDE_IVISULISATIONAREA_H_
#define INCLUDE_IVISULISATIONAREA_H_

#include <cmath>

namespace viz
{

typedef struct VisualisationArea3D
{
	VisualisationArea3D() : minX(NAN), maxX(NAN), minY(NAN), maxY(NAN), minZ(NAN), maxZ(NAN) {}

	double minX, maxX, minY, maxY, minZ, maxZ;
} VisualisationArea3D;

class IVisulisationArea
{
public:
	IVisulisationArea() { }
	virtual ~IVisulisationArea() { }

	virtual void clearArea() = 0;
	virtual void setArea(const VisualisationArea3D &area) = 0;
	virtual VisualisationArea3D& getArea() = 0;
	virtual const VisualisationArea3D& getAreaConst() const = 0;
};

} /* namespace viz */

#endif /* INCLUDE_IVISULISATIONAREA_H_ */
