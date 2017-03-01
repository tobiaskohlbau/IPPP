/*
 * IVisualiser.h
 *
 *  Created on: 27.08.2015
 *      Author: robert
 */

#ifndef INCLUDE_IVISUALISER_H_
#define INCLUDE_IVISUALISER_H_

namespace viz
{

class IDrawable;

class IVisualiser
{
public:
	IVisualiser() { }
	virtual ~IVisualiser() { }

	virtual void draw(IDrawable &drawable) = 0;
	virtual void redraw() = 0;
};

} /* namespace viz */

#endif /* INCLUDE_IVISUALISER_H_ */
