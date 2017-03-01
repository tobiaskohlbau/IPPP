/*
 * IDrawable.h
 *
 *  Created on: 26.08.2015
 *      Author: robert
 */

#ifndef INCLUDE_IDRAWABLE_H_
#define INCLUDE_IDRAWABLE_H_

namespace viz
{

class IDrawable
{
public:
	IDrawable() { }
	virtual ~IDrawable() { }

	virtual void draw() = 0;
	virtual void hide() = 0;
	virtual void show() = 0;
	virtual bool isHidden() const = 0;
};

} /* namespace viz */

#endif /* INCLUDE_IDRAWABLE_H_ */
