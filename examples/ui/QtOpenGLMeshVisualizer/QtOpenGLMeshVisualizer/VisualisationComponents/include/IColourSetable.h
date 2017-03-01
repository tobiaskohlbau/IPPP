/*
 * IColourChangeable.h
 *
 *  Created on: 03.09.2015
 *      Author: robert
 */

#ifndef INCLUDE_ICOLOURSETABLE_H_
#define INCLUDE_ICOLOURSETABLE_H_

typedef unsigned char uchar;

namespace viz
{

class IColourSetable
{
public:
	typedef uchar RGB_t[3];
	typedef uchar RGBA_t[4];
	typedef float RGB_Float_t[3];
	typedef float RGBA_Float_t[4];

public:
	IColourSetable();
	virtual ~IColourSetable();

	virtual void setColourRGBA(const uchar r, const uchar g, const uchar b, const uchar alpha = 255) = 0;
	virtual void setColourRGB(const uchar rgb[3]) = 0;
	virtual void setColourRGBA(const uchar rgba[4]) = 0;
	virtual void setColourRGBA(const float r, const float g, const float b, const float alpha = 1.0) = 0;

	virtual const RGB_t& getColourRGB() const = 0;
	virtual const RGBA_t& getColourRGBA() const = 0;

	virtual const RGB_Float_t& getColourRGBFloat() const = 0;
	virtual const RGBA_Float_t& getColourRGBAFloat() const = 0;
};

} /* namespace viz */

#endif /* INCLUDE_ICOLOURSETABLE_H_ */
