/*
 * ColorVisualiser.cpp
 *
 *  Created on: 12.02.2016
 *      Author: robert
 */
#include <GL/gl.h>

#include <stdexcept>

#include "../include/ColorVisualiser.h"

namespace viz
{

void ColorVisualiser::setOpenGLDrawColour(const viz::IColourSetable::RGBA_t &colour)
{
	const float r = colour[0]/255.0f;
	const float g = colour[1]/255.0f;
	const float b = colour[2]/255.0f;
	glColor3f(r, g, b);
}

void ColorVisualiser::setOpenGLDrawColour(const viz::IColourSetable::RGBA_Float_t &colour)
{
	glColor3f(colour[0], colour[1], colour[2]);
}

ColorVisualiser::ColorVisualiser()
{

}

ColorVisualiser::~ColorVisualiser()
{
}

// IColourSetable
void ColorVisualiser::setColourRGBA(const uchar r, const uchar g, const uchar b, const uchar alpha /* = 255 */)
{
	const uchar rgba[4] = { r, g, b, alpha };
	setColourRGBA(rgba);
}

void ColorVisualiser::setColourRGB(const uchar rgb[3])
{
	const uchar rgba[4] = { rgb[0], rgb[1], rgb[2], 255 };
	setColourRGBA(rgba);
}

void ColorVisualiser::setColourRGBA(const uchar rgba[4])
{
	_colour[0] = rgba[0];
	_colour[1] = rgba[1];
	_colour[2] = rgba[2];
	_colour[3] = rgba[3];
	setColourAsFloat(rgba[0]/255.0f, rgba[1]/255.0f, rgba[2]/255.0f, rgba[3]/255.0f);
}

void ColorVisualiser::setColourRGBA(const float r, const float g, const float b, const float alpha /*= 1.0 */)
{
	if (r < 0.0 || r > 1.0 || g < 0.0 || g > 1.0 || b < 0.0 || b > 1.0 || alpha < 0.0 || alpha > 1.0)
	{
		return;
	}
	const float MaxVal = 255.0;
	setColourRGBA((uchar)(r*MaxVal), (uchar)(g*MaxVal), (uchar)(b*MaxVal), (uchar)(alpha*MaxVal));
}

const IColourSetable::RGB_t& ColorVisualiser::getColourRGB() const
{
	const IColourSetable::RGB_t& c= (IColourSetable::RGB_t &)_colour;
	return c;
}

const IColourSetable::RGBA_t& ColorVisualiser::getColourRGBA() const
{
	return _colour;
}

const viz::IColourSetable::RGB_Float_t& ColorVisualiser::getColourRGBFloat() const
{
	return (IColourSetable::RGB_Float_t &)_floatColour;
}

const viz::IColourSetable::RGBA_Float_t& ColorVisualiser::getColourRGBAFloat() const
{
	return _floatColour;
}


// protected
void ColorVisualiser::setGLColour() const
{
	setOpenGLDrawColour(getColourRGBAFloat());
}

void ColorVisualiser::setColourAsFloat(const float r, const float g, const float b, const float alpha)
{
	if (r < 0.0 || r > 1.0 || g < 0.0 || g > 1.0 || b < 0.0 || b > 1.0 || alpha < 0.0 || alpha > 1.0)
	{
		throw std::invalid_argument("colour channel");
	}
	_floatColour[0] = r;
	_floatColour[1] = g;
	_floatColour[2] = b;
	_floatColour[3] = alpha;
}

} /* namespace viz */
