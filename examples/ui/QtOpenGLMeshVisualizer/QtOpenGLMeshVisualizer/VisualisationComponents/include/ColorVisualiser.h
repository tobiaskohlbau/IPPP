/*
 * ColourVisualiser.h
 *
 *  Created on: 12.02.2016
 *      Author: robert
 */

#ifndef COLOURVISUALISER_H_
#define COLOURVISUALISER_H_

#include "IColourSetable.h"
#include "VisualiserBase.h"

namespace viz
{

class ColorVisualiser: public viz::VisualiserBase, public viz::IColourSetable
{
protected:
    static void setOpenGLDrawColour(const viz::IColourSetable::RGBA_t &colour);
    static void setOpenGLDrawColour(const viz::IColourSetable::RGBA_Float_t &colour);

public:
    ColorVisualiser();
    virtual ~ColorVisualiser();

	// IColourSetable
	void setColourRGBA(const uchar r, const uchar g, const uchar b, const uchar alpha = 255) override;
	void setColourRGB(const uchar rgb[3]) override;
	void setColourRGBA(const uchar rgba[4]) override;
	void setColourRGBA(const float r, const float g, const float b, const float alpha = 1.0) override;
    const viz::IColourSetable::RGB_t& getColourRGB() const override;
    const viz::IColourSetable::RGBA_t& getColourRGBA() const override;

    const viz::IColourSetable::RGB_Float_t& getColourRGBFloat() const override;
    const viz::IColourSetable::RGBA_Float_t& getColourRGBAFloat() const override;

protected:
	void setGLColour() const;

	void setColourAsFloat(const float r, const float g, const float b, const float alpha);

private:
    viz::IColourSetable::RGBA_t _colour;
    viz::IColourSetable::RGBA_Float_t _floatColour;
};

} /* namespace viz */

#endif /* COLOURVISUALISER_H_ */
