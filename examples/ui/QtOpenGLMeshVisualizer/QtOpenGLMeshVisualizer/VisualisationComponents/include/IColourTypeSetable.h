/*
 * IColourTypeSetable.h
 *
 *  Created on: 03.09.2015
 *      Author: robert
 */

#ifndef INCLUDE_ICOLOURTYPESETABLE_H_
#define INCLUDE_ICOLOURTYPESETABLE_H_

namespace viz
{

template<typename TType>
class IColourTypeSetable : public virtual IColourSetable
{
public:
    IColourTypeSetable() { }
    virtual ~IColourTypeSetable() { }

	virtual void setColourRGB(const TType drawableType, const IColourSetable::RGB_t &rgba) = 0;
	virtual void setColourRGBA(const TType drawableType, const IColourSetable::RGBA_t &rgba) = 0;

	virtual const RGB_t& getColourRGB(const TType drawableType) const = 0;
	virtual const RGBA_t& getColourRGBA(const TType drawableType) const = 0;
};

} /* namespace viz */

#endif /* INCLUDE_ICOLOURTYPESETABLE_H_ */
