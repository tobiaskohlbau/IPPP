/*
 * IItemSelectable.h
 *
 *  Created on: 17.11.2015
 *      Author: robert
 */

#ifndef INCLUDE_IITEMSELECTABLE_H_
#define INCLUDE_IITEMSELECTABLE_H_

namespace viz
{

template<typename TSelectable>
class IItemSelectable
{
public:
	IItemSelectable() { }
	virtual ~IItemSelectable() { }

	virtual void clearItemSelection() = 0;
	virtual void selectItem(const TSelectable &item) = 0;
};

} /* namespace viz */

#endif /* INCLUDE_IITEMSELECTABLE_H_ */
