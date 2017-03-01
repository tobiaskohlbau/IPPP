/*
 * IDrawFilter.h
 *
 *  Created on: 02.09.2015
 *      Author: robert
 */

#ifndef INCLUDE_IDRAWFILTER_H_
#define INCLUDE_IDRAWFILTER_H_

namespace viz
{

template<typename TFilter>
class IDrawFilter
{
public:
	IDrawFilter() { }
	virtual ~IDrawFilter() { }

	virtual void setDrawFilter(const TFilter filterValue) = 0;
	virtual void clearFilter() = 0;
};

} /* namespace viz */

#endif /* INCLUDE_IDRAWFILTER_H_ */
