/*
 * VisualiserBase.cpp
 *
 *  Created on: 26.08.2015
 *      Author: robert
 */

#include "../include/VisualiserBase.h"

namespace viz
{

VisualiserBase::VisualiserBase() : _isHidden(false)
{
}

VisualiserBase::~VisualiserBase()
{
}

void VisualiserBase::hide()
{
	_isHidden = true;
	draw();
}

void VisualiserBase::show()
{
	_isHidden = false;
	draw();
}

bool VisualiserBase::isHidden() const
{
	return _isHidden;
}

} /* namespace viz */
