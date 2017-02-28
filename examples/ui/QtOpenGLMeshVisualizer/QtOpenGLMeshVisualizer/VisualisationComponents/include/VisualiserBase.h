/*
 * VisualiserBase.h
 *
 *  Created on: 26.08.2015
 *      Author: robert
 */

#ifndef VISUALISERBASE_H_
#define VISUALISERBASE_H_

#include "IDrawable.h"

namespace viz
{

class VisualiserBase : public IDrawable
{
public:
	VisualiserBase();
	virtual ~VisualiserBase();

	// IDrawable
	void hide() override;
	void show() override;
	bool isHidden() const override;

private:
	bool _isHidden;
};

} /* namespace viz */

#endif /* VISUALISERBASE_H_ */
