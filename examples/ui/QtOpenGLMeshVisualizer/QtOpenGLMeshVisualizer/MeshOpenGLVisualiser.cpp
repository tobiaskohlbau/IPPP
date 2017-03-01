/*
 * MeshOpenGLVisualiser.cpp
 *
 *  Created on: 11.02.2016
 *      Author: robert
 */

#include <GL/gl.h>

#include "MeshOpenGLVisualiser.h"

#include <Eigen/Dense>

namespace viz
{

MeshOpenGLVisualiser::MeshOpenGLVisualiser()
{
	setColourRGBA(0.8f, 0.8f, 0.8f, 1.0f);
}

MeshOpenGLVisualiser::MeshOpenGLVisualiser(const TriangleVector &triangles) : MeshOpenGLVisualiser()
{
    setTriangles(triangles);
}

MeshOpenGLVisualiser::~MeshOpenGLVisualiser()
{
}

void MeshOpenGLVisualiser::draw()
{
	if (isHidden())
	{
		return;
	}
    glLineWidth(1.0);
    glBegin(GL_TRIANGLES);
		setGLColour();
        if (_triangles.size() > 2)
        {
            for (size_t i = 0; i < _triangles.size(); ++i)
            {
                glVertex3f(_triangles[i][0](0), _triangles[i][0](1), _triangles[i][0](2));
                glVertex3f(_triangles[i][1](0), _triangles[i][1](1), _triangles[i][1](2));
                glVertex3f(_triangles[i][2](0), _triangles[i][2](1), _triangles[i][2](2));
            }
        }

	glEnd();
	glLineWidth(1.0);
}

void MeshOpenGLVisualiser::setTriangles(const TriangleVector &triangles)
{
    _triangles = triangles;
	draw();
}

const TriangleVector& MeshOpenGLVisualiser::getTriangles() const
{
    return _triangles;
}

} /* namespace viz */
