/*
 * MeshOpenGLVisualiser.h
 *
 *  Created on: 11.02.2016
 *      Author: robert
 */

#ifndef MeshOpenGLVisualiser_H_
#define MeshOpenGLVisualiser_H_

#include <list>
#include <vector>
#include <Eigen/Dense>

#include <VisualisationComponents/include/IColourSetable.h>
//#include <geoPlane.h>

#include "VisualisationComponents/include/ColorVisualiser.h"


typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef std::vector<Vector3f> Vertex3Vector;
typedef std::vector<Vertex3Vector> TriangleVector;


namespace viz
{

class MeshOpenGLVisualiser : public ColorVisualiser
{
public:
    MeshOpenGLVisualiser();
    explicit MeshOpenGLVisualiser(const TriangleVector &triangles);
    virtual ~MeshOpenGLVisualiser();

	// IDrawable
	void draw() override final;

    void setTriangles(const TriangleVector &triangles);
    const TriangleVector& getTriangles() const;
private:
    TriangleVector _triangles;
};

} /* namespace viz */

#endif /* MeshOpenGLVisualiser_H_ */
