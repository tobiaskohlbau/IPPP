#ifndef OPENGLVISUALIZERWIDGET_H
#define OPENGLVISUALIZERWIDGET_H

#include <list>

#include <QGLWidget>
#include <QtCore/QReadWriteLock>
#include <QtCore/QMutex>
#include <QtCore/QWaitCondition>

#include <QtOpenGL/QGLWidget>
#include <QtGui/QMouseEvent>
#include <QtGui/QWheelEvent>
#include <QtGui/QKeyEvent>
#include <QtCore/QMutex>
#include <GL/glut.h>

#include "VisualisationComponents/include/IVisualiser.h"
#include "VisualisationComponents/include/IDrawFilter.h"

#include "OrbitControl.h"
#include "Trackball.h"


class OpenGLVisualizerWidget : public QGLWidget,
        public viz::IVisualiser
{
    Q_OBJECT
public:
    explicit OpenGLVisualizerWidget(QWidget *parent = 0);
    virtual ~OpenGLVisualizerWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    void keyPressEvent (QKeyEvent *event);

    void draw(viz::IDrawable &drawableInstance) override;
    void addVisualiser(viz::IDrawable &drawableInstance);
    void clearVisualiserList();
    void redraw() override;

private:
    void initializeGL();
    void paintGrid(double stepX,double stepY);
    void paintLines();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent ( QWheelEvent *event);

private:
    OrbitControl									 m_orbit;
    Eigen::Vector4f                                      m_background;
    bool _worldFrame;

    std::list<viz::IDrawable *> _drawList;
    QMutex _visualiserMutex;
    bool _showGrid;

signals:
    void clicked();
public slots:

};

#endif // OPENGLVISUALIZERWIDGET_H
