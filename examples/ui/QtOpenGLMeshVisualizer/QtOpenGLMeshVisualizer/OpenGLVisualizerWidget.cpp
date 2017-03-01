#include "OpenGLVisualizerWidget.h"

#include "VisualisationComponents/include/IDrawable.h"

OpenGLVisualizerWidget::OpenGLVisualizerWidget(QWidget *parent) :
    QGLWidget(parent)
{
    _worldFrame = true;
    //m_background = math::Vec4d(0.16,0.21,0.36,1.0);
    const float ccv = 0.87;
    m_background = Eigen::Vector4f(ccv,ccv, 0.7, 1.0);
    _showGrid = true;
}

OpenGLVisualizerWidget::~OpenGLVisualizerWidget()
{
    _drawList.clear();
}

QSize OpenGLVisualizerWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize OpenGLVisualizerWidget::sizeHint() const
{
    return QSize(1600, 1000);
}

void OpenGLVisualizerWidget::paintGrid(double stepX,double stepY)
{
    if (!_showGrid)
    {
        return;
    }
    // painting grid
    glBegin(GL_LINES);
    const float lineColor = 0.0;
    glColor3f(lineColor,lineColor,lineColor);
        //glColor3f(1.0,1.0,1.0);
    for(double y=-1000.0; y < 1001;y=y+stepY){
       glVertex3f(-1000.0,y,0.0);
       glVertex3f(1000.0,y,0.0);
    }
    for(double x=-1000.0; x < 1001;x=x+stepX){
       glVertex3f(x,-1000.0,0.0);
       glVertex3f(x,1000.0,0.0);
    }
    glEnd();
}

void OpenGLVisualizerWidget::draw(viz::IDrawable &drawableInstance)
{
    addVisualiser(drawableInstance);
}

void OpenGLVisualizerWidget::addVisualiser(viz::IDrawable &drawableInstance)
{
    _visualiserMutex.lock();
    _drawList.push_back(&drawableInstance);
    _visualiserMutex.unlock();
    update();
}

void OpenGLVisualizerWidget::clearVisualiserList()
{
    _visualiserMutex.lock();
    _drawList.clear();
    _visualiserMutex.unlock();
}

void OpenGLVisualizerWidget::initializeGL()
{

   glClearColor(m_background(0), m_background(1), m_background(2), m_background(3));
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glShadeModel(GL_SMOOTH);
//	glShadeModel(GL_FLAT);
   glEnable(GL_DEPTH_TEST);
   glDisable(GL_CULL_FACE);
   //glEnable(GL_CULL_FACE);

   GLfloat light_ambient[4] = {0.2f, 0.2f, 0.2f, 1.0f};
   GLfloat light_diffuse[4] = {0.8f, 0.8f, 0.8f, 1.0f};
   GLfloat light_specular[4] = {0.5f, 0.5f, 0.5f, 1.0f};
//	GLfloat light_position[4] = {0.0f, 0.0f, -1.0f, 1.0f};
   GLfloat light_position1[4] = {0.0f, 0.0f, -1.0f, 0.0f};
   GLfloat light_position2[4] = {0.0f, 0.0f, 1.0f, 0.0f};
//	GLfloat spot_direction[]  = {-1.0,-1.0,0.0};

   glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
   glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
   glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
   glLightfv(GL_LIGHT0, GL_POSITION, light_position1);
   glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
   glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
   glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
   glLightfv(GL_LIGHT1, GL_POSITION, light_position2);


   glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION,1.5);
   glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION,0.5);
   glLightf(GL_LIGHT0,GL_QUADRATIC_ATTENUATION,0.2);
//   glLightf(GL_LIGHT0,GL_SPOT_CUTOFF,90.0);
//   glLightfv(GL_LIGHT0,GL_SPOT_DIRECTION,spot_direction);
//   glLightf(GL_LIGHT0,GL_SPOT_EXPONENT,2.0);

   glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION,1.5);
   glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION,0.5);
   glLightf(GL_LIGHT1,GL_QUADRATIC_ATTENUATION,0.2);
/*    glLightf(GL_LIGHT0,GL_SPOT_CUTOFF,90.0);
   glLightfv(GL_LIGHT0,GL_SPOT_DIRECTION,spot_direction);
   glLightf(GL_LIGHT0,GL_SPOT_EXPONENT,2.0);
 */

   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_LIGHT1);


//	glEnable(GL_LIGHT1);

   glColorMaterial(GL_FRONT_AND_BACK, GL_EMISSION);
   glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
   glEnable(GL_COLOR_MATERIAL);

}

void OpenGLVisualizerWidget::paintGL()
{


    glClearColor(m_background(0), m_background(1), m_background(2), m_background(3));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//     glMatrixMode(GL_MODELVIEW);
//     glLoadIdentity();

//     glTranslated(-(bbminx+bbmaxx)/2.0,-(bbminy+bbmaxy)/2.0,-(bbminz+bbmaxz)/2.0);
    glLineWidth(3.0);

   GLdouble *current=(GLdouble*) new double[16];

   if(_worldFrame)
   {
        if (_showGrid)
        {
            glBegin(GL_LINES);
            {
                glColor3f(1.0,0.0,0.0);
                glVertex3f(0.0,0.0,0.0);
                glVertex3f(100.0,0.0,0.0);
                glColor3f(0.0,1.0,0.0);
                glVertex3f(0.0,0.0,0.0);
                glVertex3f(0.0,100.0,0.0);
                glColor3f(1.0,1.0,0.0);
                glVertex3f(0.0,0.0,0.0);
                glVertex3f(0.0,0.0,100.0);
            }
            glEnd();
            paintGrid(250.0,250.0);
        }
   }
   glLineWidth(1.0);
   glPointSize(1.0);
    _visualiserMutex.lock();
   for(auto drawable : _drawList)
   {
       drawable->draw();
   }
   _visualiserMutex.unlock();

    glFlush();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
 //   m_trackball.use();
    m_orbit.use();
    delete [] current;
}

void OpenGLVisualizerWidget::redraw()
{
    updateGL();
}

void OpenGLVisualizerWidget::resizeGL(int width, int height)
{

    glViewport(0,0, (GLsizei) width, (GLsizei) height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//	 glFrustum(-4000.0,4000.0,-4000.0,4000.0,1.0,4000.0f);
    gluPerspective(60.0, (GLfloat) width/(GLfloat) height,0.1f,(GLfloat) 4000.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
//	 m_trackball.use();
    m_orbit.use();
}

void OpenGLVisualizerWidget::mousePressEvent(QMouseEvent *event)
{
    if(event->buttons() & Qt::LeftButton){
       // m_trackball.mouse(0,1,event->pos().x(),event->pos().y());
        m_orbit.mousePress(0,1,event->pos().x(),event->pos().y());
    }else if(event->buttons() & Qt::RightButton){
        m_orbit.mousePress(1,1,event->pos().x(),event->pos().y());
       // m_trackball.mouse(1,1,event->pos().x(),event->pos().y());
    }else if(event->buttons() & Qt::MidButton){
        m_orbit.mousePress(2,1,event->pos().x(),event->pos().y());
   //	 m_trackball.mouse(2,1,event->pos().x(),event->pos().y());
    }
    updateGL();
}

void OpenGLVisualizerWidget::mouseReleaseEvent(QMouseEvent *event)
 {
    if(event->buttons() & Qt::LeftButton){
       // m_trackball.mouse(0,0,event->pos().x(),event->pos().y());
        m_orbit.mousePress(0,0,event->pos().x(),event->pos().y());
    }else if(event->buttons() & Qt::RightButton){
        m_orbit.mousePress(1,0,event->pos().x(),event->pos().y());
       // m_trackball.mouse(1,0,event->pos().x(),event->pos().y());
    }else if(event->buttons() & Qt::MidButton){
       m_orbit.mousePress(2,0,event->pos().x(),event->pos().y());
       // m_trackball.mouse(2,0,event->pos().x(),event->pos().y());
    }
    updateGL();
 }


void OpenGLVisualizerWidget::mouseMoveEvent(QMouseEvent *event)
{
   // m_trackball.mouseMotion(event->x(),event->y());
    m_orbit.mouseMove(event->x(),event->y());
    updateGL();
}
void OpenGLVisualizerWidget::wheelEvent(QWheelEvent *event)
{
   //m_trackball.zoom(event->delta());
   m_orbit.zoom(event->delta());
   updateGL();

}

void OpenGLVisualizerWidget::keyPressEvent (QKeyEvent *event)
{
    switch(event->key())
    {
        case Qt::Key_F1:
            m_orbit.reset();
            updateGL();
            break;
        case Qt::Key_F2:
            m_orbit.reset();
            m_orbit.upsidedown();
            updateGL();
            break;
        case Qt::Key_F3:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            updateGL();
            break;
        case Qt::Key_F4:
            glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
            updateGL();
            break;
        case Qt::Key_F5:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            updateGL();
            break;
        case Qt::Key_G:
            _showGrid = false;
            updateGL();
        default:
            return;
    }
}
