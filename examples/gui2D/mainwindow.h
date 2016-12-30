#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>

#include <pathPlanner/NormalRRTPlanner.hpp>
#include <pathPlanner/PRMPlanner.hpp>
#include <pathPlanner/StarRRTPlanner.hpp>
#include <robot/PointRobot.h>
#include <ui/Drawing2D.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void loadImage();
    void computePath();
    void viewPath();
    void updateThreads(int num);
    void updateNodes(int num) ;
    void updatePlannerType(int type);
    void updateStartX(int value) ;
    void updateStartY(int value);
    void updateGoalX(int value);
    void updateGoalY(int value);
    void updateTrajectoryStepSize(double value);
    void updateSamplingType(int type);
    void updatePRMDistance(double value);
    void updateRRTStepSize(double value);

private:
    QImage convertCvMat(cv::Mat image);

    Ui::MainWindow *ui;
    QGraphicsScene *scene;

    unsigned int m_numThreads = 2;
    unsigned int m_numNodes = 5000;
    unsigned int m_plannerType = 0;
    int m_startX = 60;
    int m_startY = 60;
    int m_goalX = 870;
    int m_goalY = 870;
    double m_trajectoryStepSize = 0.5;
    unsigned int m_samplingType = 0;
    double m_prmDistance = 50;
    double m_rrtStepsize = 50;

    std::shared_ptr<rmpl::Planner<2>> m_planner;
    cv::Mat m_image;
    Eigen::MatrixXi m_workspace;
    bool m_connected = false;
};

#endif // MAINWINDOW_H
