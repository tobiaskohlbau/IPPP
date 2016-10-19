#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>

#include <pathPlanner/NormalRRTPlanner.h>
#include <pathPlanner/PRMPlanner.h>
#include <pathPlanner/StarRRTPlanner.h>
#include <robot/PointRobot.h>
#include <ui/Drawing.h>

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
    void updateThreads(int num) {m_numThreads = num;}
    void updateNodes(int num) {m_numNodes = num;}
    void updatePlannerType(int type) {m_plannerType = type;}
    void updateStartX(int value) {m_startX = value;}
    void updateStartY(int value) {m_startY = value;}
    void updateGoalX(int value) {m_goalX = value;}
    void updateGoalY(int value) {m_goalX = value;}
    void updateTrajectoryStepSize(double value) {m_trajectoryStepSize = value;}
    void updateSamplingType(int type) {m_samplingType = type;}
    void updatePRMDistance(double value) {m_prmDistance = value;}
    void updateRRTStepSize(double value) {m_rrtStepsize = value;}


private:
    Ui::MainWindow *ui;

    unsigned int m_numThreads = 2;
    unsigned int m_numNodes = 5000;
    unsigned int m_plannerType = 0;
    int m_startX = 30;
    int m_startY = 50;
    int m_goalX = 870;
    int m_goalY = 870;
    double m_trajectoryStepSize = 0.5;
    unsigned int m_samplingType = 0;
    double m_prmDistance = 50;
    double m_rrtStepsize = 50;

    rmpl::Planner *m_planner;
    cv::Mat m_image;
    Eigen::MatrixXi m_workspace;
    bool m_connected = false;

};

#endif // MAINWINDOW_H
