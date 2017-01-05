#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
#include <QMainWindow>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>

#include <pathPlanner/NormalRRTPlanner.hpp>
#include <pathPlanner/PRMPlanner.hpp>
#include <pathPlanner/StarRRTPlanner.hpp>
#include <robot/PointRobot.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

  public slots:
    void loadImage();
    void computePath();
    void Threads(int num);
    void Nodes(int num);
    void PlannerType(int type);
    void StartX(int value);
    void StartY(int value);
    void GoalX(int value);
    void GoalY(int value);
    void TrajectoryStepSize(double value);
    void SamplerType(int type);
    void SamplingType(int type);
    void EdgeHeuristic(int type);
    void PRMDistance(double value);
    void RRTStepSize(double value);
    void weightVecX(double value);
    void weightVecY(double value);

  private:
    void viewPath();
    QImage convertCvMat(cv::Mat image);

    Ui::MainWindow *ui;
    QGraphicsScene *scene;

    unsigned int m_numThreads = 2;
    unsigned int m_numNodes = 5000;
    unsigned int m_plannerType = 0;
    int m_startX = 30;
    int m_startY = 30;
    int m_goalX = 870;
    int m_goalY = 870;
    double m_trajectoryStepSize = 1;
    unsigned int m_samplerType = 0;
    unsigned int m_samplingType = 0;
    unsigned int m_edgeHeuristic = 0;
    double m_prmDistance = 50;
    double m_rrtStepsize = 50;
    double m_weightVecX = 1;
    double m_weightVecY = 1;

    std::shared_ptr<rmpl::Planner<2>> m_planner;
    cv::Mat m_image;
    Eigen::MatrixXi m_workspace;
    bool m_connected = false;
    bool m_imageLoaded = false;
};

#endif    // MAINWINDOW_H
