#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <QVBoxLayout>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMainWindow>
#include <QLabel>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>

#include <core/module/Identifier.h>
#include <pathPlanner/NormalRRTPlanner.hpp>
#include <pathPlanner/PRMPlanner.hpp>
#include <pathPlanner/RRTStarPlanner.hpp>
#include <robot/PointRobot.h>
#include <robot/TriangleRobot2D.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow, public rmpl::Identifier {
    Q_OBJECT

  public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

  public slots:
    void loadConfig();
    void saveConfig();
    void loadImage();
    void computePath();
    void updateThreads(int num);
    void updateNodes(int num);
    void updatePlannerType(int type);
    void updateRobotType(int type);
    void updateStartX(int value);
    void updateStartY(int value);
    void updateStartPhi(double value);
    void updateGoalX(int value);
    void updateGoalY(int value);
    void updateGoalPhi(double value);
    void updateTrajectoryStepSize(double value);
    void updateSamplerType(int type);
    void updateSamplingType(int type);
    void updateEdgeHeuristic(int type);
    void updatePRMDistance(double value);
    void updateRRTStepSize(double value);
    void updateWeightVecX(double value);
    void updateWeightVecY(double value);
    void updateWeightVecZ(double value);
  signals:
    void setNumThreads(int value);
    void setNumNodes(int value);
    void setPlannerType(int type);
    void setRobotType(int type);
    void setTrajectoryStepSize(double value);
    void setSamplerMethod(int type);
    void setSamplingStrategy(int type);
    void setPrmDistance(double value);
    void setRrtStepSize(double value);
    void setStartX(int value);
    void setStartY(int value);
    void setStartPhi(double value);
    void setGoalX(int value);
    void setGoalY(int value);
    void setGoalPhi(double phi);
    void setEdgeHeuristic(int type);
    void setWeightVecX(double value);
    void setWeightVecY(double value);
    void setWeightVecZ(double value);

  private:
    void viewPath();
    QImage convertCvMat(cv::Mat image);

    Ui::MainWindow *ui;
    QWidget *m_widget;
    QLayout * m_layout;

    unsigned int m_numThreads = 2;
    unsigned int m_numNodes = 5000;
    unsigned int m_plannerType = 0;
    unsigned int m_robotType = 0;
    int m_startX = 30;
    int m_startY = 30;
    double m_startPhi = 0;
    int m_goalX = 870;
    int m_goalY = 870;
    double m_goalPhi = 0;
    double m_trajectoryStepSize = 1;
    unsigned int m_samplerMethod = 0;
    unsigned int m_samplingStrategy = 0;
    unsigned int m_edgeHeuristic = 0;
    double m_prmDistance = 50;
    double m_rrtStepsize = 50;
    double m_weightVecX = 1;
    double m_weightVecY = 1;
    double m_weightVecZ = 1;
    QString m_configPath;

    cv::Mat m_image;
    Eigen::MatrixXi m_workspace;

    std::vector<rmpl::Triangle2D> m_triangles;
    std::shared_ptr<rmpl::Planner<2>> m_planner2d = nullptr;
    std::shared_ptr<rmpl::Planner<3>> m_planner3d = nullptr;
    rmpl::Vector3 m_weightVec;
    bool m_connected = false;
    bool m_imageLoaded = false;
};

#endif    // MAINWINDOW_H
