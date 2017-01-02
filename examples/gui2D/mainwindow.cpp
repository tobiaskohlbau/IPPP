#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>

#include <QFileDialog>
#include <core/types.h>
#include <pathPlanner/options/PRMOptions.h>
#include <ui/Drawing2D.hpp>

using namespace rmpl;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::loadImage() {
    QString qFileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home", tr("Image Files (*.png *.jpg *.bmp)"));

    cv::Mat obstacleWorkspace = cv::imread(qFileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
    m_workspace = drawing::cvToEigen(obstacleWorkspace);
    m_image = obstacleWorkspace;
}

void MainWindow::computePath() {
    std::cout << "compute path" << std::endl;
    std::cout << m_numNodes << std::endl;
    Vector2 minBoundary(0.0, 0.0);
    Vector2 maxBoundary(m_workspace.rows(), m_workspace.cols());
    std::shared_ptr<PointRobot> robot(new PointRobot(minBoundary, maxBoundary));
    robot->set2DWorkspace(m_workspace);

    SamplerMethod sampling = SamplerMethod::randomly;
    if (m_samplingType == 1)
        sampling = SamplerMethod::uniform;
    if (m_samplingType == 2)
        sampling = SamplerMethod::standardDistribution;

    m_planner = nullptr;
    if (m_plannerType == 0) {
        RRTOptions options(m_rrtStepsize, m_trajectoryStepSize, sampling);
        m_planner = std::shared_ptr<NormalRRTPlanner<2>>(new NormalRRTPlanner<2>(robot, options));
    } else if (m_plannerType == 1) {
        RRTOptions options(m_rrtStepsize, m_trajectoryStepSize, sampling);
        m_planner = std::shared_ptr<StarRRTPlanner<2>>(new StarRRTPlanner<2>(robot, options));
    } else {
        PRMOptions options(m_rrtStepsize, m_trajectoryStepSize, sampling);
        m_planner = std::shared_ptr<PRMPlanner<2>>(new PRMPlanner<2>(robot, options));
    }

    Vector2 start(m_startX, m_startY);
    Vector2 goal(m_goalX, m_goalY);
    m_connected = m_planner->computePath(start, goal, m_numNodes, m_numThreads);
}

void MainWindow::viewPath() {
    std::cout << "view path" << std::endl;
    cv::Mat image = m_image.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    std::vector<std::shared_ptr<Node<2>>> nodes = m_planner->getGraphNodes();
    drawing::drawTree2D<2>(nodes, image, Eigen::Vector3i(0, 0, 255), Eigen::Vector3i(0, 0, 0), 1);
    if (m_connected) {
        std::vector<Vector2> pathPoints = m_planner->getPath(0.5, true);
        drawing::drawPath2D(pathPoints, image, Eigen::Vector3i(255, 0, 0), 3);
    }

    QImage qImage = convertCvMat(image);

    QPixmap qPixmap = QPixmap::fromImage(qImage);
    scene = new QGraphicsScene(this);
    scene->addPixmap(qPixmap);
    scene->setSceneRect(qPixmap.rect());

    ui->mainImage->setScene(scene);
}

QImage MainWindow::convertCvMat(cv::Mat inMat) {
    switch (inMat.type()) {
        // 8-bit, 4 channel
        case CV_8UC4: {
            QImage image(inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_ARGB32);
            return image;
        }
        // 8-bit, 3 channel
        case CV_8UC3: {
            QImage image(inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_RGB888);
            return image.rgbSwapped();
        }
        // 8-bit, 1 channel
        case CV_8UC1: {
            static QVector<QRgb> sColorTable(256);

            // only create our color table the first time
            if (sColorTable.isEmpty()) {
                for (int i = 0; i < 256; ++i) {
                    sColorTable[i] = qRgb(i, i, i);
                }
            }
            QImage image(inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_Indexed8);
            image.setColorTable(sColorTable);
            return image;
        }
    }

    return QImage();
}

void MainWindow::updateThreads(int num) {
    m_numThreads = num;
};
void MainWindow::updateNodes(int num) {
    m_numNodes = num;
};
void MainWindow::updatePlannerType(int type) {
    m_plannerType = type;
};
void MainWindow::updateStartX(int value) {
    m_startX = value;
};
void MainWindow::updateStartY(int value) {
    m_startY = value;
};
void MainWindow::updateGoalX(int value) {
    m_goalX = value;
};
void MainWindow::updateGoalY(int value) {
    m_goalX = value;
};
void MainWindow::updateTrajectoryStepSize(double value) {
    m_trajectoryStepSize = value;
};
void MainWindow::updateSamplingType(int type) {
    m_samplingType = type;
};
void MainWindow::updatePRMDistance(double value) {
    m_prmDistance = value;
};
void MainWindow::updateRRTStepSize(double value) {
    m_rrtStepsize = value;
};
