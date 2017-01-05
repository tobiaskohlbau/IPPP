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

    cv::Mat image = cv::imread(qFileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
    if (!image.data)    // Check for invalid input
    {
        Logging::error("could not load image", "GUI");
        return;
    }

    m_workspace = drawing::cvToEigen(image);
    m_image = image;
    m_imageLoaded = true;
    Logging::info("loading image was successful", "GUI");
}

void MainWindow::computePath() {
    if (!m_imageLoaded) {
        Logging::error("image was not loaded", "GUI");
        return;
    }
    Logging::info("compute path", "GUI");
    std::cout << m_numNodes << std::endl;
    Vector2 minBoundary(0.0, 0.0);
    Vector2 maxBoundary(m_workspace.rows(), m_workspace.cols());
    std::shared_ptr<PointRobot> robot(new PointRobot(minBoundary, maxBoundary));
    robot->set2DWorkspace(m_workspace);

    SamplerMethod sampler = SamplerMethod::randomly;
    if (m_samplingType == 1)
        sampler = SamplerMethod::uniform;
    else if (m_samplerType == 2)
        sampler = SamplerMethod::standardDistribution;

    SamplingStrategy sampling = SamplingStrategy::normal;
    if (m_samplingType == 1)
        sampling = SamplingStrategy::nearObstacles;

    rmpl::EdgeHeuristic edgeH = rmpl::EdgeHeuristic::L2;
    if (m_edgeHeuristic == 1)
        edgeH = rmpl::EdgeHeuristic::L1;
    else if (m_edgeHeuristic == 2)
        edgeH = rmpl::EdgeHeuristic::INF;
    else if (m_edgeHeuristic == 3)
        edgeH = rmpl::EdgeHeuristic::WeightVec_L2;
    else if (m_edgeHeuristic == 4)
        edgeH = rmpl::EdgeHeuristic::WeightVec_L1;
    else if (m_edgeHeuristic == 5)
        edgeH = rmpl::EdgeHeuristic::WeightVec_INF;

    m_planner = nullptr;
    if (m_plannerType == 0) {
        RRTOptions options(m_rrtStepsize, m_trajectoryStepSize, sampler, sampling, edgeH);
        m_planner = std::shared_ptr<NormalRRTPlanner<2>>(new NormalRRTPlanner<2>(robot, options));
    } else if (m_plannerType == 1) {
        RRTOptions options(m_rrtStepsize, m_trajectoryStepSize, sampler, sampling, edgeH);
        m_planner = std::shared_ptr<StarRRTPlanner<2>>(new StarRRTPlanner<2>(robot, options));
    } else {
        PRMOptions options(m_prmDistance, m_trajectoryStepSize, sampler, sampling, edgeH);
        m_planner = std::shared_ptr<PRMPlanner<2>>(new PRMPlanner<2>(robot, options));
    }

    Vector2 start(m_startX, m_startY);
    Vector2 goal(m_goalX, m_goalY);
    m_connected = m_planner->computePath(start, goal, m_numNodes, m_numThreads);

    viewPath();
}

void MainWindow::viewPath() {
    cv::Mat image = m_image.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    std::vector<std::shared_ptr<Node<2>>> nodes = m_planner->getGraphNodes();
    drawing::drawTree2D<2>(nodes, image, Eigen::Vector3i(0, 0, 255), Eigen::Vector3i(0, 0, 0), 1);
    if (m_connected) {
        std::vector<Vector2> pathPoints = m_planner->getPath(1, true);
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

void MainWindow::Threads(int num) {
    m_numThreads = num;
};
void MainWindow::Nodes(int num) {
    m_numNodes = num;
};
void MainWindow::PlannerType(int type) {
    m_plannerType = type;
};
void MainWindow::StartX(int value) {
    m_startX = value;
};
void MainWindow::StartY(int value) {
    m_startY = value;
};
void MainWindow::GoalX(int value) {
    m_goalX = value;
};
void MainWindow::GoalY(int value) {
    m_goalX = value;
};
void MainWindow::TrajectoryStepSize(double value) {
    m_trajectoryStepSize = value;
};
void MainWindow::SamplingType(int type) {
    m_samplingType = type;
};
void MainWindow::SamplerType(int type) {
    m_samplerType = type;
};
void MainWindow::EdgeHeuristic(int type) {
    m_edgeHeuristic = type;
};
void MainWindow::PRMDistance(double value) {
    m_prmDistance = value;
};
void MainWindow::RRTStepSize(double value) {
    m_rrtStepsize = value;
};
void MainWindow::weightVecX(double value) {
    m_weightVecX = value;
    Vector2 weight(m_weightVecX, m_weightVecY);
    Heuristic<2>::setWeightVec(weight);
}
void MainWindow::weightVecY(double value) {
    m_weightVecY = value;
    Vector2 weight(m_weightVecX, m_weightVecY);
    Heuristic<2>::setWeightVec(weight);
}
