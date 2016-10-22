#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>

#include <QFileDialog>
#include <pathPlanner/PRMOptions.h>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::loadImage() {
    QString qFileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home", tr("Image Files (*.png *.jpg *.bmp)"));

    cv::Mat obstacleWorkspace = cv::imread(qFileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat dst;
    obstacleWorkspace.convertTo(dst, CV_32SC1);

    int rows = dst.rows;
    int cols = dst.cols;
    m_workspace = Eigen::MatrixXi(rows, cols);
    std::vector<int> entries;
    int* temp;
    for (int i = 0; i < cols; ++i) {
        temp = dst.ptr<int>(i);
        for (int j = 0; j < rows; ++j) {
            entries.push_back(*temp);
            ++temp;
        }
    }
    m_workspace = Eigen::MatrixXi::Map(&entries[0], rows, cols);
    m_image = obstacleWorkspace;
}

void MainWindow::computePath() {
    std::cout << "compute path" << std::endl;
    std::cout << m_numNodes << std::endl;
    rmpl::Vec<float> minBoundary(0.0, 0.0);
    rmpl::Vec<float> maxBoundary(m_workspace.rows(), m_workspace.cols());
    std::shared_ptr<rmpl::PointRobot> robot(new rmpl::PointRobot(minBoundary, maxBoundary));
    robot->set2DWorkspace(m_workspace);

    rmpl::SamplingMethod sampling = rmpl::SamplingMethod::randomly;
    if (m_samplingType == 1)
        sampling = rmpl::SamplingMethod::uniform;
    if (m_samplingType == 2)
        sampling = rmpl::SamplingMethod::standardDistribution;

    m_planner = nullptr;
    if (m_plannerType == 0) {
        std::shared_ptr<rmpl::RRTOptions> options(
            new rmpl::RRTOptions(m_rrtStepsize, m_trajectoryStepSize, rmpl::TrajectoryMethod::linear, sampling));
        m_planner = std::shared_ptr<rmpl::NormalRRTPlanner>(new rmpl::NormalRRTPlanner(robot, options));
    } else if (m_plannerType == 1) {
        std::shared_ptr<rmpl::RRTOptions> options(
            new rmpl::RRTOptions(m_rrtStepsize, m_trajectoryStepSize, rmpl::TrajectoryMethod::linear, sampling));
        m_planner = std::shared_ptr<rmpl::StarRRTPlanner>(new rmpl::StarRRTPlanner(robot, options));
    } else {
        std::shared_ptr<rmpl::PRMOptions> options(
            new rmpl::PRMOptions(m_rrtStepsize, m_trajectoryStepSize, rmpl::TrajectoryMethod::linear, sampling));
        m_planner = std::shared_ptr<rmpl::PRMPlanner>(new rmpl::PRMPlanner(robot, options));
    }

    rmpl::Vec<float> start(m_startX, m_startY);
    rmpl::Vec<float> goal(m_goalX, m_goalY);
    m_connected = m_planner->computePath(start, goal, m_numNodes, m_numThreads);
}

void MainWindow::viewPath() {
    std::cout << "view path" << std::endl;
    cv::Mat image = m_image.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    std::vector<std::shared_ptr<rmpl::Node>> nodes = m_planner->getGraphNodes();
    Drawing::drawTree2D(nodes, image, rmpl::Vec<uint8_t>(0, 0, 255), rmpl::Vec<uint8_t>(0, 0, 0), 1);
    if (m_connected) {
        std::vector<rmpl::Vec<float>> pathPoints = m_planner->getPath(0.5, true);
        Drawing::drawPath2D(pathPoints, image, rmpl::Vec<uint8_t>(255, 0, 0), 3);
    }

    QImage qImage = convertCvMat(image);

    QPixmap qPixmap = QPixmap::fromImage(qImage);
    scene = new QGraphicsScene(this);
    scene->addPixmap(qPixmap);
    scene->setSceneRect(qPixmap.rect());

    ui->mainImage->setScene(scene);

    // cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    // cv::imshow("pathPlanner", image);
    // cv::imwrite("result.png", image);
    // cv::waitKey(0);
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
