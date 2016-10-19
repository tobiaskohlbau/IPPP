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
    rmpl::Vec<float> minBoundary(0.0, 0.0);
    rmpl::Vec<float> maxBoundary(m_workspace.rows(), m_workspace.cols());
    std::shared_ptr<rmpl::PointRobot> robot(new rmpl::PointRobot(minBoundary, maxBoundary));
    robot->set2DWorkspace(m_workspace);

    rmpl::SamplingMethod sampling = rmpl::SamplingMethod::randomly;
    if (m_samplingType == 1)
        sampling = rmpl::SamplingMethod::uniform;
    if (m_samplingType == 1)
        sampling = rmpl::SamplingMethod::standardDistribution;

    if (m_plannerType == 0) {
        std::shared_ptr<rmpl::RRTOptions> options(
            new rmpl::RRTOptions(m_rrtStepsize, m_trajectoryStepSize, rmpl::TrajectoryMethod::linear, sampling));
        m_planner = new rmpl::NormalRRTPlanner(robot, options);
    } else if (m_plannerType == 1) {
        std::shared_ptr<rmpl::RRTOptions> options(
            new rmpl::RRTOptions(m_rrtStepsize, m_trajectoryStepSize, rmpl::TrajectoryMethod::linear, sampling));
        m_planner = new rmpl::StarRRTPlanner(robot, options);
    } else {
        std::shared_ptr<rmpl::PRMOptions> options(
                new rmpl::PRMOptions(m_rrtStepsize, m_trajectoryStepSize, rmpl::TrajectoryMethod::linear, sampling));
        m_planner = new rmpl::PRMPlanner(robot, options);
    }

    rmpl::Vec<float> start(m_startX, m_startY);
    rmpl::Vec<float> goal(m_goalX, m_goalY);
    m_connected = m_planner->computePath(start, goal, m_numNodes, m_numThreads);
}

void MainWindow::viewPath() {
    cv::Mat image = m_image.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    std::vector<std::shared_ptr<rmpl::Node>> nodes = m_planner->getGraphNodes();
    Drawing::drawTree2D(nodes, image, rmpl::Vec<uint8_t>(0, 0, 255), rmpl::Vec<uint8_t>(0, 0, 0), 1);
    if (m_connected) {
        std::vector<rmpl::Vec<float>> pathPoints = m_planner->getPath(0.5, true);
        Drawing::drawPath2D(pathPoints, image, rmpl::Vec<uint8_t>(255, 0, 0), 3);
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::imwrite("result.png", image);
    cv::waitKey(0);
}
