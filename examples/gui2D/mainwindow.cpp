#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <core/module/collisionDetection/CollisionDetection2D.hpp>
#include <core/module/collisionDetection/CollisionDetectionTriangleRobot.hpp>
#include <core/module/sampler/SamplerNormalDist.hpp>
#include <core/module/sampler/SamplerUniform.hpp>
#include <core/module/sampling/SamplingNearObstacle.hpp>
#include <core/utility/heuristic/HeuristicInf.hpp>
#include <core/utility/heuristic/HeuristicL1.hpp>
#include <core/utility/heuristic/HeuristicWeightVecInf.hpp>
#include <core/utility/heuristic/HeuristicWeightVecL1.hpp>
#include <core/utility/heuristic/HeuristicWeightVecL2.hpp>
#include <pathPlanner/options/PRMOptions.hpp>
#include <robot/model/ModelFactoryTriangle2D.h>
#include <ui/Drawing2D.hpp>

using namespace rmpl;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), Identifier("GUI"), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    m_widget = new QWidget();
    m_widget->setWindowTitle(QString("Path"));
    m_layout = new QVBoxLayout();
    m_widget->setLayout(m_layout);

    m_triangles.push_back(Triangle2D(Vector2(0, 0), Vector2(25, 0), Vector2(25, 50)));
    m_triangles.push_back(Triangle2D(Vector2(0, 0), Vector2(0, 50), Vector2(25, 50)));
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::loadImage() {
    QString qFileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home", tr("Image Files (*.png *.jpg *.bmp)"));

    cv::Mat image = cv::imread(qFileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
    if (!image.data)    // Check for invalid input
    {
        Logging::error("could not load image", this);
        return;
    }

    m_workspace = drawing::cvToEigen(image);
    m_image = image;
    m_imageLoaded = true;
    Logging::info("loading image was successful", this);
}

void MainWindow::computePath() {
    if (!m_imageLoaded) {
        Logging::error("Image was not loaded", this);
        return;
    }
    Logging::info("Compute path ...", this);

    if (m_robotType == 1) {
        const unsigned int dim = 3;
        std::shared_ptr<rmpl::Heuristic<dim>> edgeH = std::make_shared<rmpl::Heuristic<dim>>(rmpl::Heuristic<dim>());
        ;
        if (m_edgeHeuristic == 1) {
            edgeH = std::make_shared<rmpl::HeuristicL1<dim>>(rmpl::HeuristicL1<dim>());
        } else if (m_edgeHeuristic == 2) {
            edgeH = std::make_shared<rmpl::HeuristicInf<dim>>(rmpl::HeuristicInf<dim>());
        } else if (m_edgeHeuristic == 3) {
            std::shared_ptr<HeuristicWeightVecL2<dim>> heuristic(new rmpl::HeuristicWeightVecL2<dim>());
            heuristic->setWeightVec(m_weightVec);
            edgeH = heuristic;
        } else if (m_edgeHeuristic == 4) {
            std::shared_ptr<HeuristicWeightVecL1<dim>> heuristic(new rmpl::HeuristicWeightVecL1<dim>());
            heuristic->setWeightVec(m_weightVec);
            edgeH = heuristic;
        } else if (m_edgeHeuristic == 5) {
            std::shared_ptr<HeuristicWeightVecInf<dim>> heuristic(new rmpl::HeuristicWeightVecInf<dim>());
            heuristic->setWeightVec(m_weightVec);
            edgeH = heuristic;
        }

        Vector3 minBoundary(0.0, 0.0, 0.0);
        Vector3 maxBoundary(m_workspace.cols(), m_workspace.rows(), util::twoPi());
        ModelFactoryTriangle2D factory;
        std::shared_ptr<ModelContainer> baseModel = factory.createModel(m_triangles);
        std::shared_ptr<TriangleRobot2D> robot(new TriangleRobot2D(baseModel, minBoundary, maxBoundary));
        std::shared_ptr<ModelContainer> model(new Model2D(m_workspace));
        robot->setWorkspace(model);
        std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetectionTriangleRobot(robot));
        std::shared_ptr<TrajectoryPlanner<dim>> trajectory(new TrajectoryPlanner<dim>(collision, m_trajectoryStepSize));

        std::shared_ptr<Sampler<dim>> sampler(new Sampler<dim>(robot));
        if (m_samplingStrategy == 1)
            sampler = std::shared_ptr<Sampler<dim>>(new SamplerUniform<dim>(robot));
        else if (m_samplerMethod == 2)
            sampler = std::shared_ptr<Sampler<dim>>(new SamplerNormalDist<dim>(robot));

        std::shared_ptr<Sampling<dim>> sampling(new Sampling<dim>(robot, collision, trajectory, sampler));
        if (m_samplingStrategy == 1)
            sampling = std::shared_ptr<Sampling<dim>>(new SamplingNearObstacle<3>(robot, collision, trajectory, sampler));

        RRTOptions<dim> rrtOptions(m_rrtStepsize, collision, trajectory, sampling, edgeH);
        PRMOptions<dim> prmOptions(m_prmDistance, collision, trajectory, sampling, edgeH);

        if (m_plannerType == 0)
            m_planner3d = std::shared_ptr<NormalRRTPlanner<dim>>(new NormalRRTPlanner<3>(robot, rrtOptions));
        else if (m_plannerType == 1)
            m_planner3d = std::shared_ptr<RRTStarPlanner<dim>>(new RRTStarPlanner<3>(robot, rrtOptions));
        else
            m_planner3d = std::shared_ptr<PRMPlanner<dim>>(new PRMPlanner<3>(robot, prmOptions));
        Vector3 start(m_startX, m_startY, m_startPhi * util::toRad());
        Vector3 goal(m_goalX, m_goalY, m_goalPhi * util::toRad());
        m_connected = m_planner3d->computePath(start, goal, m_numNodes, m_numThreads);
    } else {
        const unsigned int dim = 2;
        std::shared_ptr<rmpl::Heuristic<dim>> edgeH = std::make_shared<rmpl::Heuristic<dim>>(rmpl::Heuristic<dim>());
        if (m_edgeHeuristic == 1) {
            edgeH = std::make_shared<rmpl::HeuristicL1<dim>>(rmpl::HeuristicL1<dim>());
        } else if (m_edgeHeuristic == 2) {
            edgeH = std::make_shared<rmpl::HeuristicInf<dim>>(rmpl::HeuristicInf<dim>());
        } else if (m_edgeHeuristic == 3) {
            std::shared_ptr<HeuristicWeightVecL2<dim>> heuristic(new rmpl::HeuristicWeightVecL2<dim>());
            heuristic->setWeightVec(Vector2(m_weightVecX, m_weightVecY));
            edgeH = heuristic;
        } else if (m_edgeHeuristic == 4) {
            std::shared_ptr<HeuristicWeightVecL1<dim>> heuristic(new rmpl::HeuristicWeightVecL1<dim>());
            heuristic->setWeightVec(Vector2(m_weightVecX, m_weightVecY));
            edgeH = heuristic;
        } else if (m_edgeHeuristic == 5) {
            std::shared_ptr<HeuristicWeightVecInf<2>> heuristic(new rmpl::HeuristicWeightVecInf<2>());
            heuristic->setWeightVec(Vector2(m_weightVecX, m_weightVecY));
            edgeH = heuristic;
        }

        Vector2 minBoundary(0.0, 0.0);
        Vector2 maxBoundary(m_workspace.cols(), m_workspace.rows());
        std::shared_ptr<PointRobot> robot(new PointRobot(minBoundary, maxBoundary));
        std::shared_ptr<ModelContainer> model(new Model2D(m_workspace));
        robot->setWorkspace(model);
        std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetection2D(robot));
        std::shared_ptr<TrajectoryPlanner<dim>> trajectory(new TrajectoryPlanner<dim>(collision, m_trajectoryStepSize));
        std::shared_ptr<Sampler<dim>> sampler(new Sampler<dim>(robot));

        if (m_samplingStrategy == 1)
            sampler = std::shared_ptr<Sampler<dim>>(new SamplerUniform<dim>(robot));
        else if (m_samplerMethod == 2)
            sampler = std::shared_ptr<Sampler<dim>>(new SamplerNormalDist<dim>(robot));

        std::shared_ptr<Sampling<dim>> sampling(new Sampling<dim>(robot, collision, trajectory, sampler));
        if (m_samplingStrategy == 1)
            sampling = std::shared_ptr<Sampling<dim>>(new SamplingNearObstacle<dim>(robot, collision, trajectory, sampler));

        RRTOptions<dim> rrtOptions(m_rrtStepsize, collision, trajectory, sampling);
        PRMOptions<dim> prmOptions(m_prmDistance, collision, trajectory, sampling);

        if (m_plannerType == 0)
            m_planner2d = std::shared_ptr<NormalRRTPlanner<dim>>(new NormalRRTPlanner<dim>(robot, rrtOptions));
        else if (m_plannerType == 1)
            m_planner2d = std::shared_ptr<RRTStarPlanner<dim>>(new RRTStarPlanner<dim>(robot, rrtOptions));
        else
            m_planner2d = std::shared_ptr<PRMPlanner<2>>(new PRMPlanner<dim>(robot, prmOptions));
        Vector2 start(m_startX, m_startY);
        Vector2 goal(m_goalX, m_goalY);
        m_connected = m_planner2d->computePath(start, goal, m_numNodes, m_numThreads);
    }
    Logging::info("... done", this);
    viewPath();
}

void MainWindow::viewPath() {
    cv::Mat image = m_image.clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);

    if (m_robotType == 1) {
        if (m_connected) {
            std::vector<Vector3> path = m_planner3d->getPath(80, true);
            drawing::drawTrianglePath(path, m_triangles, image, Eigen::Vector3i(0, 0, 255), 2);
        } else {
            return;
        }
    } else {
        std::vector<std::shared_ptr<Node<2>>> nodes = m_planner2d->getGraphNodes();
        drawing::drawTree2D<2>(nodes, image, Eigen::Vector3i(0, 0, 255), Eigen::Vector3i(0, 0, 0), 1);
        if (m_connected) {
            std::vector<Vector2> pathPoints = m_planner2d->getPath(1, true);
            drawing::drawPath2D(pathPoints, image, Eigen::Vector3i(255, 0, 0), 3);
        }
    }

    QImage qImage = convertCvMat(image);
    QPixmap qPixmap = QPixmap::fromImage(qImage);
    QGraphicsScene* scene = new QGraphicsScene();
    scene->addPixmap(qPixmap);
    scene->setSceneRect(qPixmap.rect());

    QLayoutItem* item = m_widget->layout()->takeAt(0);
    if (item != NULL)
        delete item->widget();

    QGraphicsView* view = new QGraphicsView(scene);
    m_layout->addWidget(view);
    m_widget->show();
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

void MainWindow::loadConfig() {
    QString qFileName = QFileDialog::getOpenFileName(this, tr("Open Config"), "/home", tr("Config Files (*.js *.json)"));
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(qFileName.toStdString(), pt);

    updateThreads(pt.get<int>("numThreads"));
    updateNodes(pt.get<int>("numNodes"));
    updatePlannerType(pt.get<int>("plannerType"));
    updateRobotType(pt.get<int>("robotType"));
    updateStartX(pt.get<int>("startX"));
    updateStartY(pt.get<int>("startY"));
    updateStartPhi(pt.get<double>("startPhi"));
    updateGoalX(pt.get<int>("goalX"));
    updateGoalY(pt.get<int>("goalY"));
    updateGoalPhi(pt.get<double>("goalPhi"));
    updateTrajectoryStepSize(pt.get<double>("trajectoryStepSize"));
    updateSamplerType(pt.get<int>("samplerMethod"));
    updateSamplingType(pt.get<int>("samplingStrategy"));
    updateEdgeHeuristic(pt.get<int>("edgeHeuristic"));
    updatePRMDistance(pt.get<double>("prmDistance"));
    updateRRTStepSize(pt.get<double>("rrtStepSize"));
    updateWeightVecX(pt.get<double>("weightVecX"));
    updateWeightVecZ(pt.get<double>("weightVecZ"));

    setNumThreads(m_numThreads);
    setNumNodes(m_numNodes);
    setPlannerType(m_plannerType);
    setRobotType(m_robotType);
    setTrajectoryStepSize(m_trajectoryStepSize);
    setSamplerMethod(m_samplerMethod);
    setSamplingStrategy(m_samplingStrategy);
    setPrmDistance(m_prmDistance);
    setRrtStepSize(m_rrtStepsize);
    setStartX(m_startX);
    setStartY(m_startY);
    setStartPhi(m_startPhi);
    setGoalX(m_goalX);
    setGoalY(m_goalY);
    setGoalPhi(m_goalPhi);
    setEdgeHeuristic(m_edgeHeuristic);
    setWeightVecX(m_weightVecX);
    setWeightVecY(m_weightVecY);
    setWeightVecZ(m_weightVecZ);
}

void MainWindow::saveConfig() {
    boost::property_tree::ptree pt;

    pt.put("numThreads", m_numThreads);
    pt.put("numNodes", m_numNodes);
    pt.put("plannerType", m_plannerType);
    pt.put("robotType", m_robotType);
    pt.put("trajectoryStepSize", m_trajectoryStepSize);
    pt.put("samplerMethod", m_samplerMethod);
    pt.put("samplingStrategy", m_samplingStrategy);
    pt.put("prmDistance", m_prmDistance);
    pt.put("rrtStepSize", m_rrtStepsize);
    pt.put("startX", m_startX);
    pt.put("startY", m_startY);
    pt.put("startPhi", m_startPhi);
    pt.put("goalX", m_goalX);
    pt.put("goalY", m_goalY);
    pt.put("goalPhi", m_goalPhi);
    pt.put("edgeHeuristic", m_edgeHeuristic);
    pt.put("weightVecX", m_weightVecX);
    pt.put("weightVecY", m_weightVecY);
    pt.put("weightVecZ", m_weightVecZ);

    QString qFileName = QFileDialog::getSaveFileName(this, tr("Save Config"), "/home", tr("Config Files (*.js *.json)"));
    boost::property_tree::write_json(qFileName.toStdString(), pt);
}

void MainWindow::updateThreads(int num) {
    m_numThreads = num;
}
void MainWindow::updateNodes(int num) {
    m_numNodes = num;
}
void MainWindow::updatePlannerType(int type) {
    m_plannerType = type;
}
void MainWindow::updateRobotType(int type) {
    m_robotType = type;
}
void MainWindow::updateTrajectoryStepSize(double value) {
    m_trajectoryStepSize = value;
}
void MainWindow::updateSamplingType(int type) {
    m_samplingStrategy = type;
}
void MainWindow::updateSamplerType(int type) {
    m_samplerMethod = type;
}
void MainWindow::updatePRMDistance(double value) {
    m_prmDistance = value;
}
void MainWindow::updateRRTStepSize(double value) {
    m_rrtStepsize = value;
}
void MainWindow::updateStartX(int value) {
    m_startX = value;
}
void MainWindow::updateStartY(int value) {
    m_startY = value;
}
void MainWindow::updateStartPhi(double value) {
    m_startPhi = value;
}
void MainWindow::updateGoalX(int value) {
    m_goalX = value;
}
void MainWindow::updateGoalY(int value) {
    m_goalY = value;
}
void MainWindow::updateGoalPhi(double value) {
    m_goalPhi = value;
}
void MainWindow::updateEdgeHeuristic(int type) {
    m_edgeHeuristic = type;
}
void MainWindow::updateWeightVecX(double value) {
    m_weightVecX = value;
    m_weightVec = Vector3(m_weightVecX, m_weightVecY, m_weightVecZ);
}
void MainWindow::updateWeightVecY(double value) {
    m_weightVecY = value;
    m_weightVec = Vector3(m_weightVecX, m_weightVecY, m_weightVecZ);
}
void MainWindow::updateWeightVecZ(double value) {
    m_weightVecY = value;
    m_weightVec = Vector3(m_weightVecX, m_weightVecY, m_weightVecZ);
}
