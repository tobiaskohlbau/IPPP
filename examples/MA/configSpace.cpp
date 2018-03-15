#include <chrono>

#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");
const unsigned int dim = 2;

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    Logging::setLogLevel(LogLevel::trace);

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(AABB(Vector3(0, 0, -1), Vector3(450, 450, 1)));
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/obstacle50x50.obj", util::Vecd(200, 55, 0, 0, 0, 0));
    envConfigurator.setRobotType(RobotType::Serial);
    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    Vector2 min(-util::pi(), -util::pi());
    Vector2 max(util::pi(), util::pi());
    envConfigurator.setRobotBaseProperties(dim, std::vector<DofType>({DofType::joint, DofType::joint}), std::make_pair(min, max));
    std::vector<DhParameter> dhParameters(dim, DhParameter(0, 100));
    std::vector<std::string> jointModelFiles(dim, FLAGS_assetsDir + "/robotModels/2dLine.obj");
    envConfigurator.setSerialRobotProperties(dhParameters, jointModelFiles);
    auto environment = envConfigurator.getEnvironment();
    environment->getRobot()->setPose(util::Vecd(225, 225, 0, 0, 0, util::toRad(-90)));

    double resolution = util::toRad(0.5);
    auto validityChecker = std::make_shared<CollisionFclSerial<dim>>(environment);
    auto sampler = std::make_shared<GridSampler<dim>>(environment, resolution);
    auto sampling = std::make_shared<StraightSampling<dim>>(environment, validityChecker, sampler);

    auto timer = std::make_shared<StatsTimeCollector>("Serial2D Planning Time");
    Stats::addCollector(timer);
    timer->start();
    std::vector<Vector2> samples;
    for (size_t i = 0; i < sampler->numSamples(); ++i) {
        auto sample = sampling->getSample();
        if (util::empty<dim>(sample))
            continue;
        samples.push_back(sample);
    }
    timer->stop();

    Vector2 start = util::toRad<dim>(Vector2(-60, 35));
    Vector2 goal = util::toRad<dim>(Vector2(50, -55));

    // config space
    cv::Mat configSpace(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    drawing::drawConfigs2D(configSpace, samples, Vector2i(400, 400), Vector3i(0, 0, 255), -1, 100);
    cv::namedWindow("configSpace");
    cv::imshow("configSpace", configSpace);
    cv::imwrite("configSpace.png", configSpace);

    // work space
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(environment->getRobot());
    auto workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    for (const auto& obstacle : environment->getObstacles())
        drawing::drawPolygons(image, obstacle->model->m_mesh, obstacle->getPose(), workspace2D.second, Vector3i(50, 50, 50));
    cv::Mat imageCopy = image.clone();
    drawing::drawSerialRobot2D<dim>(start, *serialRobot, imageCopy, workspace2D.second, Vector3i(0, 0, 255), 8);
    drawing::drawSerialRobot2D<dim>(goal, *serialRobot, imageCopy, workspace2D.second, Vector3i(0, 0, 255), 8);
    cv::namedWindow("pathPlanner");
    cv::imshow("pathPlanner", imageCopy);
    cv::imwrite("workspace.png", imageCopy);

    Stats::writeData(std::cout);
    cv::waitKey(0);
}
