#include <thread>

#include <ConfigurationMA.h>
#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");

const unsigned int dim = 2;
AABB workspace(Vector3(0, 0, 0), Vector3(2000, 2000, 2000));
Vector2 start(300, 300);
Vector2 goal(1500, 1000);

ConfigurationMA m_configurationMA(RobotType::Point2D, 2, true, false);
std::vector<ParamsMA> m_paramsMA;

void drawImage(std::shared_ptr<Planner<2>> planner, std::shared_ptr<Environment> env, size_t index) {
    auto workspace2D = cad::create2dspace(workspace, 255);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    for (const auto& obstacle : env->getObstacles())
        drawing::drawPolygons(image, obstacle->model->m_mesh, obstacle->getPose(), workspace2D.second, Vector3i(50, 50, 50));

    auto nodes = planner->getGraphNodes();
    drawing::drawGraph2D(image, nodes, Vector3i(125, 125, 200), Vector3i(125, 125, 200), 1);

    auto path = planner->getPath();
    if (!path.empty())
        drawing::drawPath2D(image, path, Vector3i(255, 0, 0), 2);

    // cv::namedWindow("Planner", CV_WINDOW_AUTOSIZE);
    // cv::imshow("Planner", image);
    // cv::waitKey(0);

    cv::imwrite("images/" + std::to_string(index) + ".png", image);
}

std::shared_ptr<Environment> createEnvironment(ParamsMA params) {
    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(workspace);
    envConfigurator.setRobotType(RobotType::Point2D);

    if (params.useObstacle) {
        Vector2 min = Vector2(workspace.min()[0], workspace.min()[1]);
        Vector2 max = Vector2(workspace.max()[0], workspace.max()[1]);

        cad::MapGenerator<dim> mapGenerator(workspace, std::make_shared<SamplerRandom<2>>(std::make_pair(min, max), params.seed));
        auto meshes = mapGenerator.generateMap(200, Vector2(10, 10), Vector2(80, 80));
        for (auto& mesh : meshes)
            envConfigurator.addObstacle(mesh);
    }

    return envConfigurator.getEnvironment();
}

ModuleConfigurator<2> getCreator(std::shared_ptr<Environment> env, ParamsMA params) {
    ModuleConfigurator<dim> creator;
    creator.setEnvironment(env);
    creator.setPathModifierType(PathModifierType::NodeCut);
    creator.setValidityCheckerType(ValidityCheckerType::Dim2);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(EvaluatorType::TreeConfigOrTime);
    creator.setEvaluatorProperties(params.stepSize, 45);
    creator.setSamplerType(params.samplerType);
    creator.setSamplingType(params.samplingType);
    creator.setSamplerProperties(params.seed, 1);
    creator.setSamplingProperties(10, 80);
    return creator;
}

void planningThread(size_t startIndex, size_t endIndex) {
    for (auto params = m_paramsMA.begin() + startIndex; params < m_paramsMA.begin() + endIndex; ++params) {
        Stats::initializeCollectors();
        m_configurationMA.updatePropertyStats(params - m_paramsMA.begin());
        auto env = createEnvironment(*params);
        auto creator = getCreator(env, *params);

        auto planner =
            std::make_shared<RRTStar<2>>(creator.getEnvironment(), creator.getRRTOptions(params->stepSize), creator.getGraph());

        planner->computePath(start, goal, 3000, 1);
        if (params->optimize)
            planner->optimize(1000, 1);

        drawImage(planner, env, params - m_paramsMA.begin());
        ui::save("data/" + std::to_string(params - m_paramsMA.begin()) + ".json", Stats::serialize());
    }
}

void testMobile() {
    m_configurationMA = ConfigurationMA(RobotType::Point2D, 2, true, false);
    std::cout << m_configurationMA.numParams() << std::endl;
    m_paramsMA = m_configurationMA.getParamsList();
    std::vector<std::thread> threads;

    size_t nbOfThreads = 1;
    size_t threadAmount = m_configurationMA.numParams() / nbOfThreads;

    auto startTime = std::chrono::system_clock::now();
    for (size_t i = 0; i < nbOfThreads; ++i)
        threads.push_back(std::thread(&planningThread, i * threadAmount, (i + 1) * threadAmount));

    for (size_t i = 0; i < nbOfThreads; ++i)
        threads[i].join();
    std::chrono::duration<double> duration = std::chrono::system_clock::now() - startTime;
    std::cout << std::endl << "Evaluation time: " << duration.count() << std::endl;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::warn);

    testMobile();
}
