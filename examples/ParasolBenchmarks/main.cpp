#include <chrono>
#include <iostream>
#include <memory>

#include <core/module/collisionDetection/CollisionDetectionPqpBenchmark.hpp>
#include <core/utility/UtilVec.hpp>
//#include <core/module/collisionDetection/CollisionDetectionFcl.hpp>
//#include <robot/model/ModelFactoryFcl.h>
#include <core/utility/Logging.h>
#include <pathPlanner/NormalRRTPlanner.hpp>
#include <pathPlanner/PRMPlanner.hpp>
#include <pathPlanner/RRTStarPlanner.hpp>

#include <robot/MobileRobot.h>
#include <robot/model/ModelFactoryPqp.h>

#include <modelDirectory.h>

#include <ui/BenchmarkReader.h>
#include <ui/Drawing2D.hpp>
#include <ui/Writer.hpp>

using namespace rmpl;

std::string modelDir;

bool computePath(std::string benchmarkDir, std::string queryPath, EnvironmentConfig config) {
    // ModelFactoryFcl factoryFcl;
    // std::shared_ptr<ModelContainer> robotModel = factoryFcl.createModel(benchmarkDir + config.robotFile);
    // std::shared_ptr<ModelContainer> obstacleModel = factoryFcl.createModel(benchmarkDir + config.obstacleFile);
    // std::static_pointer_cast<ModelFcl>(obstacleModel)->transform(config.obstacleConfig);
    ModelFactoryPqp factoryPqp;
    std::shared_ptr<ModelContainer> robotModel = factoryPqp.createModel(benchmarkDir + config.robotFile);
    std::shared_ptr<ModelContainer> obstacleModel = factoryPqp.createModel(benchmarkDir + config.obstacleFile);
    std::static_pointer_cast<ModelPqp>(obstacleModel)->transform(config.obstacleConfig);
    // save models as obj
    exportCad(ExportFormat::OBJ, "robot", robotModel->m_vertices, robotModel->m_faces);
    exportCad(ExportFormat::OBJ, "obstacle", obstacleModel->m_vertices, obstacleModel->m_faces);

    std::vector<Vector6> queries = readQuery(queryPath);

    Vector6 minBoundary = util::Vecf(config.minBoundary[0], config.minBoundary[1], config.minBoundary[2], 0, 0, 0);
    Vector6 maxBoundary = util::Vecf(config.maxBoundary[0], config.maxBoundary[1], config.maxBoundary[2], util::twoPi(),
                                        util::twoPi(), util::twoPi());
    std::shared_ptr<RobotBase<6>> robot(new MobileRobot<6>(minBoundary, maxBoundary));
    robot->setWorkspace(obstacleModel);
    robot->setBaseModel(robotModel);

    std::shared_ptr<CollisionDetection<6>> collision(new CollisionDetectionPqp<6>(robot));
    std::shared_ptr<CollisionDetection<6>> collisionBenchmark(new CollisionDetectionPqpBenchmark<6>(robot));
    std::shared_ptr<TrajectoryPlanner<6>> trajectory(new TrajectoryPlanner<6>(3, collision));
    std::shared_ptr<TrajectoryPlanner<6>> trajectoryBenchmark(new TrajectoryPlanner<6>(3, collisionBenchmark));
    std::shared_ptr<Sampler<6>> sampler(new Sampler<6>(robot));
    std::shared_ptr<Sampling<6>> sampling(new Sampling<6>(robot, collision, trajectory, sampler));
    for (int i = 3; i < 6; ++i) {
        config.obstacleConfig[i] *= util::toRad();
    }

    std::shared_ptr<Sampling<6>> samplingBenchmark(new Sampling<6>(robot, collisionBenchmark, trajectoryBenchmark, sampler));

    RRTOptions<6> options(40, collision, trajectory, sampling);
    RRTOptions<6> optionsBenchmark(40, collisionBenchmark, trajectoryBenchmark, samplingBenchmark);
    RRTStarPlanner<6> planner(robot, options);
    RRTStarPlanner<6> plannerBenchmark(robot, optionsBenchmark);

    //std::static_pointer_cast<ModelPqp>(robotModel)->transform(queries[1]);
    //exportCad(ExportFormat::OBJ, "robotStart", robotModel->m_vertices, robotModel->m_faces);

    auto startTime = std::chrono::system_clock::now();
    bool result = planner.computePath(queries[0], queries[1], 8000, 18);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);

    if (result) {
        plannerBenchmark.computePath(queries[0], queries[1], 8000, 18);
    }

    std::cout << "Computation time: " << duration.count() / 1000.0 << std::endl;
    std::shared_ptr<CollisionDetectionPqpBenchmark<6>> collisionDetectionPqpBenchmark =
        std::static_pointer_cast<CollisionDetectionPqpBenchmark<6>>(collisionBenchmark);
    std::cout << "Collision count: " << collisionDetectionPqpBenchmark->getCount() << std::endl;
    std::cout << "Mean collision computation time: " << collisionDetectionPqpBenchmark->getMeanComputationTime().count()
              << " nano seconds" << std::endl;
    return result;
}

void benchmarkAlphaPuzzle() {
    std::string puzzleDir = modelDir + "parasol_benchmarks/alpha1.5/";
    std::string envPath = puzzleDir + "alpha.env";
    std::string queryPath = puzzleDir + "alpha.query";

    Logging::info("AlphaPuzzle:");
    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(puzzleDir, queryPath, config);

    if (result)
        Logging::info("could be planned");
    else
        Logging::warning("could NOT be planned");
    std::cout << std::endl;
}

void benchmarkFlange() {
    std::string flangeDir = modelDir + "parasol_benchmarks/flange_1.0/";
    std::string envPath = flangeDir + "flange.env";
    std::string queryPath = flangeDir + "flange.query";

    Logging::info("Flange:");
    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(flangeDir, queryPath, config);

    if (result)
        Logging::info("could be planned");
    else
        Logging::warning("could NOT be planned");
    std::cout << std::endl;
}

void benchmarkHedgehog() {
    std::string hedgehogDir = modelDir + "parasol_benchmarks/Hedgehog/";
    std::string envPath = hedgehogDir + "hedgehog.env";
    std::string queryPath = hedgehogDir + "hedgehog.query";

    Logging::info("Hedgehog:");
    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(hedgehogDir, queryPath, config);

    if (result)
        Logging::info("could be planned");
    else
        Logging::warning("could NOT be planned");
    std::cout << std::endl;
}

int main(int argc, char** argv) {
    modelDir = getModelDirectory();
    benchmarkFlange();
    benchmarkAlphaPuzzle();
    benchmarkHedgehog();

    return 0;
}
