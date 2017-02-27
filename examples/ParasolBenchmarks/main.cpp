#include <chrono>
#include <iostream>
#include <memory>

#include <core/utility/UtilVec.hpp>
#include <core/module/collisionDetection/CollisionDetectionPqp.hpp>
#include <core/utility/Logging.h>
#include <pathPlanner/NormalRRTPlanner.hpp>
#include <pathPlanner/PRMPlanner.hpp>
#include <pathPlanner/RRTStarPlanner.hpp>
#include <robot/model/ModelFactoryPqp.h>
#include <robot/MobileRobot.h>

#include <modelDirectory.h>

#include <ui/BenchmarkReader.h>
#include <ui/Drawing2D.hpp>
#include <ui/Writer.hpp>

using namespace rmpl;

bool computePath(std::string benchmarkDir, std::string queryPath, EnvironmentConfig config) {
    ModelFactoryPqp factoryPqp;
    std::shared_ptr<ModelContainer> robotModel = factoryPqp.createModel(benchmarkDir + config.robotFile);
    std::shared_ptr<ModelContainer> obstacleModel = factoryPqp.createModel(benchmarkDir + config.obstacleFile);
    std::static_pointer_cast<ModelPqp>(obstacleModel)->transform(config.obstacleConfig);
    std::vector<Vector6> queries = readQuery(queryPath);

    Vector6 minBoundary = utilVec::Vecf(config.minBoundary[0], config.minBoundary[1], config.minBoundary[2], 0, 0, 0);
    Vector6 maxBoundary = utilVec::Vecf(config.maxBoundary[0], config.maxBoundary[1], config.maxBoundary[2], 360, 360, 360);
    std::shared_ptr<RobotBase<6>> robot(new MobileRobot<6>(minBoundary, maxBoundary));
    robot->setWorkspace(obstacleModel);
    robot->setBaseModel(robotModel);
    std::shared_ptr<CollisionDetection<6>> collision(new CollisionDetectionPqp<6>(robot));

    RRTOptions<6> options(40, 3, collision);
    RRTStarPlanner<6> planner(robot, options);
    return planner.computePath(queries[0], queries[1], 8000, 6);
}

void benchmarkAlphaPuzzle() {
    std::string modelDir = getModelDirectory();
    std::string puzzleDir = modelDir + "parasol_benchmarks/alpha1.5/";
    std::string envPath = puzzleDir + "alpha.env";
    std::string queryPath = puzzleDir + "alpha.query";

    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(puzzleDir, queryPath, config);

    if (result)
        Logging::info("Path of AlphaPuzzle could be planned");
    else
        Logging::info("Path of AlphaPuzzle could NOT be planned");
}

void benchmarkFlange() {
    std::string modelDir = getModelDirectory();
    std::string flangeDir = modelDir + "parasol_benchmarks/flange_1.0/";
    std::string envPath = flangeDir + "flange.env";
    std::string queryPath = flangeDir + "flange.query";

    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(flangeDir, queryPath, config);

    if (result)
        Logging::info("Path of Flange could be planned");
    else
        Logging::info("Path of Flange could NOT be planned");
}

void benchmarkHedgehog() {
    std::string modelDir = getModelDirectory();
    std::string hedgehogDir = modelDir + "parasol_benchmarks/Hedgehog/";
    std::string envPath = hedgehogDir + "hedgehog.env";
    std::string queryPath = hedgehogDir + "hedgehog.query";

    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(hedgehogDir, queryPath, config);

    if (result)
        Logging::info("Path of Hedgehog could be planned");
    else
        Logging::info("Path of Hedgehog could NOT be planned");
}

int main(int argc, char** argv) {
    benchmarkFlange();
    benchmarkAlphaPuzzle();
    benchmarkHedgehog();

    return 0;
}
