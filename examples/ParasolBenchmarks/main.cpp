#include <chrono>
#include <iostream>
#include <memory>
#include <string>

//#include <gperftools/profiler.h>

//#include <boost/filesystem.hpp>
//#include <boost/range/iterator_range.hpp>

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/core/collisionDetection/CollisionDetectionPqpBenchmark.hpp>
//#include <ippp/core/collisionDetection/CollisionDetectionFcl.hpp>
//#include <ippp/environment/model/ModelFactoryFcl.h>

#include <modelDirectory.h>
#include <ui/BenchmarkReader.h>
#include <ui/ModuleConfigurator.hpp>
#include <ui/Writer.hpp>

using namespace ippp;

std::string modelDir;

bool computePath(std::string benchmarkDir, std::string queryPath, EnvironmentConfig config) {
    const unsigned int dim = 6;
    // ModelFactoryFcl factoryFcl;
    // std::shared_ptr<ModelContainer> robotModel = factoryFcl.createModel(benchmarkDir + config.robotFile);
    // std::shared_ptr<ModelContainer> obstacleModel = factoryFcl.createModel(benchmarkDir + config.obstacleFile);
    // std::static_pointer_cast<ModelFcl>(obstacleModel)->transform(config.obstacleConfig);
    ModelFactoryPqp factoryPqp;
    std::shared_ptr<ModelContainer> robotModel = factoryPqp.createModel(benchmarkDir + config.robotFile);
    std::shared_ptr<ModelContainer> obstacleModel = factoryPqp.createModel(benchmarkDir + config.obstacleFile);
    obstacleModel->transformModel(config.obstacleConfig);
    // save models as obj
    // exportCad(ExportFormat::OBJ, "robot", robotModel->m_vertices, robotModel->m_faces);
    // exportCad(ExportFormat::OBJ, "obstacle", obstacleModel->m_vertices, obstacleModel->m_faces);

    std::vector<Vector6> queries = readQuery(queryPath);

    Vector6 minBoundary = util::Vecd(config.minBoundary[0], config.minBoundary[1], config.minBoundary[2], 0, 0, 0);
    Vector6 maxBoundary = util::Vecd(config.maxBoundary[0], config.maxBoundary[1], config.maxBoundary[2], util::twoPi(),
                                     util::twoPi(), util::twoPi());
    std::vector<DofType> dofTypes = {DofType::volumetricPos, DofType::volumetricPos, DofType::volumetricPos,
                          DofType::volumetricRot, DofType::volumetricRot, DofType::volumetricRot};
    std::shared_ptr<RobotBase> robot(new MobileRobot(6, std::make_pair(minBoundary, maxBoundary), dofTypes));
    robot->setBaseModel(robotModel);

    std::shared_ptr<Environment> environment(new Environment(3, AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    environment->addObstacle(obstacleModel);

    std::shared_ptr<CollisionDetection<6>> collision(new CollisionDetectionPqp<6>(environment));
    std::shared_ptr<CollisionDetection<6>> collisionBenchmark(new CollisionDetectionPqpBenchmark<6>(environment));

    ModuleConfigurator<dim> creator;
    creator.setCollisionType(CollisionType::PQP);
    creator.setEnvironment(environment);
    ModuleConfigurator<dim> creatorBenchmark;
    creatorBenchmark.setCollisionType(CollisionType::PQP);
    creatorBenchmark.setEnvironment(environment);

    for (int i = 3; i < 6; ++i) {
        config.obstacleConfig[i] *= util::toRad();
    }

    RRTStar<6> planner(environment, creator.getRRTOptions(40), creator.getGraph());
    RRTStar<6> plannerBenchmark(environment, creatorBenchmark.getRRTOptions(40), creatorBenchmark.getGraph());

    // std::static_pointer_cast<ModelPqp>(robotModel)->transform(queries[1]);
    // exportCad(ExportFormat::OBJ, "robotStart", robotModel->m_vertices, robotModel->m_faces);

    auto startTime = std::chrono::system_clock::now();
    bool result = planner.computePath(queries[0], queries[1], 8000, 10);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);

    if (result) {
        plannerBenchmark.computePath(queries[0], queries[1], 8000, 10);
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

    std::cout << "AlphaPuzzle:" << std::endl;
    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(puzzleDir, queryPath, config);

    if (result)
        std::cout << "could be planned" << std::endl;
    else
        std::cout << "could NOT be planned" << std::endl;
    std::cout << std::endl;
}

void benchmarkFlange() {
    std::string flangeDir = modelDir + "parasol_benchmarks/flange_1.0/";
    std::string envPath = flangeDir + "flange.env";
    std::string queryPath = flangeDir + "flange.query";

    std::cout << "Flange:" << std::endl;
    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(flangeDir, queryPath, config);

    if (result)
        std::cout << "could be planned" << std::endl;
    else
        std::cout << "could NOT be planned" << std::endl;
    std::cout << std::endl;
}

void benchmarkHedgehog() {
    std::string hedgehogDir = modelDir + "parasol_benchmarks/Hedgehog/";
    std::string envPath = hedgehogDir + "hedgehog.env";
    std::string queryPath = hedgehogDir + "hedgehog.query";

    std::cout << "Hedgehog:" << std::endl;
    EnvironmentConfig config = readEnvironment(envPath);
    bool result = computePath(hedgehogDir, queryPath, config);

    if (result)
        std::cout << "could be planned" << std::endl;
    else
        std::cout << "could NOT be planned" << std::endl;
    std::cout << std::endl;
}

void generateMap() {
    const unsigned int dim = 2;
    Vector2 min(0,0);
    Vector2 max(1000,1000);
    std::shared_ptr<Sampler<dim>> sampler(new SamplerRandom<dim>(min, max));

    MapGenerator<dim> mapGenerator(min, max, sampler);
    auto meshes = mapGenerator.generateMap(400, Vector2(50, 50));
    auto mesh = cad::mergeMeshes(meshes);
    cad::exportCad(cad::ExportFormat::OBJ, "obstacle", mesh);

    for (size_t i = 0; i < meshes.size(); ++i)
        cad::exportCad(cad::ExportFormat::OBJ, "obstacle" + std::to_string(i), meshes[i]);
}

int main(int argc, char** argv) {
    modelDir = getModelDirectory();

//    std::string file = modelDir + "assembly/boxRot.dae";
//    std::vector<Mesh> meshes;
//    auto result = cad::importMeshes(file, meshes, 1, false, true);
//    if (result)
//        std::cout << "successful reading" << std::endl;
//    for (int i = 0; i < meshes.size(); ++i)
//        cad::exportCad(cad::ExportFormat::OBJ, std::to_string(i), meshes[i]);

    generateMap();

    // ProfilerStart("/tmp/cpu.prof");
    //    benchmarkFlange();
    //    benchmarkAlphaPuzzle();
    //    benchmarkHe dgehog();
    // ProfilerStop();
    return 0;
}