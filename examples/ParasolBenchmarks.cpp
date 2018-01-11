#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

#include <BenchmarkReader.h>

using namespace ippp;

bool computePath(const std::string& benchmarkDir, const std::string& queryPath, EnvironmentConfig config) {
    const unsigned int dim = 6;
    // ModelFactoryFcl factoryFcl;
    // std::shared_ptr<ModelContainer> robotModel = factoryFcl.createModel(benchmarkDir + config.robotFile);
    // std::shared_ptr<ModelContainer> obstacleModel = factoryFcl.createModel(benchmarkDir + config.obstacleFile);
    // std::static_pointer_cast<ModelFcl>(obstacleModel)->transform(config.obstacleConfig);
    ModelFactoryPqp factoryPqp;
    std::shared_ptr<ModelContainer> robotModel = factoryPqp.createModelFromFile(benchmarkDir + config.robotFile);
    std::shared_ptr<ModelContainer> obstacleModel = factoryPqp.createModelFromFile(benchmarkDir + config.obstacleFile);
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

    std::shared_ptr<Environment> environment(new Environment(AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    environment->addEnvObject(std::make_shared<ObstacleObject>("obstacle", obstacleModel));

    std::shared_ptr<CollisionDetection<6>> collision(new CollisionDetectionPqp<6>(environment));

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
    return result;
}

void benchmarkAlphaPuzzle() {
    std::string puzzleDir = "assets/parasol_benchmarks/alpha1.5/";
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
    std::string flangeDir = "assets/parasol_benchmarks/flange_1.0/";
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
    std::string hedgehogDir = "assets/parasol_benchmarks/Hedgehog/";
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
    Vector2 min(0, 0);
    Vector2 max(1000, 1000);
    std::shared_ptr<Sampler<dim>> sampler(new SamplerRandom<dim>(min, max));

    MapGenerator<dim> mapGenerator(min, max, sampler);
    auto meshes = mapGenerator.generateMap(400, Vector2(50, 50), Vector2(10, 10));
    auto mesh = cad::mergeMeshes(meshes);
    cad::exportCad(cad::ExportFormat::OBJ, "obstacle", mesh);

    for (size_t i = 0; i < meshes.size(); ++i)
        cad::exportCad(cad::ExportFormat::OBJ, "obstacle" + std::to_string(i), meshes[i]);
}

int main(int /*argc*/, char** /*argv*/) {
    std::vector<Mesh> meshes;
    Mesh mesh;
    for (size_t i = 0; i < 8; ++i) {
        cad::importMesh("C:/develop/iiwa_mesh/simplified/link" + std::to_string(i) + ".obj", mesh, 1, false, true);
        meshes.push_back(mesh);
    }

    cad::centerMeshes(meshes);

    for (int i = 0; i < meshes.size(); ++i)
        cad::exportCad(cad::ExportFormat::OBJ, "C:/develop/iiwa_mesh/simplifiedCentered/" + std::to_string(i), meshes[i]);

    // testFCL();

    // ProfilerStart("/tmp/cpu.prof");
    //    benchmarkFlange();
    //    benchmarkAlphaPuzzle();
    //    benchmarkHe dgehog();
    // ProfilerStop();
    return 0;
}
