#include <chrono>
#include <iostream>
#include <memory>
#include <string>

//#include <gperftools/profiler.h>

//#include <boost/filesystem.hpp>
//#include <boost/range/iterator_range.hpp>

#include <ippp/Core>
#include <ippp/Environment>
#include <ippp/Planner>
#include <ippp/core/collisionDetection/CollisionDetectionPqpBenchmark.hpp>
//#include <ippp/core/collisionDetection/CollisionDetectionFcl.hpp>
//#include <ippp/environment/model/ModelFactoryFcl.h>

#include <modelDirectory.h>
#include <ui/BenchmarkReader.h>
#include <ui/ModuleCreator.hpp>
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

    ModuleCreator<dim> creator(environment, collision, MetricType::L2, NeighborType::KDTree, PathModifierType::NodeCut,
                               SamplerType::SamplerRandom, SamplingType::Straight, TrajectoryType::Linear);
    ModuleCreator<dim> creatorBenchmark(environment, collisionBenchmark, MetricType::L2, NeighborType::KDTree,
                                        PathModifierType::NodeCut, SamplerType::SamplerRandom, SamplingType::Straight,
                                        TrajectoryType::Linear);

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

void transformCad() {
    std::string file = "/users/skaden/untitled.dae";
    std::vector<Mesh> meshes;
    bool result = cad::importMeshes(file, meshes);
    for (int i = 0; i < meshes.size(); ++i)
        cad::exportCad(cad::ExportFormat::OBJ, std::to_string(i), meshes[i]);
}

void generateMap() {
    const unsigned int dim = 3;
    Vector3 min(-50,-50,-50);
    Vector3 max(50,50,50);
    AABB bounding(min, max);

    std::vector<Triangle2D> triangles;
    triangles.push_back(Triangle2D(Vector2(0, 0), Vector2(25, 0), Vector2(25, 50)));
    ModelFactoryTriangle2D factory;
    std::shared_ptr<ModelContainer> baseModel = factory.createModel(triangles);

    std::shared_ptr<RobotBase> robot(new TriangleRobot2D(baseModel, std::make_pair(min,max)));
    std::shared_ptr<Environment> environment(new Environment(dim, bounding, robot));
    std::shared_ptr<Sampler<dim>> sampler(new SamplerRandom<dim>(environment));

    MapGenerator<3> mapGenerator(bounding, sampler);
    auto meshes = mapGenerator.generateMap(20, Vector3(30, 30, 30));
    auto mesh = cad::mergeMeshes(meshes);
    cad::exportCad(cad::ExportFormat::OBJ, "obstacle", mesh);

    for (size_t i = 0; i < meshes.size(); ++i) {
        cad::exportCad(cad::ExportFormat::OBJ, "obstacle" + std::to_string(i), meshes[i]);
    }
}

int main(int argc, char** argv) {
    modelDir = getModelDirectory();

    // transformCad();
    generateMap();

    // ProfilerStart("/tmp/cpu.prof");
    //    benchmarkFlange();
    //    benchmarkAlphaPuzzle();
    //    benchmarkHe dgehog();
    // ProfilerStop();
    return 0;
}

//    std::string dir = "/users/skaden/";
//    std::vector<std::string> meshFiles;
//    meshFiles.push_back(dir + "i.obj");
//    meshFiles.push_back(dir + "y.obj");
//    meshFiles.push_back(dir + "z.obj");
//    meshFiles.push_back(dir + "l.obj");
//    meshFiles.push_back(dir + "x.obj");
//    meshFiles.push_back(dir + "c.obj");
//    meshFiles.push_back(dir + "f.obj");
//    meshFiles.push_back(dir + "s.obj");
//    meshFiles.push_back(dir + "b.obj");
//    meshFiles.push_back(dir + "m.obj");
//    meshFiles.push_back(dir + "t.obj");
//    meshFiles.push_back(dir + "v.obj");
//
//    std::vector<std::string> names = {"i", "y", "z", "l", "x", "c", "f", "s", "b", "m", "t", "v"};
//    std::vector<Mesh> meshes;
////    int count = 0;
////    for (auto file : meshFiles) {
////        Mesh mesh;
////        bool result = cad::importMesh(file, mesh);
////        meshes.push_back(mesh);
////
////        ++count;
////    }
//
//    cad::importMeshes(dir + "pentomino.dae", meshes);
////    double pi = util::pi();
////    cad::transformVertices(util::Vecd(0.4, 2.4, 0.4,    0, 0 ,0), meshes[0].vertices);
////    cad::transformVertices(util::Vecd(1.85, 2.65, 0.4,  1*pi, 0, 0), meshes[1].vertices);
////    cad::transformVertices(util::Vecd(2.9, 3.15, 0.4,   1*pi, 0, 0.5*pi), meshes[2].vertices);
////    cad::transformVertices(util::Vecd(3.4, 3.27, -0.13, 0, -0.5*pi, 0), meshes[3].vertices);
////    cad::transformVertices(util::Vecd(1.4, 0.4, -0.6,   0.5*pi, 0 ,0), meshes[4].vertices);
////    cad::transformVertices(util::Vecd(2.7, 0.4, -0.6,   0.5*pi, 0, 1*pi), meshes[5].vertices);
////    cad::transformVertices(util::Vecd(2.32, 1.4, -0.72, -0.5*pi , 0, 1*pi), meshes[6].vertices);
////    cad::transformVertices(util::Vecd(1.4, 2.4, -0.6,   0, 0, 0.5*pi), meshes[7].vertices);
////    cad::transformVertices(util::Vecd(1.53, 3.93, -0.6, 0, 0, 0.5*pi), meshes[8].vertices);
////    cad::transformVertices(util::Vecd(1.28, 1.52, -1.6, 0, 0, 0), meshes[9].vertices);
////    cad::transformVertices(util::Vecd(1.1, 3.4, -1.6,   0, 0, 0), meshes[10].vertices);
////    cad::transformVertices(util::Vecd(2.6, 3.6, -1.6,   0, 0, -0.5*pi), meshes[11].vertices);
//
//    for (int i = 0; i < 12; ++i)
//        cad::exportCad(cad::ExportFormat::OBJ, names[i], meshes[i]);