#include <chrono>
#include <iostream>
#include <memory>

//#include <gperftools/profiler.h>

#include <Core>
#include <Environment>
#include <Planner>
#include <core/collisionDetection/CollisionDetectionPqpBenchmark.hpp>
//#include <core/collisionDetection/CollisionDetectionFcl.hpp>
//#include <environment/model/ModelFactoryFcl.h>

#include <modelDirectory.h>
#include <ui/BenchmarkReader.h>
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

    Vector6 minBoundary = util::Vecf(config.minBoundary[0], config.minBoundary[1], config.minBoundary[2], 0, 0, 0);
    Vector6 maxBoundary = util::Vecf(config.maxBoundary[0], config.maxBoundary[1], config.maxBoundary[2], util::twoPi(),
                                     util::twoPi(), util::twoPi());
    std::shared_ptr<RobotBase> robot(new MobileRobot(6, minBoundary, maxBoundary));
    robot->setBaseModel(robotModel);

    std::shared_ptr<Environment> environment(new Environment(3, AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    environment->addObstacle(obstacleModel);

    std::shared_ptr<CollisionDetection<6>> collision(new CollisionDetectionPqp<6>(environment));
    std::shared_ptr<CollisionDetection<6>> collisionBenchmark(new CollisionDetectionPqpBenchmark<6>(environment));
    std::shared_ptr<TrajectoryPlanner<6>> trajectory(new TrajectoryPlanner<6>(collision, 3));
    std::shared_ptr<TrajectoryPlanner<6>> trajectoryBenchmark(new TrajectoryPlanner<6>(collisionBenchmark, 3));
    std::shared_ptr<Sampler<6>> sampler(new Sampler<6>(environment));
    std::shared_ptr<Sampling<6>> sampling(new Sampling<6>(environment, collision, trajectory, sampler));
    for (int i = 3; i < 6; ++i) {
        config.obstacleConfig[i] *= util::toRad();
    }

    std::shared_ptr<Sampling<6>> samplingBenchmark(
        new Sampling<6>(environment, collisionBenchmark, trajectoryBenchmark, sampler));

    std::shared_ptr<DistanceMetric<dim>> distanceMetric(new DistanceMetric<dim>());
    RRTOptions<6> options(40, collision, trajectory, sampling, distanceMetric);
    RRTOptions<6> optionsBenchmark(40, collisionBenchmark, trajectoryBenchmark, samplingBenchmark, distanceMetric);

    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> neighborFinder(
        new KDTree<dim, std::shared_ptr<Node<dim>>>(distanceMetric));
    std::shared_ptr<Graph<dim>> graph(new Graph<dim>(0, neighborFinder));
    std::shared_ptr<Graph<dim>> graphBenchmark(new Graph<dim>(0, neighborFinder));

    RRTStar<6> planner(environment, options, graph);
    RRTStar<6> plannerBenchmark(environment, optionsBenchmark, graphBenchmark);

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

int main(int argc, char** argv) {
    std::vector<Mesh> meshes;
    bool result = cad::importMeshes("/users/skaden/Downloads/box.dae", meshes);
    if (result) {
        int count = 1;
        for (auto mesh : meshes) {
            cad::exportCad(cad::ExportFormat::OBJ, "cube" + std::to_string(count), mesh);
            ++count;
        }
    }

    // ProfilerStart("/tmp/cpu.prof");
    modelDir = getModelDirectory();
    // benchmarkFlange();
    // benchmarkAlphaPuzzle();
    // benchmarkHedgehog();
    // ProfilerStop();
    return 0;
}
