#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/UI.h>

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

using namespace ippp;

bool myfunction(fs::v1::directory_entry i, fs::v1::directory_entry j) {
    std::string strI = i.path().stem().string();
    std::string strJ = j.path().stem().string();
    int intI = std::stoi(strI);
    int intJ = std::stoi(strJ);
    return intI < intJ;
}

void splitCad() {
    std::vector<Mesh> meshes;
    Mesh mesh;
    cad::importMeshes("C:/develop/tmp/gearbox/gearbox.dae", meshes, 1, false, true);

    for (int i = 0; i < meshes.size(); ++i)
        cad::exportCad(cad::ExportFormat::OBJ, "C:/develop/tmp/" + std::to_string(i), meshes[i]);
}

std::shared_ptr<RobotBase> createMobileRobot(ModelFactory &factory, const fs::v1::directory_entry &path, const AABB &bounding) {
    Vector3 min = bounding.min();
    Vector3 max = bounding.max();

    Vector6 min6, max6;
    min6 = util::Vecd(min[0], min[1], min[2], 0, 0, 0);
    max6 = util::Vecd(max[0], max[1], max[2], util::twoPi(), util::twoPi(), util::twoPi());
    std::vector<DofType> types = {DofType::volumetricPos, DofType::volumetricPos, DofType::volumetricPos,
                                  DofType::volumetricRot, DofType::volumetricRot, DofType::volumetricRot};
    auto robotModel = factory.createModelFromFile(path.path().string());
    auto robot = std::make_shared<MobileRobot>(6, std::make_pair(min6, max6), types, fs::path(path).filename().string());
    robot->setBaseModel(robotModel);
    return robot;
}

void testCollision() {
    AABB bounding(Vector3(-10000, -10000, -10000), Vector3(10000, 10000, 10000));
    auto environment = std::make_shared<Environment>(bounding);
    ModelFactoryFcl factory;

    std::string directory = "C:/develop/tmp/";
    std::vector<fs::v1::directory_entry> filePaths;
    for (auto &p : fs::directory_iterator(directory))
        if (fs::path(p).extension() == ".obj")
            filePaths.push_back(p);

    std::sort(filePaths.begin(), filePaths.end(), myfunction);
    for (auto &filePath : filePaths) {
        auto robot = createMobileRobot(factory, filePath, bounding);
        environment->addRobot(robot);
    }

    CollisionRequest request;
    request.completeCheck = true;
    auto collsion = std::make_shared<CollisionFclMobile<174>>(environment, request);
    Vector<174> config = Vector<174>::Constant(0);
    std::cout << "checking collision" << std::endl;
    collsion->check(config);
}

void centerMeshes() {
    std::string directory = "C:/develop/tmp/";
    std::vector<fs::v1::directory_entry> filePaths;
    for (auto &p : fs::directory_iterator(directory))
        if (fs::path(p).extension() == ".obj")
            filePaths.push_back(p);

    std::vector<Mesh> meshes;
    std::vector<Vector3> meshCenters;
    for (auto &filePath : filePaths) {
        Mesh mesh;
        if (cad::importMesh(filePath.path().string(), mesh)) {
            meshCenters.push_back(cad::calcCenterOfMesh(mesh));
            cad::centerMesh(mesh);
            meshes.push_back(mesh);
        }
    }

    size_t count = 0;
    for (auto &mesh : meshes) {
        cad::exportCad(cad::ExportFormat::OBJ, directory + "/centered/" + std::to_string(count), mesh);
        ++count;
    }

    std::ofstream file;
    file.open(directory + "center.txt");
    for (auto center : meshCenters) {
        for (size_t i = 0; i < 3; ++i) {
            file << center[i] << "  ";
        }
        file << std::endl;
    }
    file.close();
}

void transformMesh() {
    std::string directory = "C:/develop/tmp/";
    std::vector<fs::v1::directory_entry> filePaths;
    for (auto &p : fs::directory_iterator(directory))
        if (fs::path(p).extension() == ".obj")
            filePaths.push_back(p);

    std::vector<Mesh> meshes;
    Transform transform = util::toTransform(util::Vecd(13, 7, 127, 0, 0, 0));
    for (auto &filePath : filePaths) {
        Mesh mesh;
        if (cad::importMesh(filePath.path().string(), mesh)) {
            cad::transformMesh(mesh, transform);
            meshes.push_back(mesh);
        }
    }

    size_t count = 0;
    for (auto &mesh : meshes) {
        cad::exportCad(cad::ExportFormat::OBJ, directory + "/transformed/" + std::to_string(count), mesh);
        ++count;
    }
}

int main(int /*argc*/, char ** /*argv*/) {
    Logging::setLogLevel(LogLevel::trace);

    // splitCad();
    // testCollision();
    // centerMeshes();
    transformMesh();
    std::string str;
    std::cin >> str;
    return 0;
}
