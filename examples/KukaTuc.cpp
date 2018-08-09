#include <ctime>

#include <gflags/gflags.h>

#include <Udp.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");
DEFINE_string(workDir, "/media/nfs/debussy/data_raid6/smb_public_share/Tmp/openDoor/", "work directory");

// predefined parameter
const unsigned int dim = 7;
double stepSize = util::toRad(90);
Vector<dim> start = util::toRad<dim>(util::Vecd(18.4, 2.17, 19.55, 103.33, 1.28, 105.48, -0.73));
Vector<dim> tmpGoal = util::toRad<dim>(util::Vecd(-120.4, 2.17, 19.55, 83.33, 1.28, 75.48, 60.1));
const double oriRes = util::toRad(5);
double epsOri = util::toRad(10);
double epsPos = 5;
Vector6 cMin = util::Vecd(-epsPos, -epsPos, -epsPos, -epsOri, -epsOri, -epsOri);
auto goalC = std::make_pair(cMin, -cMin);

std::shared_ptr<Environment> getEnv() {
    EnvironmentConfigurator envConfigurator;

    envConfigurator.setWorkspaceProperties(AABB(Vector3(-1500, -1500, -5), Vector3(1500, 1500, 2000)));
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/floor.obj");
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/iiwaEvaluation/floor.obj", util::Vecd(0, 0, 1210, 0, 0, 0));
    //envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/tableOriginal.obj");
    //envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/camera.obj");
    //envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/coffee.obj");

    Vector7 minRobotBound = util::Vecd(-165, -115, -165, -115, -165, -115, -170);
    Vector7 maxRobotBound = util::Vecd(165, 115, 165, 115, 165, 115, 170);
    minRobotBound = util::toRad<7>(minRobotBound);
    maxRobotBound = util::toRad<7>(maxRobotBound);
    std::vector<DhParameter> dhParameters({DhParameter(util::toRad(-90), 0, 340), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(util::toRad(90), 0, 400), DhParameter(util::toRad(-90), 0, 0),
                                           DhParameter(util::toRad(-90), 0, 400), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(0, 0, 126)});
    std::string iiwa = FLAGS_assetsDir + "/robotModels/iiwa/";
    std::vector<std::string> linkModelFiles = {iiwa + "link1.obj", iiwa + "link2.obj", iiwa + "link3.obj", iiwa + "link4.obj",
                                               iiwa + "link5.obj", iiwa + "link6.obj", iiwa + "link7.obj"};
    envConfigurator.setRobotBaseModelFile(iiwa + "link0.obj");
    envConfigurator.setRobotType(RobotType::Serial);

    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    std::vector<DofType> dofTypes(7, DofType::jointRot);
    envConfigurator.setRobotBaseProperties(dim, dofTypes, std::make_pair(minRobotBound, maxRobotBound));
    std::vector<Vector6> linkOffsets(7, util::Vecd(0, 0, 0, 0, 0, 0));
    linkOffsets[0] = util::Vecd(0, 0, 150, 0, 0, 0);
    linkOffsets[1] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    linkOffsets[2] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[3] = util::Vecd(0, 0, 0, -util::halfPi(), 0, 0);
    linkOffsets[4] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[5] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    auto linkTransforms = util::toTransform(linkOffsets);
    envConfigurator.setSerialRobotProperties(dhParameters, linkModelFiles, linkTransforms, Transform::Identity(),
                                             util::toTransform(util::Vecd(0, 0, 205, 0, 0, 0)), Transform::Identity(),
                                             FLAGS_assetsDir + "/robotModels/rhmiGripper.obj");

    envConfigurator.saveConfig("KukaEnvConfig.json");
    return envConfigurator.getEnvironment();
}

Vector<dim> getGoal(SerialRobot &robot, Vector<dim> startConfig, Vector6 goalPose) {
    Vector<dim> start = startConfig;
    Vector<dim> goal;

    auto goalTransform = util::toTransform(goalPose);
    auto goalRotation = goalTransform.rotation();
    Vector3 goalTranslation = goalTransform.translation();
    Transform goalInverse = goalTransform.inverse();

    size_t maxIterations = 300000;
    for (size_t i = 0; i < maxIterations; ++i) {
        // for (size_t i = 0; i < 200; ++i) {
        // auto T =goalInverse *  robot->getTransformation(start);
        // AngleAxis angleAxis(T.rotation());
        // Vector6 diff = util::append<3, 3>(T.translation(), angleAxis.axis() * angleAxis.angle());

        //auto startPose = robot.getTransformation(start);
        //auto diffRot = startPose.rotation().transpose() * goalRotation;
        //AngleAxis angleAxis(diffRot);

        Transform pose = robot.getTransformation(start);
        auto rotation = pose.rotation();
        //AngleAxis angleAxis(pose.rotation());

        Vector6 diff = util::append<3, 3>(pose.translation() - goalTranslation, rotation.eulerAngles(0,1,2));
        //Vector6 diff = util::append<3, 3>(goalTranslation - startPose.translation(), angleAxis.axis() * angleAxis.angle());

        if (i == maxIterations - 1)
            std::cout << "pose difference: " << diff.transpose() << std::endl;
        if (diff.squaredNorm() < 0.01)
            break;

        diff.block<3, 1>(3, 0) *= 10;
        while (diff.squaredNorm() > 1)
            diff /= 10;

        MatrixX invJ = robot.calcJacobian(start).completeOrthogonalDecomposition().pseudoInverse();
        start -= (invJ * diff);

        for (size_t j = 0; j < dim; ++j) {
            while (goal[j] > util::pi())
                goal[j] -= util::twoPi();
            while (goal[j] < -util::pi())
                goal[j] += util::twoPi();
        }
    }
    goal = start;
    // reduce multiply of pi
    for (size_t j = 0; j < dim; ++j) {
        while (goal[j] > util::pi())
            goal[j] -= util::twoPi();
        while (goal[j] < -util::pi())
            goal[j] += util::twoPi();
    }
    // check boundary of robot
    auto boundary = robot.getBoundary();
    for (size_t j = 0; j < dim; ++j) {
        if (goal[j] > boundary.second[j])
            goal[j] = boundary.second[j] - IPPP_EPSILON;
        if (goal[j] < boundary.first[j])
            goal[j] = boundary.first[j] + IPPP_EPSILON;
    }
    //    std::cout << "Configuration: ";
    //    for (size_t j = 0; j < dim; ++j)
    //        std::cout << util::toDeg(goal[j]) << " ";
    //    std::cout << std::endl;
    return goal;
}

std::vector<Vector<dim>> planPath(std::shared_ptr<Environment> &env, const Vector6 &goalPose) {
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(env->getRobot());
    Vector<dim> goal;

    // first connection
    ModuleConfigurator<dim> connectCreator;
    connectCreator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
    connectCreator.setEvaluatorProperties(stepSize, 30);
    connectCreator.setGraphSortCount(1500);
    connectCreator.setEnvironment(env);
    connectCreator.setValidityCheckerType(ValidityCheckerType::FclSerial);
    // computation to pose
    ModuleConfigurator<dim> freeCreator;
    freeCreator.setEvaluatorType(EvaluatorType::TreePoseOrTime);
    freeCreator.setEvaluatorProperties(stepSize, 180);
    freeCreator.setGraphSortCount(4000);
    freeCreator.setEnvironment(env);
    freeCreator.setValidityCheckerType(ValidityCheckerType::FclSerial);
    freeCreator.setSamplerType(SamplerType::InverseJacobi);
    freeCreator.setSamplingType(SamplingType::NearObstacle);
    freeCreator.setGoalPose(goalPose);

    // create planner
    RRTStarConnect<dim> connectPlanner(env, connectCreator.getRRTOptions(stepSize), connectCreator.getGraph(),
                                       connectCreator.getGraphB());
    RRTStar<dim> freePlanner(env, freeCreator.getRRTOptions(stepSize), freeCreator.getGraph());

    // generate goal configuration
    //goal = getGoal(*serialRobot, start, goalPose);
    //goal = getGoal(*serialRobot, goal, goalPose);
    goal = tmpGoal;
    std::cout << "goal configuration" << goal.transpose() << std::endl;

    Logging::info("Start planning of first path", "KUKA");
    bool connected1 = connectPlanner.computePath(start, goal, 240, 24);
    Logging::info("Start planning of second path", "KUKA");
    bool connected2 = true;//freePlanner.computePathToPose(goal, goalPose, goalC, 120, 24);

    std::vector<Vector<dim>> path;
    if (!connected1 || !connected2)
        return path;

    path = connectPlanner.getPath(oriRes, oriRes);
    auto path2 = freePlanner.getPath(oriRes, oriRes);
    //path.insert(path.begin(), path2.begin(), path2.end());
    return path;
}

std::string savePath(const std::vector<Vector<dim>> &path) {
    auto pathData = txtSerializer::serialize<dim>(path);
    ui::save(FLAGS_workDir + "freePath.txt", pathData);

    auto json = jsonSerializer::serialize<dim>(path);
    ui::save("kukaPath.json", json);

    return pathData;
}

void simpleRRT() {
    // generate environment and get robot
    auto env = getEnv();

    // init upd
    std::vector<unsigned char> ip{127, 0, 0, 1}, msg;
    Udp udp(ip, 30006);
    std::string stringMsg = "true \n 1 1 1";

    while (true) {
        std::cout << "Waiting for UDP-packet..." << std::endl;
        //udp.recvMsg(msg);
        //stringMsg = udp.msgToString(msg);
        std::cout << "stringMsg: " << stringMsg << std::endl << std::endl;
        if (stringMsg.find("true") == std::string::npos)
            continue;

        // read goal pose from file
        Vector6 goalPose = txtSerializer::deserializePosition(stringMsg, 1000);
        goalPose[1] = -goalPose[1];
        goalPose[2] = -goalPose[2];
        //goalPose[2] = 278;    // add offset in z direction
        //goalPose[3] = util::toRad(180);    // flip x axis to set gripper upside down
//        auto goalTransform = util::toTransform(goalPose);
//        std::cout << "goal pose: " << goalTransform.translation().transpose() << std::endl;
//        goalTransform = goalTransform * toRobotBase;
//        std::cout << "goal pose: " << goalTransform.translation().transpose() << std::endl;
//        goalPose = util::toPoseVec(goalTransform);
        std::cout << "goal pose: " << goalPose.transpose() << std::endl;

        auto path = planPath(env, goalPose);

        if (path.empty()) {
            udp.sendMsg(udp.msgToChar("STOP!"), udp.getRemoteIp(), udp.getRemotePort());
            continue;
        }

        auto pathData = savePath(path);
        Logging::info("First and second Path planned!", "KUKA");

        udp.sendMsg(udp.msgToChar("MOVE!"), udp.getRemoteIp(), udp.getRemotePort());
        udp.recvMsg(msg);
        std::cout << udp.msgToString(msg) << std::endl;
        udp.sendMsg(udp.msgToChar(std::to_string(path.size())), udp.getRemoteIp(), udp.getRemotePort());
        udp.recvMsg(msg);
        std::cout << udp.msgToString(msg) << std::endl;
        std::stringstream stream(pathData);
        std::string line;
        std::getline(stream, line);
        for (size_t i = 0; i < path.size(); ++i) {
            std::getline(stream, line);
            udp.sendMsg(udp.msgToChar(line), udp.getRemoteIp(), udp.getRemotePort());
            udp.recvMsg(msg);
            std::cout << udp.msgToString(msg) << std::endl;
        }
        udp.sendMsg(udp.msgToChar("STOP!"), udp.getRemoteIp(), udp.getRemotePort());
    }
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    Logging::setLogLevel(LogLevel::info);

    simpleRRT();

    return 0;
}
