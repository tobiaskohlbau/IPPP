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

const unsigned int dim = 7;
const double oriRes = util::toRad(0.5);

std::shared_ptr<Environment> getEnv() {
    EnvironmentConfigurator envConfigurator;

    envConfigurator.setWorkspaceProperties(AABB(Vector3(-1500, -1500, -5), Vector3(1500, 1500, 2000)));
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/tableOriginal.obj");
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/camera.obj");
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/coffee.obj");

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
    auto goalInverse = goalTransform.inverse();

    for (size_t i = 0; i < 200000; ++i) {
        // for (size_t i = 0; i < 200; ++i) {
        // auto T =goalInverse *  robot->getTransformation(start);
        // AngleAxis angleAxis(T.rotation());
        // Vector6 diff = util::append<3, 3>(T.translation(), angleAxis.axis() * angleAxis.angle());

        auto startPose = robot.getTransformation(start);
        auto diffRot = startPose.rotation().transpose() * goalRotation;
        AngleAxis angleAxis(diffRot);
        Vector6 diff = util::append<3, 3>(goalTranslation - startPose.translation(), angleAxis.axis() * angleAxis.angle());

        if (i == 199999) {
            for (size_t j = 0; j < 6; ++j)
                std::cout << diff[j] << " ";
            std::cout << std::endl;
        }

        diff.block<3, 1>(3, 0) *= 10;
        while (diff.norm() > 1)
            diff /= 10;
        if (diff.norm() < 0.001)
            break;
        MatrixX invJ = robot.calcJacobian(start).completeOrthogonalDecomposition().pseudoInverse();
        start += (invJ * diff) * 0.5;
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

void simpleRRT() {
    auto environment = getEnv();
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(environment->getRobot());

    // set step size and goal pose constraint
    double stepSize = util::toRad(90);
    auto epsRad = util::toRad(5);
    Vector6 cMin = util::Vecd(-5, -5, -5, -epsRad + IPPP_EPSILON, -epsRad + IPPP_EPSILON, -IPPP_MAX);
    auto goalC = std::make_pair(cMin, -cMin);

    // set the task frame and the tolerance
    auto taskFrame = util::toTransform(util::Vecd(0, 0, 0, util::toRad(180), 0, 0));
    cMin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -epsRad, -epsRad, -IPPP_MAX);
    auto taskFrameC = std::make_pair(cMin, -cMin);

    // set start and goal
    Vector<dim> start = util::Vecd(9.69, 18.39, -9.69, -64, 2.51, 96.3, 0.35);
    start = util::toRad<dim>(start);

    // init upd properties
    std::vector<unsigned char> ip{127, 0, 0, 1};
    Udp udp(ip, 30005);
    // ip = {127,0,0,1};
    // udp.setRemoteDevice(ip, 30008);		//optional: only allow data from address, port

    std::vector<unsigned char> msg;
    while (1) {
        std::cout << "Waiting for UDP-packet..." << std::endl;
        udp.recvMsg(msg);
        std::string stringMsg = udp.msgToString(msg);
        std::cout << udp.msgToString(msg) << std::endl;
        if (stringMsg.find("true") == std::string::npos)
            continue;

        // read goal pose from file
        Vector6 goalPose = txtSerializer::serializePosition(stringMsg, 1000);
        goalPose[2] = 278;                 // add offset in z direction
        goalPose[3] = util::toRad(180);    // flip x axis to set gripper upside down
        std::cout << "goal pose: ";
        for (size_t i = 0; i < 6; ++i)
            std::cout << goalPose[i] << " ";
        std::cout << std::endl;

        // first connection
        ModuleConfigurator<dim> connectCreator;
        connectCreator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
        connectCreator.setEvaluatorProperties(stepSize, 30, goalC);
        connectCreator.setGraphSortCount(1500);
        connectCreator.setEnvironment(environment);
        connectCreator.setValidityCheckerType(ValidityCheckerType::FclSerial);
        connectCreator.setConstraintProperties(taskFrameC, taskFrame);

        // computation to pose
        ModuleConfigurator<dim> freeCreator;
        freeCreator.setEvaluatorType(EvaluatorType::TreePoseOrTime);
        freeCreator.setEvaluatorProperties(stepSize, 180, goalC);
        freeCreator.setGraphSortCount(4000);
        freeCreator.setEnvironment(environment);
        freeCreator.setValidityCheckerType(ValidityCheckerType::FclSerial);
        freeCreator.setSamplerType(SamplerType::InverseJacobi);
        freeCreator.setSamplingType(SamplingType::NearObstacle);
        freeCreator.setConstraintProperties(taskFrameC, taskFrame);
        freeCreator.setGoalPose(goalPose);

        // constrained planning
        //        ModuleConfigurator<dim> constrainedCreator;
        //        constrainedCreator.setEvaluatorType(EvaluatorType::TreeConnectOrTime);
        //        constrainedCreator.setEvaluatorProperties(stepSize, 90, goalC);
        //        constrainedCreator.setGraphSortCount(2000);
        //        constrainedCreator.setEnvironment(environment);
        //        constrainedCreator.setValidityCheckerType(ValidityCheckerType::FclSerialAndConstraint);
        //        constrainedCreator.setSamplerType(SamplerType::UniformBiased);
        //        constrainedCreator.setSamplingType(SamplingType::Berenson);
        //        constrainedCreator.setConstraintProperties(taskFrameC, taskFrame);

        // create planner
        RRTStarConnect<dim> connectPlanner(environment, connectCreator.getRRTOptions(stepSize), connectCreator.getGraph(),
                                           connectCreator.getGraphB());
        RRTStar<dim> freePlanner(environment, freeCreator.getRRTOptions(stepSize), freeCreator.getGraph());
        //        RRTStarConnect<dim> constrainedPlanner(environment, constrainedCreator.getRRTOptions(stepSize),
        //                                               constrainedCreator.getGraph(), constrainedCreator.getGraphB(),
        //                                               "Constrained RRTStar");

        // generate goal configuration
        auto goal = getGoal(*std::dynamic_pointer_cast<SerialRobot>(environment->getRobot()), start, goalPose);
        goal = getGoal(*std::dynamic_pointer_cast<SerialRobot>(environment->getRobot()), goal, goalPose);

        bool connected1 = connectPlanner.computePath(start, goal, 600, 24);
        bool connected2 = freePlanner.computePathToPose(goal, goalPose, goalC, 120, 24);
        //        bool connected3 = false;

        std::vector<Vector<dim>> path, pathConstrained;
        Vector<dim> constrainedGoal;
        if (!connected1 || !connected2) {
            udp.sendMsg(udp.msgToChar("STOP!"), udp.getRemoteIp(), udp.getRemotePort());
            continue;
        }

        Logging::info("First and second Path planned!", "KUKA");
        path = connectPlanner.getPath(oriRes, oriRes);
        auto path2 = freePlanner.getPath(oriRes, oriRes);
        path.insert(path.begin(), path2.begin(), path2.end());
        auto pathData = txtSerializer::serialize<dim>(path);
        ui::save(FLAGS_workDir + "freePath.txt", pathData);

        auto json = jsonSerializer::serialize<dim>(path);
        ui::save("kukaPath.json", json);

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


        // constrainedGoal = path.back();
        // connected3 = constrainedPlanner.computePath(start, constrainedGoal, 1200, 24);
        // Logging::info("Constrained Path planned!", "KUKA");
        // pathConstrained = constrainedPlanner.getPath(oriRes, oriRes);
        // auto json = jsonSerializer::serialize<dim>(pathConstrained);
        // ui::save("kukaPathConstrained.json", json);

//        auto json = jsonSerializer::serialize<dim>(path);
//        ui::save("kukaPath.json", json);
    }
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    Logging::setLogLevel(LogLevel::info);

    simpleRRT();
    Stats::writeData(std::cout);
    //    std::string string;
    //    std::cin >> string;

    return 0;
}
