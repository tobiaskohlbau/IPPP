#include <vrep/Helper.h>

using namespace rmpl;

Helper::Helper(const unsigned int &dim) {
    m_dim = dim;
    m_clientId = -1;

    m_jointHandles = Vec<simxInt>(m_dim);
    m_linkHandles = Vec<simxInt>(m_dim);
    m_collisionHandle = Vec<simxInt>(m_dim);
    for (unsigned int i = 0; i < m_dim; ++i)
        m_jointHandles[i] = -1;
}

void Helper::start() {
    std::cout << "Program started" << std::endl;

    simxInt errorCodejaco, errorcode[10];

    m_clientId=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
    if (m_clientId!=-1) {
        std::cout << "connected to  remote API" << std::endl;

        errorcode[1]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint1",&m_jointHandles[0],simx_opmode_oneshot_wait);
        errorcode[2]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint2",&m_jointHandles[1],simx_opmode_oneshot_wait);
        errorcode[3]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint3",&m_jointHandles[2],simx_opmode_oneshot_wait);
        errorcode[4]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint4",&m_jointHandles[3],simx_opmode_oneshot_wait);
        errorcode[5]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint5",&m_jointHandles[4],simx_opmode_oneshot_wait);
        errorcode[6]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint6",&m_jointHandles[5],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link1",&m_linkHandles[0],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link2",&m_linkHandles[1],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link3",&m_linkHandles[2],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link4",&m_linkHandles[3],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link5",&m_linkHandles[4],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link6",&m_linkHandles[5],simx_opmode_oneshot_wait);

        simxInt handleAll = -2;
        simxInt linkhandle = simxGetObjectHandle(m_clientId, (simxChar*)"Jaco_link1", &handleAll, simx_opmode_oneshot_wait);

        std::cout << std::endl << std::endl;
    }
    else {
        std::cout << "could not connect to server" << std::endl;
    }
}

bool Helper::setPos(const Vec<float> &vec) {
    if (m_clientId != -1) {
        Vec<simxFloat> pos = convertVecToRad(vec);
        for (unsigned int i = 0; i < m_dim; ++i)
            simxSetJointTargetPosition(m_clientId,m_jointHandles[i],pos[i],simx_opmode_oneshot_wait);
        usleep(3000000);
        simxInt numResult;
        simxInt *result;
        simxInt numDist;
        simxFloat *distance;
        simxCallScriptFunction(m_clientId,"Jaco",sim_scripttype_childscript,"checkDist_function",0,NULL,0,NULL,0,NULL,0,NULL,&numResult,&result,&numDist,&distance,NULL,NULL,NULL,NULL,simx_opmode_blocking);

        if (result[0] == 1)
            std::cout << "distance: " << distance[0] << std::endl;
    }
}

bool Helper::checkCollision(const Vec<float> &jointAngles) {
    const Vec<simxFloat> pos = convertVecToRad(jointAngles);
    simxFloat angles[6] = {pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]};
    simxInt handles[6] = {m_jointHandles[0], m_jointHandles[1], m_jointHandles[2], m_jointHandles[3], m_jointHandles[4], m_jointHandles[5]};
    simxInt numDist;
    simxFloat *distance;
    simxCallScriptFunction(m_clientId,"Jaco",sim_scripttype_childscript,"checkCollision_function",1,handles,6,angles,0,NULL,0,NULL,NULL,NULL,&numDist,&distance,NULL,NULL,NULL,NULL,simx_opmode_blocking);

    if (numDist != NULL)
        if (numDist > 0)
            if (distance[0] > 0.01) {
                std::cout << "Distance to objects is: " << distance[0] << std::endl;
                return false;
            }

    std::cout << "Roboter is in collisiont" << std::endl;
    return true;
}

Vec<simxFloat> Helper::convertVecToRad(const Vec<float> &vec) {
    Vec<simxFloat> rads(vec.getDim());
    for (unsigned int i = 0; i < m_dim; ++i) {
        rads[i] = vec[i]*M_PI/180;
    }
    return rads;
}
