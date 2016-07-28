#include <vrep/Helper.h>

using namespace rmpl;

Helper::Helper(unsigned int dim)
        : Base("VREP Helper") {
    m_dim = dim;
    m_clientId = -1;
    m_jointHandles = Vec<simxInt>(m_dim);
    m_started = false;
}

void Helper::start() {
    if (m_started)
        return;
    m_started = true;
    this->sendMessage("Program started");

    m_clientId=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
    if (m_clientId == -1) {
        this->sendMessage("Could not connect to server");
        return;
    }

    this->sendMessage("Connected to remote API");
    simxInt errorcode[m_dim];
    errorcode[0]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint1",&m_jointHandles[0],simx_opmode_oneshot_wait);
    errorcode[1]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint2",&m_jointHandles[1],simx_opmode_oneshot_wait);
    errorcode[2]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint3",&m_jointHandles[2],simx_opmode_oneshot_wait);
    errorcode[3]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint4",&m_jointHandles[3],simx_opmode_oneshot_wait);
    errorcode[4]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint5",&m_jointHandles[4],simx_opmode_oneshot_wait);
    errorcode[5]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint6",&m_jointHandles[5],simx_opmode_oneshot_wait);
}

bool Helper::setPos(const Vec<REAL> &vec) {
    if (m_clientId == -1)
        return false;

    Vec<simxFloat> pos = convertVecToRad(vec);
    for (unsigned int i = 0; i < m_dim; ++i)
        simxSetJointTargetPosition(m_clientId,m_jointHandles[i],pos[i],simx_opmode_oneshot_wait);
    usleep(100);
    return true;
}

bool Helper::checkCollision(const Vec<REAL> &jointAngles) {
    if (m_clientId == -1)
        return false;

    const Vec<simxFloat> pos = convertVecToRad(jointAngles);
    simxFloat angles[m_dim];
    simxInt handles[m_dim];
    for (unsigned int i = 0; i < m_dim; ++i) {
        angles[i] = pos[i];
        handles[i] = m_jointHandles[i];
    }
    simxInt numResult;
    simxInt *result;
    simxInt numDist;
    simxFloat *distance;
    simxCallScriptFunction(m_clientId,"Jaco",sim_scripttype_childscript,"checkCollision_function",m_dim,handles,m_dim,angles,0,NULL,0,NULL,&numResult,&result,&numDist,&distance,NULL,NULL,NULL,NULL,simx_opmode_blocking);

    if (numResult == 1)
        if (result[0] == 1)
            if (distance[0] > 0.01) {
                this->sendMessage("Distance to objects is: " + std::to_string(distance[0]));
                return false;
            }
            else {
                this->sendMessage("Roboter is in collision");
                return true;
            }

    return true;
}

bool Helper::checkCollision(const std::vector<Vec<REAL>> &jointAngles) {
    if (m_clientId == -1)
        return false;

    simxFloat angles[m_dim * jointAngles.size()];
    simxInt handles[m_dim * jointAngles.size()];
    for (int i = 0; i < jointAngles.size(); ++i) {
        const Vec<simxFloat> pos = convertVecToRad(jointAngles[i]);
        for (unsigned int j = 0; j < m_dim; ++j) {
            angles[i + j] = pos[j];
            handles[i + j] = m_jointHandles[j];
        }
    }
    simxInt numResult;
    simxInt *result;
    simxInt numDist;
    simxFloat *distance;
    simxCallScriptFunction(m_clientId,"Jaco",sim_scripttype_childscript,"checkTrajectory_function",m_dim,handles,m_dim,angles,0,NULL,0,NULL,&numResult,&result,&numDist,&distance,NULL,NULL,NULL,NULL,simx_opmode_blocking);

    if (numResult == 1)
        if (result[0] == 1)
            if (distance[0] > 0.01) {
                this->sendMessage("Distance to objects is: " + std::to_string(distance[0]));
                return false;
            }
            else {
                this->sendMessage("Roboter is in collision");
                return true;
            }

    return true;
}

Vec<simxFloat> Helper::convertVecToRad(const Vec<REAL> &vec) {
    Vec<simxFloat> rads(vec.getDim());
    for (unsigned int i = 0; i < m_dim; ++i)
        rads[i] = vec[i]*M_PI/180;
    return rads;
}
