#include <vrep/Helper.h>

#include <core/Logging.h>

using namespace rmpl;

Helper::Helper(unsigned int dim) : Base("VREP Helper") {
    m_dim = dim;
    m_clientId = -1;
    m_jointHandles = Vec<simxInt>(m_dim);
    m_started = false;
}

void Helper::start() {
    if (m_started)
        return;
    m_started = true;
    Logging::info("Program started", this);

    m_clientId = simxStart((simxChar *)"127.0.0.1", 19999, true, true, 2000, 5);
    if (m_clientId == -1) {
        Logging::warning("Could not connect to server", this);
        return;
    }

    Logging::info("Connected to remote API", this);
    simxInt errorcode[m_dim];
    errorcode[0] = simxGetObjectHandle(m_clientId, (simxChar *)"Jaco_joint1", &m_jointHandles[0], simx_opmode_oneshot_wait);
    errorcode[1] = simxGetObjectHandle(m_clientId, (simxChar *)"Jaco_joint2", &m_jointHandles[1], simx_opmode_oneshot_wait);
    errorcode[2] = simxGetObjectHandle(m_clientId, (simxChar *)"Jaco_joint3", &m_jointHandles[2], simx_opmode_oneshot_wait);
    errorcode[3] = simxGetObjectHandle(m_clientId, (simxChar *)"Jaco_joint4", &m_jointHandles[3], simx_opmode_oneshot_wait);
    errorcode[4] = simxGetObjectHandle(m_clientId, (simxChar *)"Jaco_joint5", &m_jointHandles[4], simx_opmode_oneshot_wait);
    errorcode[5] = simxGetObjectHandle(m_clientId, (simxChar *)"Jaco_joint6", &m_jointHandles[5], simx_opmode_oneshot_wait);
}

bool Helper::setPos(const Vec<float> &vec) {
    if (m_clientId == -1)
        return false;

    Vec<simxFloat> pos = convertVecToRad(vec);
    for (unsigned int i = 0; i < m_dim; ++i)
        simxSetJointTargetPosition(m_clientId, m_jointHandles[i], pos[i], simx_opmode_oneshot_wait);
    // usleep(1);
    return true;
}

Vec<simxFloat> Helper::convertVecToRad(const Vec<float> &vec) {
    Vec<simxFloat> rads(vec.getDim());
    for (unsigned int i = 0; i < m_dim; ++i)
        rads[i] = vec[i] * M_PI / 180;
    return rads;
}
