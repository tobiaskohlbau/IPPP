#ifndef VREPHELPER_H_
#define VREPHELPER_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string>
#include <cstdint>

#include "Vec.h"

extern "C" {
    #include <extApi.h>
    #include <extApiPlatform.h>
    #include <v_repConst.h>
}
#define NON_MATLAB_PARSING

template<unsigned int dim>
class VrepHelper
{
public:
    VrepHelper();
    void startVrep();
    bool setPos(const Vec<dim, float> &vec);

private:
    Vec<dim, simxFloat> convertVecToRad(const Vec<dim, float> &vec); // convert to simxFloat array and radiant
    simxInt m_clientId;
    simxInt m_jacoHandle;
    Vec<dim, simxInt> m_jointHandles;
    Vec<dim, simxInt> m_linkHandles;
    Vec<dim, simxInt> m_collisionHandle;
};

template<unsigned int dim>
VrepHelper<dim>::VrepHelper() {
    m_clientId = -1;
    for (unsigned int i = 0; i < dim; ++i)
        m_jointHandles[i] = -1;
}

template<unsigned int dim>
void VrepHelper<dim>::startVrep() {
    std::cout << "Program started" << std::endl;

    simxInt errorCodejaco, errorcode[10];

    m_clientId=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
    if (m_clientId!=-1) {
        std::cout << "connected to Vrep remote API" << std::endl;

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
        m_collisionHandle[0] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link1", &m_linkHandles[0], simx_opmode_oneshot_wait);
        m_collisionHandle[1] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link2", &m_linkHandles[1], simx_opmode_oneshot_wait);
        m_collisionHandle[2] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link3", &m_linkHandles[2], simx_opmode_oneshot_wait);
        m_collisionHandle[3] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link4", &m_linkHandles[3], simx_opmode_oneshot_wait);
        m_collisionHandle[4] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link5", &m_linkHandles[4], simx_opmode_oneshot_wait);
        m_collisionHandle[5] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link6", &m_linkHandles[5], simx_opmode_oneshot_wait);

        simxInt handleAll = -2;
        simxInt linkhandle = simxGetObjectHandle(m_clientId, (simxChar*)"Jaco_link1", &handleAll, simx_opmode_oneshot_wait);

        //for(int i=0;i<6;i++)
        //    printf("Joint code%d %d\n",i,m_jointHandles[i]);

        errorCodejaco=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco", &m_jacoHandle , simx_opmode_oneshot_wait);
        printf("Error code Jaco %d\n",errorCodejaco);

        simxUChar collisionState;
        for (unsigned int i = 0; i < dim; ++i) {
            simxReadCollision(m_clientId, m_collisionHandle[i], &collisionState, simx_opmode_oneshot_wait);
            printf("Error code Jaco link collision %d %d\n",i,collisionState);
        }

        std::cout << std::endl << std::endl;
    }
    else {
        std::cout << "could not connect to server" << std::endl;
    }
}

template<unsigned int dim>
bool VrepHelper<dim>::setPos(const Vec<dim, float> &vec) {
    if (m_clientId != -1) {

        simxInt errorCodeJointPos[dim];
        Vec<dim, simxFloat> pos = convertVecToRad(vec);
        for (unsigned int i = 0; i < dim; ++i)
            errorCodeJointPos[i]=simxSetJointPosition(m_clientId,m_jointHandles[i],pos[i],simx_opmode_oneshot_wait);
        usleep(2000000);

        for (int i = 0; i < dim; ++i)
            printf("Error code Joint Position%d %d\n",i,errorCodeJointPos[i]);

        simxUChar collisionState;
        for (unsigned int i = 0; i < dim; ++i) {
            simxReadCollision(m_clientId, m_collisionHandle[i], &collisionState, simx_opmode_oneshot_wait);
            printf("Error code Jaco link collision %d %d\n",i,collisionState);
        }
        //collisionHandle = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco", &m_jacoHandle, simx_opmode_oneshot_wait);
        //simxReadCollision(m_clientId, collisionHandle, &collisionState[0], simx_opmode_oneshot_wait);
        //printf("Error code Jaco collision  %d\n",collisionState[0]);


        return true;
    }
    else {
        return false;
    }
}

template<unsigned int dim>
Vec<dim, simxFloat> VrepHelper<dim>::convertVecToRad(const Vec<dim, float> &vec) {
    Vec<dim, simxFloat> rads;
    for (unsigned int i = 0; i < dim; ++i) {
        rads[i] = vec[i]*M_PI/180;
    }
    return rads;
}

#endif /* VREPHELPER_H_ */
