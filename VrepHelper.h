#ifndef VREPHELPER_H_
#define VREPHELPER_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string>
#include <cstdint>
#include <memory>

#include "Vec.h"

extern "C" {
    #include <extApi.h>
    #include <extApiPlatform.h>
    #include <v_repConst.h>
}
#define NON_MATLAB_PARSING

class VrepHelper
{
public:
    VrepHelper(const unsigned int &dim);
    void startVrep();
    bool setPos(const Vec<float> &vec);

    bool isInCollision(const Vec<float> &jointAngles);
    bool isInCollision(const Vec<float> &qStart, const Vec<float> &qGoal);

private:
    unsigned int m_dim;
    Vec<simxFloat> convertVecToRad(const Vec<float> &vec); // convert to simxFloat array and radiant
    simxInt m_clientId;
    simxInt m_jacoHandle;
    Vec<simxInt> m_jointHandles;
    Vec<simxInt> m_linkHandles;
    Vec<simxInt> m_collisionHandle;
};

#endif /* VREPHELPER_H_ */
