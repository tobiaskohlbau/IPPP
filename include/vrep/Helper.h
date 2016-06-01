#ifndef HELPER_H_
#define HELPER_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string>
#include <cstdint>
#include <memory>
#include <iostream>

#include <core/Vec.hpp>

extern "C" {
    #include <vrep/extApi.h>
    #include <vrep/extApiPlatform.h>
    #include <vrep/v_repConst.h>
}

namespace rmpl {

class Helper
{
public:
    Helper(const unsigned int &dim);
    void start();
    bool setPos(const Vec<float> &vec);

    bool checkCollision(const Vec<float> &jointAngles);


private:
    Vec<simxFloat> convertVecToRad(const Vec<float> &vec); // convert to simxFloat array and radiant

    unsigned int m_dim;
    simxInt m_clientId;
    simxInt m_jacoHandle;
    Vec<simxInt> m_jointHandles;
    Vec<simxInt> m_linkHandles;
    Vec<simxInt> m_collisionHandle;
};

} /* namespace rmpl */

#endif /* HELPER_H_ */
