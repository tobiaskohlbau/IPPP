#ifndef HELPER_H_
#define HELPER_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

extern "C" {
    #include <vrep/extApi.h>
    #include <vrep/extApiPlatform.h>
    #include <vrep/v_repConst.h>
}

#include <core/Base.h>
#include <core/Vec.hpp>

namespace rmpl {

class Helper : public Base
{
public:
    Helper(const unsigned int &dim);
    void start();
    bool setPos(const Vec<float> &vec);
    bool checkCollision(const Vec<float> &jointAngles);
    bool checkCollision(const std::vector<Vec<float>> &jointAngles);

private:
    Vec<simxFloat> convertVecToRad(const Vec<float> &vec); // convert to simxFloat array and radiant

    unsigned int m_dim;
    simxInt m_clientId;
    simxInt m_jacoHandle;
    Vec<simxInt> m_jointHandles;
    bool m_started;
};

} /* namespace rmpl */

#endif /* HELPER_H_ */
