#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

#include <Eigen/Core>

extern "C" {
#include "extApi.h"
#include "extApiPlatform.h"
#include "v_repConst.h"
}

#include <IPPP/core/Identifier.h>

namespace ippp {

class Helper : public Identifier {
  public:
    Helper(unsigned int dim);
    void start();
    bool setPos(const Eigen::VectorXf &vec);

  private:
    Eigen::VectorXf convertVecToRad(const Eigen::VectorXf &vec);    // convert to simxFloat array and radiant

    unsigned int m_dim;
    simxInt m_clientId;
    Eigen::VectorXi m_jointHandles;
    bool m_started;
};

} /* namespace ippp */

#endif /* HELPER_H */
