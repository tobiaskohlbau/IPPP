//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

#include <string>
#include <vector>

#include <ippp/ui/ModuleConfigurator.hpp>

namespace ippp {

enum class RRTType { normal, star, starConnect, Adapted, AdaptedConnect, CiBRRT };

struct ParamsMA {
    bool useObstacle = false;
    bool useConstraint = false;
    bool optimize = false;

    std::string seed;
    double stepSize = 1;
    SamplerType samplerType = SamplerType::Uniform;
    SamplingType samplingType = SamplingType::Straight;
    RRTType rrtType = RRTType::normal;
    PathModifierType pathModifier = PathModifierType::Dummy;
};

class ConfigurationMA {
  public:
      ConfigurationMA(bool useObstacle, bool useConstraint, bool isMobile);

      ParamsMA getParams();
      size_t numParams();

    static std::vector<std::string> getSeeds();
    static std::vector<double> getMobileStepSizes();
    static std::vector<double> getSerialStepSizes();

private:
    std::vector<ParamsMA> m_paramsMA;
    size_t m_index = 0;
};

} /* namespace ippp */
