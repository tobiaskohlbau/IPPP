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


#include <ippp/statistic/StatsPropertyCollector.h>
#include <ippp/types.h>

namespace ippp {

struct ParamsMA {
    bool useObstacle = false;
    bool useConstraint = false;
    bool optimize = false;

    std::string seed;
    double stepSize = 1;
    SamplerType samplerType = SamplerType::Uniform;
    SamplingType samplingType = SamplingType::Straight;
    PlannerType plannerType = PlannerType::RRT;
    PathModifierType pathModifier = PathModifierType::Dummy;
    RobotType robotType = RobotType::Point2D;
};

class ConfigurationMA {
  public:
    ConfigurationMA(RobotType robotType, bool useObstacle, bool useConstraint, bool isMobile);

    ParamsMA getParams();
    std::vector<ParamsMA> getParamsList();
    size_t numParams();
    void updatePropertyStats(size_t index);

    static std::vector<std::string> getSeeds();
    static std::vector<double> getMobileStepSizes();
    static std::vector<double> getSerialStepSizes();

  private:
    std::vector<ParamsMA> m_paramsMA;
    size_t m_index = 0;
    std::shared_ptr<StatsPropertyCollector> m_propertyCollector = nullptr;
};

} /* namespace ippp */
