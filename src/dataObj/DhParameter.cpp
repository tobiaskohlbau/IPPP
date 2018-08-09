//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#include <ippp/dataObj/DhParameter.h>

#include <ippp/util/Logging.h>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

DhParameter::DhParameter(double alphaAngle, double aOffset, double dOffset, double thetaAngle) {
    if (-util::twoPi() <= alphaAngle && alphaAngle <= util::twoPi())
        alpha = alphaAngle;
    else
        Logging::error("Alpha angle has to be 0 < alpha < 2 * pi", "DhParameter");

    if (-util::twoPi() <= thetaAngle && thetaAngle <= util::twoPi())
        theta = thetaAngle;
    else
        Logging::error("Alpha angle has to be 0 < alpha < 2 * pi", "DhParameter");

    a = aOffset;
    d = dOffset;
}

} /* namespace ippp */
