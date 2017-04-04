//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <core/types.h>
#include <core/utility/UtilVec.hpp>

namespace rmpl {

class BoundingBox {
public:
    BoundingBox();
    BoundingBox(const Vector3 &minBoundary, const Vector3 &maxBoundary);

    Vector3 getMinBoundary() const;
    Vector3 getMaxBoundary() const;
    float getDiameter() const;
    bool intersect(const BoundingBox &box);

private:
    Vector3 m_minBoundary;
    Vector3 m_maxBoundary;
};

} /* namespace rmpl */

#endif //BOUNDINGBOX_H
