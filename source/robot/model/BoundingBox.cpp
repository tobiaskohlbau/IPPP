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

#include <robot/model/BoundingBox.h>

#include <core/utility/Logging.h>

namespace rmpl {

BoundingBox::BoundingBox() {
    m_minBoundary = util::NaNVector<3>();
    m_maxBoundary = util::NaNVector<3>();
}

BoundingBox::BoundingBox(const Vector3 &minBoundary, const Vector3 &maxBoundary) {
    if (util::empty(minBoundary) || util::empty(maxBoundary)) {
        Logging::error("Empty Boundary", "BoundingBox");
    }
    m_minBoundary = minBoundary;
    m_maxBoundary = maxBoundary;
}

Vector3 BoundingBox::getMinBoundary() const {
    return m_minBoundary;
}

Vector3 BoundingBox::getMaxBoundary() const {
    return m_maxBoundary;
}

float BoundingBox::getDiameter() const {
    return (m_maxBoundary - m_minBoundary).norm();
}

bool BoundingBox::intersect(const BoundingBox &box) {
    Vector3 min = box.getMinBoundary();
    Vector3 max = box.getMaxBoundary();
    for (int i = 0; i < 3; ++i) {
        if (m_maxBoundary[i] < min[i]) {
            return false;
        } else if (m_minBoundary[i] > max[i]) {
            return false;
        }
    }
    return true;
}

} /* namespace rmpl */