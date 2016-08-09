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

#include <boost/mpl/list.hpp>
#include <boost/test/test_case_template.hpp>
#include <boost/test/unit_test.hpp>

#include <core/Vec.hpp>

using namespace rmpl;

BOOST_AUTO_TEST_SUITE(constructorFloatDouble)
typedef boost::mpl::list<float, double> testTypes;

BOOST_AUTO_TEST_CASE_TEMPLATE(standardConstructor, T, testTypes) {
    Vec<T> obj;
    BOOST_CHECK(obj.getDim() == 0);
    BOOST_CHECK_EQUAL(obj.empty(), true);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(dimensionConstructor, T, testTypes) {
    for (unsigned int i = 0; i < 10; ++i) {
        Vec<T> obj(i);
        BOOST_CHECK(obj.getDim() == i);
        BOOST_CHECK_EQUAL(obj.empty(), true);
    }
}

BOOST_AUTO_TEST_CASE_TEMPLATE(arrayConstructor, T, testTypes) {
    T array[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    Vec<T> obj(10, array);
    BOOST_CHECK(obj.getDim() == 10);
    BOOST_CHECK_EQUAL(obj.empty(), false);
    for (unsigned int i = 0; i < 10; ++i)
        BOOST_CHECK_EQUAL(obj[i], i);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(setAllTo, T, testTypes) {
    T array[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    Vec<T> obj(10, array);
    obj.setAllTo(0);
    for (unsigned int i = 0; i < 10; ++i)
        BOOST_CHECK_EQUAL(obj[i], 0);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(append, T, testTypes) {
    T array1[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    T array2[6] = {10, 11, 12, 13, 14, 15};
    Vec<T> obj1(10, array1);
    Vec<T> obj2(6, array2);
    obj1.append(obj2);
    for (unsigned int i = 0; i < obj1.getDim(); ++i)
        BOOST_CHECK_EQUAL(obj1[i], i);
    obj1.append(Vec<T>());    // append empty Vec
    for (unsigned int i = 0; i < obj1.getDim(); ++i)
        BOOST_CHECK_EQUAL(obj1[i], i);
}

BOOST_AUTO_TEST_SUITE_END()
