#define BOOST_TEST_MAIN

#define BOOST_TEST_MODULE Vec

#include <boost/test/unit_test.hpp>
#include <boost/test/test_case_template.hpp>
#include <boost/mpl/list.hpp>

#include <core/Vec.hpp>

using namespace rmpl;

BOOST_AUTO_TEST_SUITE( constructorFloatDouble )
typedef boost::mpl::list<float,double> test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE( standardConstructor, T, test_types ) {
    Vec<T> obj;
    BOOST_CHECK(obj.getDim() == 0);
    BOOST_CHECK_EQUAL(obj.empty(), true);
}

BOOST_AUTO_TEST_CASE_TEMPLATE( dimensionContructor, T, test_types ) {
    for (unsigned int i = 0; i < 10; ++i) {
        Vec<T> obj(i);
        BOOST_CHECK(obj.getDim() == i);
        BOOST_CHECK_EQUAL(obj.empty(), true);
    }
}

BOOST_AUTO_TEST_CASE_TEMPLATE( arrayConstructor, T, test_types ) {
    T array[10] = {0,1,2,3,4,5,6,7,8,9};
    Vec<T> obj(10, array);
    BOOST_CHECK(obj.getDim() == 10);
    BOOST_CHECK_EQUAL(obj.empty(), false);
    for (unsigned int i = 0; i < 10; ++i)
        BOOST_CHECK_EQUAL(obj[i], i);
}

BOOST_AUTO_TEST_SUITE_END()
