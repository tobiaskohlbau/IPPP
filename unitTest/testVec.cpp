#define BOOST_TEST_MAIN

#define BOOST_TEST_MODULE Vec

#include <boost/test/unit_test.hpp>

#include <core/Vec.hpp>

using namespace rmpl;

BOOST_AUTO_TEST_SUITE( constructor )
BOOST_AUTO_TEST_CASE( constructor1 )
{
    Vec<float> obj;
    BOOST_CHECK(obj.getDim() == 0);
    BOOST_CHECK_EQUAL(obj.empty(), true);
}

BOOST_AUTO_TEST_CASE( constructorDimension )
{
    unsigned int i = 1;
    Vec<float> obj(i);
    BOOST_CHECK(obj.getDim() == i);
    BOOST_CHECK_EQUAL(obj.empty(), true);
    //Vec<float> obj((unsigned int) -1);
    //BOOST_CHECK(obj.getDim() == 0);
    //BOOST_CHECK_EQUAL(obj.empty(), true);
}

BOOST_AUTO_TEST_SUITE_END()
