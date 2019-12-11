#include <boost/test/unit_test.hpp>
#include <base/Temperature.hpp>

using namespace std;
using namespace base;

namespace utf = boost::unit_test;

BOOST_AUTO_TEST_SUITE(TemperatureTests)

BOOST_AUTO_TEST_CASE(it_creates_a_temperature_from_its_value_in_celsius,
                     * utf::tolerance(0.01))
{
    auto temp = Temperature::fromCelsius(10);
    BOOST_TEST(temp.getKelvin() == 283.15);
}

BOOST_AUTO_TEST_CASE(it_creates_a_temperature_from_its_value_in_kelvin)
{
    auto temp = Temperature::fromKelvin(10);
    BOOST_TEST(temp.getKelvin() == 10);
}

BOOST_AUTO_TEST_CASE(it_converts_a_temperature_to_a_plain_value_in_celsius)
{
    auto temp = Temperature::fromKelvin(10);
    BOOST_TEST(temp.getCelsius() == -263.15);
}

BOOST_AUTO_TEST_CASE(it_multiplies_a_temperature_by_a_scalar)
{
    auto temp = Temperature::fromKelvin(10);
    BOOST_TEST((temp * 5).getKelvin() == 50);
    BOOST_TEST((5 * temp).getKelvin() == 50);
}

BOOST_AUTO_TEST_CASE(it_adds_temperatures)
{
    auto temp0 = Temperature::fromKelvin(10);
    auto temp1 = Temperature::fromKelvin(20);
    BOOST_TEST((temp0 + temp1).getKelvin() == 30);
}

BOOST_AUTO_TEST_CASE(it_compares_temperatures_for_equality)
{
    auto temp = Temperature::fromKelvin(10);
    BOOST_TEST(temp == Temperature::fromKelvin(10));
    BOOST_TEST(!(temp == Temperature::fromKelvin(20)));
}

BOOST_AUTO_TEST_CASE(it_determines_if_a_temperature_is_less_than_another)
{
    auto temp0 = Temperature::fromKelvin(10);
    auto temp1 = Temperature::fromKelvin(20);
    BOOST_TEST(temp0 < temp1);
    BOOST_TEST(!(temp1 < temp0));
    BOOST_TEST(!(temp0 < temp0));
}

BOOST_AUTO_TEST_CASE(it_determines_if_a_temperature_is_greater_than_another)
{
    auto temp0 = Temperature::fromKelvin(10);
    auto temp1 = Temperature::fromKelvin(20);
    BOOST_TEST(temp1 > temp0);
    BOOST_TEST(!(temp0 > temp1));
    BOOST_TEST(!(temp0 > temp0));
}

BOOST_AUTO_TEST_CASE(it_assigns_temperatures)
{
    auto temp0 = Temperature::fromKelvin(10);
    auto temp1 = Temperature::fromKelvin(20);
    temp0 = temp1;
    BOOST_TEST(temp0.getKelvin() == 20);
}

BOOST_AUTO_TEST_CASE(it_determines_if_two_temperatures_are_equal_by_a_given_tolerance)
{
    auto temp = Temperature::fromKelvin(10);
    BOOST_TEST(!temp.isApprox(Temperature::fromKelvin(11)));
    BOOST_TEST(temp.isApprox(Temperature::fromKelvin(10.0 + 1e-6)));
}

BOOST_AUTO_TEST_CASE(it_allows_to_specify_the_tolerance)
{
    auto temp = Temperature::fromKelvin(10);
    BOOST_TEST(!temp.isApprox(Temperature::fromKelvin(11), 1));
    BOOST_TEST(temp.isApprox(Temperature::fromKelvin(10.0 + 0.99999), 1));
}

BOOST_AUTO_TEST_CASE(it_reports_if_a_temperature_is_in_a_range)
{
    auto min = Temperature::fromKelvin(10);
    auto max = Temperature::fromKelvin(100);
    auto temp = Temperature::fromKelvin(50);
    BOOST_TEST(temp.isInRange(min, max));
}

BOOST_AUTO_TEST_CASE(it_handles_the_range_first_argument_being_the_highest_temperature)
{
    auto min = Temperature::fromKelvin(10);
    auto max = Temperature::fromKelvin(100);
    auto temp = Temperature::fromKelvin(50);
    BOOST_TEST(temp.isInRange(max, min));
}

BOOST_AUTO_TEST_CASE(it_reports_if_a_temperature_is_below_the_min_bound_of_the_range)
{
    auto min = Temperature::fromKelvin(10);
    auto max = Temperature::fromKelvin(100);
    auto temp = Temperature::fromKelvin(5);
    BOOST_TEST(!temp.isInRange(max, min));
}

BOOST_AUTO_TEST_CASE(it_reports_if_a_temperature_is_above_the_max_bound_of_the_range)
{
    auto min = Temperature::fromKelvin(10);
    auto max = Temperature::fromKelvin(100);
    auto temp = Temperature::fromKelvin(105);
    BOOST_TEST(!temp.isInRange(max, min));
}

BOOST_AUTO_TEST_CASE(it_formats_a_temperature_on_a_ostream)
{
    ostringstream os;
    os << Temperature::fromCelsius(32.254);
    BOOST_TEST(os.str() == "[32.3 celsius]");
}

BOOST_AUTO_TEST_SUITE_END()
