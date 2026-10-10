#include "hakoniwa/drone/control_adapter/ekf_adapter.hpp"

#define private public
#include "hakoniwa/drone/control_adapter/px4_ekf_adapter.hpp"
#undef private

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

bool nearly_equal(double lhs, double rhs)
{
    return std::fabs(lhs - rhs) <= 1e-9;
}

}  // namespace

int main()
{
    Px4EkfAdapter adapter;
    require(nearly_equal(adapter.gps_delay_ms_, 0.0),
        "not calling set_sensor_delays_ms must keep GPS delay at zero");
    require(nearly_equal(adapter.baro_delay_ms_, 0.0),
        "not calling set_sensor_delays_ms must keep barometer delay at zero");

    adapter.set_sensor_delays_ms(100.0, 60.0);
    require(nearly_equal(adapter.gps_delay_ms_, 100.0), "GPS delay was not retained");
    require(nearly_equal(adapter.baro_delay_ms_, 60.0), "barometer delay was not retained");

    adapter.set_sensor_delays_ms(-10.0, -20.0);
    require(nearly_equal(adapter.gps_delay_ms_, 0.0), "negative GPS delay must clamp to zero");
    require(nearly_equal(adapter.baro_delay_ms_, 0.0), "negative barometer delay must clamp to zero");
    return EXIT_SUCCESS;
}
