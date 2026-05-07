#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

bool nearly_equal(double a, double b, double eps = 1e-9)
{
    return std::fabs(a - b) <= eps;
}

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

}  // namespace

int main()
{
    Px4ControllerConfigLoader loader;
    const Px4ControllerConfig config =
        loader.load_from_file("../config/px4-controller-config.sample.json");

    require(nearly_equal(config.runtime.altitude_hz, 1000.0), "unexpected altitude_hz");
    require(nearly_equal(config.runtime.attitude_hz, 1000.0), "unexpected attitude_hz");
    require(nearly_equal(config.runtime.horizontal_hz, 1000.0), "unexpected horizontal_hz");
    require(nearly_equal(config.runtime.rate_hz, 1000.0), "unexpected rate_hz");
    require(nearly_equal(config.altitude_control.position_gain_z, 10.0), "unexpected altitude pos p");
    require(nearly_equal(config.altitude_control.velocity_p_z, 15.0), "unexpected altitude vel p");
    require(nearly_equal(config.altitude_control.velocity_d_z, 10.0), "unexpected altitude vel d");
    require(nearly_equal(config.altitude_control.velocity_max_up_mps, 10.0), "unexpected altitude vel up");
    require(nearly_equal(config.altitude_control.hover_thrust, 0.5), "unexpected hover thrust");
    require(nearly_equal(config.altitude_control.thrust_max, 0.9), "unexpected thrust max");
    require(nearly_equal(config.attitude_control.proportional_gains.roll, 2.5), "unexpected attitude roll p");
    require(nearly_equal(config.attitude_control.proportional_gains.pitch, 2.5), "unexpected attitude pitch p");
    require(nearly_equal(config.attitude_control.proportional_gains.yaw, 0.1), "unexpected attitude yaw p");
    require(nearly_equal(config.attitude_control.yaw_weight, 0.4), "unexpected yaw weight");
    require(nearly_equal(config.attitude_control.rate_limits.roll_rad_sec, 314.1592653589793), "unexpected roll rate limit");
    require(nearly_equal(config.attitude_control.rate_limits.yaw_rad_sec, 31.41592653589793), "unexpected yaw rate limit");
    require(nearly_equal(config.horizontal_control.position_gain_xy, 6.0), "unexpected horizontal pos p");
    require(nearly_equal(config.horizontal_control.velocity_p_xy, 10.0), "unexpected horizontal vel p");
    require(nearly_equal(config.horizontal_control.velocity_d_xy, 0.1), "unexpected horizontal vel d");
    require(nearly_equal(config.horizontal_control.velocity_max_xy_mps, 20.0), "unexpected horizontal vel max");
    require(nearly_equal(config.horizontal_control.tilt_limit_rad, 0.2617993877991494), "unexpected tilt limit");
    require(nearly_equal(config.horizontal_control.horizontal_thrust_margin, 0.3), "unexpected horizontal thrust margin");
    require(nearly_equal(config.rate_control.gains.roll.p, 1.5), "unexpected roll p");
    require(nearly_equal(config.rate_control.gains.pitch.d, 0.02), "unexpected pitch d");
    require(nearly_equal(config.rate_control.gains.yaw.p, 0.452), "unexpected yaw p");
    require(nearly_equal(config.rate_control.feed_forward.roll, 0.0), "unexpected roll ff");
    require(nearly_equal(config.rate_control.integrator_limits.yaw_integrator, 0.2), "unexpected yaw int lim");

    std::cout << "loader smoke test passed" << std::endl;
    return EXIT_SUCCESS;
}
