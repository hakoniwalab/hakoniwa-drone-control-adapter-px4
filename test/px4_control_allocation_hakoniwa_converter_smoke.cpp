#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_control_allocation_hakoniwa_converter.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

bool nearly_equal(double a, double b, double eps = 1e-9)
{
    return std::fabs(a - b) <= eps;
}

}  // namespace

int main()
{
    HakoniwaControlAllocationConverterInput input{};
    input.rotor_count = 4;
    input.rotor_model.thrust_coefficient = 2.0;
    input.rotor_model.torque_coefficient = 0.4;
    input.rotor_model.actuator_min = 0.0;
    input.rotor_model.actuator_max = 1.0;
    input.rotor_model.actuator_trim = 0.1;
    input.rotor_model.actuator_linearization_point = 0.2;

    input.rotors[0] = {1.0, 1.0, 0.0, 1.0};
    input.rotors[1] = {-1.0, 1.0, 0.0, -1.0};
    input.rotors[2] = {-1.0, -1.0, 0.0, 1.0};
    input.rotors[3] = {1.0, -1.0, 0.0, -1.0};

    const ControlAllocationInput converted = compose_control_allocation_input_from_hakoniwa(input);

    require(converted.actuator_count == 4, "unexpected converted actuator count");
    require(nearly_equal(converted.actuators[0].geometry.thrust_coefficient, 2.0), "unexpected converted Ct");
    require(nearly_equal(converted.actuators[0].geometry.moment_ratio, 0.2), "unexpected positive moment ratio");
    require(nearly_equal(converted.actuators[1].geometry.moment_ratio, -0.2), "unexpected negative moment ratio");
    require(nearly_equal(converted.actuators[0].geometry.axis.z, -1.0), "unexpected default axis");
    require(nearly_equal(converted.actuators[0].limit.max, 1.0), "unexpected actuator max");
    require(nearly_equal(converted.actuators[0].trim, 0.1), "unexpected actuator trim");
    require(nearly_equal(converted.actuators[0].linearization_point, 0.2), "unexpected actuator linearization point");
    require(nearly_equal(compute_px4_moment_ratio_from_hakoniwa(2.0, 0.4, -1.0), -0.2),
            "unexpected standalone moment ratio conversion");

    return EXIT_SUCCESS;
}
