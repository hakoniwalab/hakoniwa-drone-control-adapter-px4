#pragma once

#include "hakoniwa/drone/control_adapter/px4_velocity_filter_config.hpp"

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/math.hpp>

namespace hakoniwa::drone::control_adapter {

// The velocity and acceleration PX4 hands its PositionControl
// (MulticopterPositionControl::set_vehicle_states): velocity through AlphaFilter (MPC_VEL_LP), the
// acceleration as the derivative of that filtered velocity through AlphaFilter (MPC_VELD_LP). PX4
// does not use the accelerometer here. MPC_VEL_NF_FRQ (notch, PX4 default 0) is not reproduced.
// Configured for the stage's sample rate on the first sample and whenever it changes; the first
// sample starts the velocity filter at its value and the derivative at 0.
class Px4VelocityStateFilter {
public:
    void configure(const Px4VelocityFilterConfig& config) { config_ = config; started_ = false; }
    void reset() { started_ = false; }

    // velocity in, filtered velocity and its filtered derivative out (NED, per axis).
    void apply(matrix::Vector3f& velocity, matrix::Vector3f& acceleration, float dt_s)
    {
        if (!(dt_s > 0.0f)) {
            return;
        }
        const float sample_rate_hz = 1.0f / dt_s;
        if (!started_ || std::fabs(sample_rate_hz - sample_rate_hz_) > 1.0f) {
            for (int axis = 0; axis < 3; ++axis) {
                set_cutoff(velocity_lp_[axis], sample_rate_hz, config_.velocity_cutoff_hz);
                set_cutoff(derivative_lp_[axis], sample_rate_hz, config_.velocity_derivative_cutoff_hz);
                if (!started_) {
                    velocity_lp_[axis].reset(velocity(axis));
                    derivative_lp_[axis].reset(0.0f);
                }
            }
            sample_rate_hz_ = sample_rate_hz;
            started_ = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            const float previous = velocity_lp_[axis].getState();
            velocity(axis) = velocity_lp_[axis].update(velocity(axis));
            acceleration(axis) = derivative_lp_[axis].update((velocity_lp_[axis].getState() - previous) / dt_s);
        }
    }

private:
    static void set_cutoff(AlphaFilter<float>& filter, float sample_rate_hz, double cutoff_hz)
    {
        if (!(cutoff_hz > 0.0) || !filter.setCutoffFreq(sample_rate_hz, static_cast<float>(cutoff_hz))) {
            filter.setAlpha(1.f);  // disabled, as PX4
        }
    }

    Px4VelocityFilterConfig config_{};
    AlphaFilter<float> velocity_lp_[3]{};
    AlphaFilter<float> derivative_lp_[3]{};
    float sample_rate_hz_{0.0f};
    bool started_{false};
};

}  // namespace hakoniwa::drone::control_adapter
