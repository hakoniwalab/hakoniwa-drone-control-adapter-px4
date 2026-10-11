#include "hakoniwa/drone/control_adapter/px4_position_control_3d_backend.hpp"
#include "px4_velocity_state_filter.hpp"

#include "PositionControl.hpp"
#ifdef HAKO_EKF_IMU_ACCELERATION
#include <motion_planning/HeadingSmoothing.hpp>
#include <motion_planning/PositionSmoothing.hpp>
#endif

#include <cfloat>
#include <cmath>

namespace hakoniwa::drone::control_adapter {

namespace {

constexpr float kMinHoverThrust = 1e-6f;

float f64(double value)
{
    return static_cast<float>(value);
}

matrix::Vector3f to_px4_vector(const Vector3D& value)
{
    return matrix::Vector3f{f64(value.x), f64(value.y), f64(value.z)};
}

AttitudeQuaternion to_adapter_quaternion(const vehicle_attitude_setpoint_s& attitude_setpoint)
{
    return AttitudeQuaternion{
        attitude_setpoint.q_d[0],
        attitude_setpoint.q_d[1],
        attitude_setpoint.q_d[2],
        attitude_setpoint.q_d[3]
    };
}

NormalizedVerticalThrustCommand px4_body_thrust_to_adapter(float px4_thrust_body_z, float hover_thrust)
{
    if (!std::isfinite(px4_thrust_body_z) || !std::isfinite(hover_thrust) || std::fabs(hover_thrust) <= kMinHoverThrust) {
        return NormalizedVerticalThrustCommand{};
    }

    // Existing adapter thrust contract normalizes hover to approximately -1.0.
    // PX4 attitude thrust is normalized around hover=-MPC_THR_HOVER.
    return NormalizedVerticalThrustCommand{
        px4_thrust_body_z / hover_thrust
    };
}

void apply_common_state(PositionControl& controller, const PositionControl3DState& input_state,
                        Px4VelocityStateFilter& velocity_filter, double dt_sec)
{
    PositionControlStates state{};
    state.position = to_px4_vector(input_state.position);
    state.velocity = to_px4_vector(input_state.velocity);
    state.acceleration = to_px4_vector(input_state.acceleration);
#ifdef HAKO_EKF_IMU_ACCELERATION
    // MulticopterPositionControl: the filtered velocity and its filtered derivative, not Drone PRO's acceleration.
    velocity_filter.apply(state.velocity, state.acceleration, f64(dt_sec > 0.0 ? dt_sec : 0.0));
#else
    (void)velocity_filter;
    (void)dt_sec;
#endif
    state.yaw = f64(input_state.yaw_rad);
    controller.setState(state);
}

void apply_common_yaw_setpoint(
    trajectory_setpoint_s& setpoint,
    const PositionControl3DState& state,
    const std::optional<double>& target_yaw_rad,
    const std::optional<double>& target_yaw_rate_rad_sec)
{
    setpoint.yaw = f64(target_yaw_rad.value_or(state.yaw_rad));
    setpoint.yawspeed = f64(target_yaw_rate_rad_sec.value_or(0.0));
}

PositionControl3DOutput make_output(
    PositionControl& controller,
    double hover_thrust,
    const std::optional<double>& target_yaw_rate_rad_sec)
{
    vehicle_attitude_setpoint_s attitude_setpoint{};
    controller.getAttitudeSetpoint(attitude_setpoint);

    vehicle_local_position_setpoint_s local_position_setpoint{};
    controller.getLocalPositionSetpoint(local_position_setpoint);

    PositionControl3DOutput output{};
    output.target_attitude = to_adapter_quaternion(attitude_setpoint);
    output.thrust = px4_body_thrust_to_adapter(attitude_setpoint.thrust_body[2], f64(hover_thrust));
    output.target_yaw_rate_rad_sec = target_yaw_rate_rad_sec;
    output.debug_thrust_sp = Vector3D{
        local_position_setpoint.thrust[0],
        local_position_setpoint.thrust[1],
        local_position_setpoint.thrust[2]
    };
    return output;
}

}  // namespace

#ifdef HAKO_EKF_IMU_ACCELERATION
// PX4 SITL flies a position target (DO_REPOSITION) through FlightTaskAuto: PositionSmoothing turns the
// target into a jerk-limited position/velocity/acceleration setpoint from the vehicle's position at
// the command (navigator: previous = current position, next = current), and HeadingSmoothing limits
// the yaw. Without it the controller steps straight to the target (more tilt, a shorter leg).
struct Px4PositionControl3DBackend::TrajectoryState {
    PositionSmoothing position;
    HeadingSmoothing heading;
    bool started{false};
    matrix::Vector3f previous_waypoint{};
    matrix::Vector3f target{};
    float unsmoothed_velocity_z{0.f};
};
#else
struct Px4PositionControl3DBackend::TrajectoryState {};
#endif

Px4PositionControl3DBackend::Px4PositionControl3DBackend(const Px4PositionControl3DBackendConfig& config)
    : config_(config)
    , controller_(new PositionControl())
    , velocity_filter_(new Px4VelocityStateFilter())
#ifdef HAKO_EKF_IMU_ACCELERATION
    , trajectory_(new TrajectoryState())
#endif
{
    apply_config();
    reset();
}

Px4PositionControl3DBackend::~Px4PositionControl3DBackend()
{
    delete controller_;
    delete velocity_filter_;
    delete trajectory_;
}

void Px4PositionControl3DBackend::reset()
{
    controller_->resetIntegral();
    velocity_filter_->reset();
#ifdef HAKO_EKF_IMU_ACCELERATION
    trajectory_->started = false;
#endif
    yaw_locked_ = false;
}

PositionControl3DOutput Px4PositionControl3DBackend::run_position(
    const PositionControl3DPositionInput& input,
    double dt_sec)
{
    yaw_locked_ = false;  // a position target gives its own heading
    apply_common_state(*controller_, input.state, *velocity_filter_, dt_sec);

    trajectory_setpoint_s setpoint = PositionControl::empty_trajectory_setpoint;
    setpoint.position[0] = f64(input.target_position.x);
    setpoint.position[1] = f64(input.target_position.y);
    setpoint.position[2] = f64(input.target_position.z);

    if (input.feedforward_velocity) {
        setpoint.velocity[0] = f64(input.feedforward_velocity->x);
        setpoint.velocity[1] = f64(input.feedforward_velocity->y);
        setpoint.velocity[2] = f64(input.feedforward_velocity->z);
    }

    if (input.feedforward_acceleration) {
        setpoint.acceleration[0] = f64(input.feedforward_acceleration->x);
        setpoint.acceleration[1] = f64(input.feedforward_acceleration->y);
        setpoint.acceleration[2] = f64(input.feedforward_acceleration->z);
    }

    apply_common_yaw_setpoint(setpoint, input.state, input.target_yaw_rad, input.target_yaw_rate_rad_sec);
    std::optional<double> output_yaw_rate = input.target_yaw_rate_rad_sec;
#ifdef HAKO_EKF_IMU_ACCELERATION
    output_yaw_rate = apply_trajectory(input, f64(dt_sec > 0.0 ? dt_sec : 0.0), setpoint);
#endif
    controller_->setInputSetpoint(setpoint);

    const bool ok = controller_->update(f64(dt_sec > 0.0 ? dt_sec : 0.0));
    if (!ok) {
        return PositionControl3DOutput{};
    }

    return make_output(*controller_, config_.hover_thrust, output_yaw_rate);
}

#ifdef HAKO_EKF_IMU_ACCELERATION
std::optional<double> Px4PositionControl3DBackend::apply_trajectory(
    const PositionControl3DPositionInput& input,
    float dt_sec,
    trajectory_setpoint_s& setpoint)
{
    auto& trajectory = *trajectory_;
    const bool target_is_position = std::isfinite(input.target_position.x)
        && std::isfinite(input.target_position.y) && std::isfinite(input.target_position.z);
    if (!config_.trajectory.enabled || !target_is_position
        || input.feedforward_velocity || input.feedforward_acceleration) {
        // Trajectory off, or not a plain position target (an axis left free, or the caller shapes
        // its own trajectory with a feedforward): pass it through unchanged.
        trajectory.started = false;
        return input.target_yaw_rate_rad_sec;
    }
    const auto& cfg = config_.trajectory;
    const matrix::Vector3f position = to_px4_vector(input.state.position);
    const matrix::Vector3f velocity = to_px4_vector(input.state.velocity);
    const matrix::Vector3f target = to_px4_vector(input.target_position);

    if (!trajectory.started) {
        // FlightTaskAuto::activate: start from the vehicle's state.
        trajectory.position.reset(matrix::Vector3f{0.f, 0.f, 0.f}, velocity, position);
        trajectory.heading.reset(f64(input.state.yaw_rad), 0.f);
        trajectory.previous_waypoint = position;
        trajectory.target = target;
        trajectory.unsmoothed_velocity_z = 0.f;
        trajectory.started = true;
    }
    else if ((target - trajectory.target).longerThan(1e-3f)) {
        // A new reposition: navigator stores the vehicle's position as the previous waypoint.
        trajectory.previous_waypoint = position;
        trajectory.target = target;
    }

    // FlightTaskAuto::_updateTrajConstraints (no emergency braking, no takeoff ramp).
    auto& smoothing = trajectory.position;
    smoothing.setMaxAllowedHorizontalError(f64(cfg.max_horizontal_error_m));
    smoothing.setVerticalAcceptanceRadius(f64(cfg.vertical_acceptance_m));
    smoothing.setCruiseSpeed(f64(std::fmin(cfg.cruise_speed_mps, config_.velocity_max_xy_mps)));
    smoothing.setHorizontalTrajectoryGain(f64(cfg.trajectory_gain_xy));
    smoothing.setTargetAcceptanceRadius(f64(cfg.target_acceptance_m));
    smoothing.setMaxAccelerationXY(f64(cfg.acceleration_xy_mps2));
    smoothing.setMaxVelocityXY(f64(config_.velocity_max_xy_mps));
    smoothing.setMaxJerk(f64(cfg.jerk_mps3));
    if (trajectory.unsmoothed_velocity_z < 0.f) {  // up (NED)
        smoothing.setMaxVelocityZ(f64(cfg.velocity_up_mps));
        smoothing.setMaxAccelerationZ(f64(cfg.acceleration_up_mps2));
    }
    else {
        smoothing.setMaxAccelerationZ(f64(cfg.acceleration_down_mps2));
        smoothing.setMaxVelocityZ(f64(cfg.velocity_down_mps));
    }

    const matrix::Vector3f waypoints[3] = {trajectory.previous_waypoint, trajectory.target, trajectory.target};
    const matrix::Vector3f feedforward_velocity{NAN, NAN, NAN};
    PositionSmoothing::PositionSmoothingSetpoints smoothed{};
    smoothing.generateSetpoints(position, waypoints, feedforward_velocity, dt_sec, false, smoothed);
    trajectory.unsmoothed_velocity_z = smoothed.unsmoothed_velocity(2);

    for (int axis = 0; axis < 3; ++axis) {
        setpoint.position[axis] = smoothed.position(axis);
        setpoint.velocity[axis] = smoothed.velocity(axis);
        setpoint.acceleration[axis] = smoothed.acceleration(axis);
    }

    // FlightTaskAuto::_smoothYaw: only a yaw target is smoothed; a yaw-rate command passes through.
    if (!input.target_yaw_rad) {
        trajectory.heading.reset(f64(input.state.yaw_rad), 0.f);
        return input.target_yaw_rate_rad_sec;
    }
    trajectory.heading.setMaxHeadingRate(f64(cfg.yaw_rate_max_deg_s * M_PI / 180.0));
    trajectory.heading.setMaxHeadingAccel(f64(cfg.yaw_acceleration_max_deg_s2 * M_PI / 180.0));
    trajectory.heading.update(f64(*input.target_yaw_rad), dt_sec);
    setpoint.yaw = trajectory.heading.getSmoothedHeading();
    setpoint.yawspeed = trajectory.heading.getSmoothedHeadingRate();
    return static_cast<double>(setpoint.yawspeed);
}
#endif

PositionControl3DOutput Px4PositionControl3DBackend::run_velocity(
    const PositionControl3DVelocityInput& input,
    double dt_sec)
{
#ifdef HAKO_EKF_IMU_ACCELERATION
    trajectory_->started = false;  // a later position target starts its trajectory from the vehicle
#endif
    apply_common_state(*controller_, input.state, *velocity_filter_, dt_sec);

    trajectory_setpoint_s setpoint = PositionControl::empty_trajectory_setpoint;
    setpoint.velocity[0] = f64(input.target_velocity.x);
    setpoint.velocity[1] = f64(input.target_velocity.y);
    setpoint.velocity[2] = f64(input.target_velocity.z);

    if (input.feedforward_acceleration) {
        setpoint.acceleration[0] = f64(input.feedforward_acceleration->x);
        setpoint.acceleration[1] = f64(input.feedforward_acceleration->y);
        setpoint.acceleration[2] = f64(input.feedforward_acceleration->z);
    }

    apply_common_yaw_setpoint(setpoint, input.state, input.target_yaw_rad, input.target_yaw_rate_rad_sec);
#ifdef HAKO_EKF_IMU_ACCELERATION
    // PX4 StickYaw::updateYawLock: hold the heading while the yaw stick rests, free it while it turns.
    const double yaw_rate = input.target_yaw_rate_rad_sec.value_or(0.0);
    if (!config_.manual_yaw_lock || input.target_yaw_rad || std::fabs(yaw_rate) > FLT_EPSILON) {
        yaw_locked_ = false;
    }
    else {
        if (!yaw_locked_) {
            locked_yaw_rad_ = input.state.yaw_rad;
            yaw_locked_ = true;
        }
        setpoint.yaw = f64(locked_yaw_rad_);
    }
#endif
    controller_->setInputSetpoint(setpoint);

    const bool ok = controller_->update(f64(dt_sec > 0.0 ? dt_sec : 0.0));
    if (!ok) {
        return PositionControl3DOutput{};
    }

    return make_output(*controller_, config_.hover_thrust, input.target_yaw_rate_rad_sec);
}

void Px4PositionControl3DBackend::set_config(const Px4PositionControl3DBackendConfig& config)
{
    config_ = config;
    apply_config();
}

void Px4PositionControl3DBackend::apply_config()
{
    velocity_filter_->configure(config_.velocity_filter);
    controller_->setPositionGains(
        matrix::Vector3f{
            f64(config_.position_gain_xy),
            f64(config_.position_gain_xy),
            f64(config_.position_gain_z)
        });

    controller_->setVelocityGains(
        matrix::Vector3f{
            f64(config_.velocity_p_xy),
            f64(config_.velocity_p_xy),
            f64(config_.velocity_p_z)
        },
        matrix::Vector3f{
            f64(config_.velocity_i_xy),
            f64(config_.velocity_i_xy),
            f64(config_.velocity_i_z)
        },
        matrix::Vector3f{
            f64(config_.velocity_d_xy),
            f64(config_.velocity_d_xy),
            f64(config_.velocity_d_z)
        });

    controller_->setVelocityLimits(
        f64(config_.velocity_max_xy_mps),
        f64(config_.velocity_max_up_mps),
        f64(config_.velocity_max_down_mps));

    controller_->setThrustLimits(
        f64(config_.thrust_min),
        f64(config_.thrust_max));

    controller_->setHorizontalThrustMargin(f64(config_.horizontal_thrust_margin));
    controller_->setTiltLimit(f64(config_.tilt_limit_rad));
    controller_->setHoverThrust(f64(config_.hover_thrust));
    controller_->decoupleHorizontalAndVecticalAcceleration(config_.decouple_horizontal_and_vertical_acceleration);
}

}  // namespace hakoniwa::drone::control_adapter
