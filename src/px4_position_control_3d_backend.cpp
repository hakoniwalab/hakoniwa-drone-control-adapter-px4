#include "hakoniwa/drone/control_adapter/px4_position_control_3d_backend.hpp"
#include "px4_velocity_state_filter.hpp"

#include "PositionControl.hpp"
#include <motion_planning/HeadingSmoothing.hpp>
#include <motion_planning/PositionSmoothing.hpp>

#include <algorithm>
#include <random>
#include <cfloat>
#include <optional>
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
    // MulticopterPositionControl: the filtered velocity and its filtered derivative, not Drone PRO's acceleration.
    velocity_filter.apply(state.velocity, state.acceleration, f64(dt_sec > 0.0 ? dt_sec : 0.0));
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
    // update_interval_sec: time since start, of the last update (uORB SubscriptionInterval), and the
    // setpoint held until the next one.
    double time_sec{0.0};
    double last_update_sec{0.0};  // SubscriptionInterval's reference (advances by the interval)
    double last_run_sec{0.0};     // FlightModeManager's _time_stamp_last_loop (its dt)
    bool held_valid{false};
    float held_position[3]{};
    float held_velocity[3]{};
    float held_acceleration[3]{};
    float held_yaw{NAN};
    float held_yawspeed{NAN};
    std::optional<double> held_yaw_rate{};
    // navigator_delay_ms: the latest requested target, when it came, its delay, and the target in effect.
    matrix::Vector3f requested{NAN, NAN, NAN};
    std::optional<double> requested_yaw{};
    std::optional<double> effective_yaw{};
    double requested_sec{0.0};
    double request_delay_sec{0.0};
    std::mt19937 delay_rng{};
    // takeoff_profile: the takeoff in progress (FlightTaskAuto's takeoff type) and TakeoffHandling's state.
    bool takeoff_active{false};
    matrix::Vector3f takeoff_target{};
    float takeoff_ground_z{0.f};
    int takeoff_state{-1};            // -1 none, 0 spool-up, 1 ready, 2 ramp-up, 3 flight
    double takeoff_elapsed_sec{0.0};
    double takeoff_ramp_progress{0.0};
    bool takeoff_limits_restored{true};
    // flight-phase support: the caller sends phases (the takeoff is no longer inferred), and the landing's
    // shaped descent speed (down) and acceleration.
    bool phase_seen{false};
    bool landing{false};
    double land_velocity{0.0};
    double land_acceleration{0.0};
};

Px4PositionControl3DBackend::Px4PositionControl3DBackend(const Px4PositionControl3DBackendConfig& config)
    : config_(config)
    , controller_(new PositionControl())
    , velocity_filter_(new Px4VelocityStateFilter())
    , trajectory_(new TrajectoryState())
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
    trajectory_->started = false;
    trajectory_->time_sec = 0.0;
    trajectory_->last_update_sec = 0.0;
    trajectory_->last_run_sec = 0.0;
    trajectory_->delay_rng.seed(config_.trajectory.navigator_delay_seed);
    trajectory_->takeoff_active = false;
    trajectory_->takeoff_state = -1;
    trajectory_->takeoff_limits_restored = true;
    trajectory_->phase_seen = false;
    trajectory_->landing = false;
    trajectory_->land_velocity = 0.0;
    trajectory_->land_acceleration = 0.0;
    yaw_locked_ = false;
}

// FlightModeManager runs on every position update (each position-control cycle, position or velocity
// target alike), so its clock advances on every call of the stage.
void Px4PositionControl3DBackend::advance_trajectory_clock(double dt_sec)
{
    if (dt_sec > 0.0) {
        trajectory_->time_sec += dt_sec;
    }
}

PositionControl3DOutput Px4PositionControl3DBackend::run_position(
    const PositionControl3DPositionInput& requested,
    double dt_sec)
{
    PositionControl3DPositionInput input = requested;
    apply_landing(input, dt_sec > 0.0 ? dt_sec : 0.0);
    yaw_locked_ = false;  // a position target gives its own heading
    advance_trajectory_clock(dt_sec);
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
    output_yaw_rate = apply_trajectory(input, f64(dt_sec > 0.0 ? dt_sec : 0.0), setpoint);
    apply_takeoff(setpoint, dt_sec > 0.0 ? dt_sec : 0.0);
    controller_->setInputSetpoint(setpoint);

    const bool ok = controller_->update(f64(dt_sec > 0.0 ? dt_sec : 0.0));
    if (!ok) {
        return PositionControl3DOutput{};
    }

    return make_output(*controller_, config_.hover_thrust, output_yaw_rate);
}

// takeoff_profile: MulticopterPositionControl with TakeoffHandling (mc_pos_control/Takeoff). The takeoff
// starts armed (spool-up), is ready after COM_SPOOLUP_TIME, ramps up when the trajectory wants to climb
// (unsmoothed upward velocity above 0.3 m/s) and flies when the ramp is complete.
void Px4PositionControl3DBackend::apply_takeoff(trajectory_setpoint_s& setpoint, double dt_sec)
{
    auto& tr = *trajectory_;
    const auto& cfg = config_.trajectory;
    if (!cfg.takeoff_profile || tr.takeoff_state < 0) {
        return;
    }
    // TakeoffHandling::updateTakeoffState
    tr.takeoff_elapsed_sec += dt_sec;
    if (tr.takeoff_state == 0 && tr.takeoff_elapsed_sec + 1e-9 >= cfg.spoolup_sec) {
        tr.takeoff_state = 1;
    }
    if (tr.takeoff_state == 1 && tr.unsmoothed_velocity_z < -0.3f) {
        tr.takeoff_state = 2;
        tr.takeoff_ramp_progress = 0.0;
    }
    if (tr.takeoff_state == 2 && tr.takeoff_ramp_progress >= 1.0) {
        tr.takeoff_state = 3;
    }
    if (tr.takeoff_state == 3) {
        if (!tr.takeoff_limits_restored) {
            // Flying: the usual limits (MPC_THR_MIN, MPC_TILTMAX_AIR, the climb limit).
            controller_->setVelocityLimits(f64(config_.velocity_max_xy_mps), f64(config_.velocity_max_up_mps),
                                           f64(config_.velocity_max_down_mps));
            controller_->setThrustLimits(f64(config_.thrust_min), f64(config_.thrust_max));
            controller_->setTiltLimit(f64(config_.tilt_limit_rad));
            tr.takeoff_limits_restored = true;
        }
        return;
    }
    // TakeoffHandling::updateRamp: from -g / MPC_Z_VEL_P_ACC to the climb constraint (FlightTaskAuto's
    // speed_up, 1.2 x MPC_Z_V_AUTO_UP) over MPC_TKO_RAMP_T.
    const double vz_init = -9.80665 / std::fmax(config_.velocity_p_z, 0.01);
    const double desired = 1.2 * cfg.velocity_up_mps;
    double speed_up = vz_init;
    if (tr.takeoff_state == 2) {
        tr.takeoff_ramp_progress = (cfg.takeoff_ramp_sec > dt_sec)
            ? tr.takeoff_ramp_progress + dt_sec / cfg.takeoff_ramp_sec : 1.0;
        speed_up = (tr.takeoff_ramp_progress < 1.0)
            ? vz_init + tr.takeoff_ramp_progress * (desired - vz_init) : desired;
    }
    controller_->setVelocityLimits(f64(config_.velocity_max_xy_mps),
                                   f64(std::fmin(speed_up, config_.velocity_max_up_mps)),
                                   f64(config_.velocity_max_down_mps));
    controller_->setThrustLimits(0.f, f64(config_.thrust_max));  // ramping from zero thrust
    controller_->setTiltLimit(f64(cfg.tilt_max_land_rad));       // MPC_TILTMAX_LND until flight
    if (tr.takeoff_state == 2 && std::isfinite(setpoint.velocity[2])) {
        setpoint.acceleration[2] = NAN;  // the ramp is not amended by the acceleration feed-forward
    }
    if (tr.takeoff_state < 2) {
        // Not taken off: no corrections, no thrust (a high downward acceleration), no integrator windup.
        const float yaw = setpoint.yaw;
        const float yawspeed = setpoint.yawspeed;
        setpoint = PositionControl::empty_trajectory_setpoint;
        setpoint.acceleration[0] = 0.f;
        setpoint.acceleration[1] = 0.f;
        setpoint.acceleration[2] = 100.f;
        setpoint.yaw = yaw;
        setpoint.yawspeed = yawspeed;
        controller_->resetIntegral();
    }
}

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
    matrix::Vector3f target = to_px4_vector(input.target_position);
    std::optional<double> target_yaw = input.target_yaw_rad;
    if (!cfg.navigator_delay_ms.empty()) {
        // navigator_delay_ms: a new target reaches the trajectory after the navigator's delay.
        if (!trajectory.started) {
            trajectory.requested = target;
            trajectory.requested_yaw = target_yaw;
            trajectory.effective_yaw = target_yaw;
            trajectory.request_delay_sec = 0.0;
        }
        else if ((target - trajectory.requested).longerThan(1e-3f) || target_yaw != trajectory.requested_yaw) {
            trajectory.requested = target;
            trajectory.requested_yaw = target_yaw;
            trajectory.requested_sec = trajectory.time_sec;
            const auto& choices = cfg.navigator_delay_ms;
            const std::size_t index = choices.size() == 1 ? 0
                : std::uniform_int_distribution<std::size_t>(0, choices.size() - 1)(trajectory.delay_rng);
            trajectory.request_delay_sec = choices[index] / 1000.0;
        }
        if (trajectory.started && trajectory.time_sec - trajectory.requested_sec + 1e-9 < trajectory.request_delay_sec) {
            target = trajectory.target;  // the navigator has not published it yet
            target_yaw = trajectory.effective_yaw;
        }
        else {
            trajectory.effective_yaw = target_yaw;
        }
    }

    // FlightModeManager runs only when its position subscription interval has passed (uORB
    // SubscriptionInterval: the last update advances by the interval, kept within one interval of now).
    // The clock and the updates continue across targets and restarts (advance_trajectory_clock).
    float trajectory_dt = dt_sec;
    if (config_.trajectory.update_interval_sec > 0.0 && trajectory.started) {
        const double interval = config_.trajectory.update_interval_sec;
        // A new target waits for the next update too (FlightModeManager runs on the position, not on
        // the navigator's setpoint).
        if (trajectory.held_valid && trajectory.time_sec - trajectory.last_update_sec + 1e-9 < interval) {
            for (int axis = 0; axis < 3; ++axis) {
                setpoint.position[axis] = trajectory.held_position[axis];
                setpoint.velocity[axis] = trajectory.held_velocity[axis];
                setpoint.acceleration[axis] = trajectory.held_acceleration[axis];
            }
            if (target_yaw) {
                setpoint.yaw = trajectory.held_yaw;
                setpoint.yawspeed = trajectory.held_yawspeed;
            }
            return target_yaw ? trajectory.held_yaw_rate : input.target_yaw_rate_rad_sec;
        }
        trajectory_dt = static_cast<float>(trajectory.time_sec - trajectory.last_run_sec);
        trajectory.last_run_sec = trajectory.time_sec;
        trajectory.last_update_sec = std::clamp(trajectory.last_update_sec + interval,
                                                trajectory.time_sec - interval, trajectory.time_sec);
    }

    if (!trajectory.started && config_.trajectory.update_interval_sec > 0.0
        && trajectory.time_sec - trajectory.last_update_sec + 1e-9 < config_.trajectory.update_interval_sec) {
        // FlightModeManager activates the task on one of its updates: until the next update (the interval
        // since the last one, on the position updates that never stop), the target passes through.
        return input.target_yaw_rate_rad_sec;
    }
    bool start_takeoff = !trajectory.started && cfg.takeoff_profile && trajectory.takeoff_state < 0;
    bool phase_ends_takeoff = false;
    if (input.flight_phase != FlightPhase::Normal) {
        trajectory.phase_seen = true;
    }
    if (trajectory.phase_seen) {
        // The caller's phase: the takeoff starts on its request and ends when it asks for anything else.
        start_takeoff = start_takeoff && input.flight_phase == FlightPhase::Takeoff;
        phase_ends_takeoff = input.flight_phase != FlightPhase::Takeoff;
    }
    else
    {
        // takeoff_profile: a trajectory from rest under a target above it is a takeoff (navigator's takeoff).
        start_takeoff = start_takeoff && position(2) - target(2) > 0.5f && std::fabs(velocity(2)) < 0.2f;
    }
    if (start_takeoff) {
        trajectory.takeoff_active = true;
        trajectory.takeoff_target = target;
        trajectory.takeoff_ground_z = position(2);
        trajectory.takeoff_state = 0;
        trajectory.takeoff_elapsed_sec = 0.0;
        trajectory.takeoff_ramp_progress = 0.0;
        trajectory.takeoff_limits_restored = false;
    }
    else if (trajectory.takeoff_active && (phase_ends_takeoff || (target - trajectory.takeoff_target).longerThan(1e-3f))) {
        trajectory.takeoff_active = false;  // the next target is a position, not the takeoff
    }
    else if (trajectory.takeoff_active) {
        // navigator: the takeoff is complete within the altitude acceptance (NAV_MC_ALT_RAD; half the takeoff
        // height when that is smaller); the setpoint becomes a loiter at the same position.
        const float takeoff_height = trajectory.takeoff_ground_z - trajectory.takeoff_target(2);
        float acceptance = f64(cfg.vertical_acceptance_m);
        if (takeoff_height > 0.f && takeoff_height < acceptance) {
            acceptance = takeoff_height / 2.f;
        }
        if (trajectory.takeoff_ground_z - position(2) > takeoff_height - acceptance) {
            trajectory.takeoff_active = false;
        }
    }
    if (!trajectory.started) {
        // FlightTaskAuto::activate: start from the vehicle's state.
        trajectory.position.reset(matrix::Vector3f{0.f, 0.f, 0.f}, velocity, position);
        trajectory.heading.reset(f64(input.state.yaw_rad), 0.f);
        trajectory.previous_waypoint = position;
        trajectory.target = target;
        trajectory.unsmoothed_velocity_z = 0.f;
        trajectory.started = true;
        // The start is an update of FlightModeManager too: the update history continues from it (no reset
        // to 0; SubscriptionInterval keeps the reference within one interval of now).
        trajectory.last_run_sec = trajectory.time_sec;
        trajectory.last_update_sec = std::clamp(trajectory.last_update_sec + config_.trajectory.update_interval_sec,
                                                trajectory.time_sec - config_.trajectory.update_interval_sec,
                                                trajectory.time_sec);
        trajectory.held_valid = false;
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
    double cruise_speed_mps = cfg.cruise_speed_mps;
#ifdef HAKO_ENABLE_GOTO_SPEED_LIMIT
    if (input.horizontal_speed_limit_m_s
        && std::isfinite(*input.horizontal_speed_limit_m_s)
        && *input.horizontal_speed_limit_m_s > 0.0) {
        cruise_speed_mps = *input.horizontal_speed_limit_m_s;
    }
#endif
    smoothing.setCruiseSpeed(f64(std::fmin(cruise_speed_mps, config_.velocity_max_xy_mps)));
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
    if (trajectory.takeoff_active && trajectory.unsmoothed_velocity_z < 0.f
        && trajectory.takeoff_ground_z - position(2) < f64(cfg.land_alt1_m)) {
        // FlightTaskAuto::_updateTrajConstraints for a takeoff below MPC_LAND_ALT1.
        smoothing.setMaxVelocityZ(f64(cfg.takeoff_speed_mps));
        smoothing.setMaxAccelerationZ(std::fmin(f64(cfg.acceleration_up_mps2),
                                                f64(cfg.takeoff_speed_mps / cfg.takeoff_ramp_sec)));
        smoothing.forceSetPosition(matrix::Vector3f{NAN, NAN, position(2)});
    }

    const matrix::Vector3f waypoints[3] = {trajectory.previous_waypoint, trajectory.target, trajectory.target};
    const matrix::Vector3f feedforward_velocity{NAN, NAN, NAN};
    PositionSmoothing::PositionSmoothingSetpoints smoothed{};
    smoothing.generateSetpoints(position, waypoints, feedforward_velocity, trajectory_dt, false, smoothed);
    trajectory.unsmoothed_velocity_z = smoothed.unsmoothed_velocity(2);
    if (trajectory.takeoff_active && trajectory.takeoff_state >= 0 && trajectory.takeoff_state < 2) {
        // FlightModeManager: before the takeoff ramp-up the task is re-activated after each update
        // (FlightTaskAuto::reActivate: zero acceleration, 0.7 m/s down, the vehicle's position).
        smoothing.reset(matrix::Vector3f{0.f, 0.f, 0.f}, matrix::Vector3f{0.f, 0.f, 0.7f}, position);
    }

    for (int axis = 0; axis < 3; ++axis) {
        setpoint.position[axis] = smoothed.position(axis);
        setpoint.velocity[axis] = smoothed.velocity(axis);
        setpoint.acceleration[axis] = smoothed.acceleration(axis);
        trajectory.held_position[axis] = setpoint.position[axis];
        trajectory.held_velocity[axis] = setpoint.velocity[axis];
        trajectory.held_acceleration[axis] = setpoint.acceleration[axis];
    }
    trajectory.held_valid = true;

    // FlightTaskAuto::_smoothYaw: only a yaw target is smoothed; a yaw-rate command passes through.
    if (!target_yaw) {
        trajectory.heading.reset(f64(input.state.yaw_rad), 0.f);
        trajectory.held_yaw_rate = input.target_yaw_rate_rad_sec;
        return input.target_yaw_rate_rad_sec;
    }
    trajectory.heading.setMaxHeadingRate(f64(cfg.yaw_rate_max_deg_s * M_PI / 180.0));
    trajectory.heading.setMaxHeadingAccel(f64(cfg.yaw_acceleration_max_deg_s2 * M_PI / 180.0));
    trajectory.heading.update(f64(*target_yaw), trajectory_dt);
    setpoint.yaw = trajectory.heading.getSmoothedHeading();
    setpoint.yawspeed = trajectory.heading.getSmoothedHeadingRate();
    trajectory.held_yaw = setpoint.yaw;
    trajectory.held_yawspeed = setpoint.yawspeed;
    trajectory.held_yaw_rate = static_cast<double>(setpoint.yawspeed);
    return trajectory.held_yaw_rate;
}

// landing_profile: FlightTaskAuto::_prepareLandSetpoints. The horizontal target is held and the vertical
// axis follows a descent speed (the altitude setpoint at the vehicle's), interpolated between MPC_LAND_SPEED
// at MPC_LAND_ALT2 and MPC_Z_V_AUTO_DN at MPC_LAND_ALT1 by the height above the takeoff point and shaped as
// PX4's trajectory: the acceleration moves toward the time-optimal one for the remaining speed change within
// the jerk, bounded by the acceleration limit.
void Px4PositionControl3DBackend::apply_landing(PositionControl3DPositionInput& input, double dt_sec)
{
    auto& tr = *trajectory_;
    const auto& cfg = config_.trajectory;
    if (input.flight_phase != FlightPhase::Landing || !cfg.landing_profile) {
        tr.landing = false;
        return;
    }
    if (!tr.landing) {
        tr.landing = true;
        tr.land_velocity = 0.0;
        tr.land_acceleration = 0.0;
        dt_sec = 0.0;
    }
    const double ground_z = (tr.takeoff_state >= 0) ? static_cast<double>(tr.takeoff_ground_z) : 0.0;
    const double h = ground_z - input.state.position.z;
    const double t = (cfg.land_alt1_m > cfg.land_alt2_m)
        ? std::clamp((h - cfg.land_alt2_m) / (cfg.land_alt1_m - cfg.land_alt2_m), 0.0, 1.0)
        : (h > cfg.land_alt2_m ? 1.0 : 0.0);
    const double target = cfg.land_speed_mps + t * (cfg.velocity_down_mps - cfg.land_speed_mps);
    const double dv = target - tr.land_velocity;
    const double a_goal = std::clamp(std::copysign(std::sqrt(2.0 * cfg.jerk_mps3 * std::fabs(dv)), dv),
                                     -cfg.acceleration_down_mps2, cfg.acceleration_down_mps2);
    tr.land_acceleration += std::clamp(a_goal - tr.land_acceleration, -cfg.jerk_mps3 * dt_sec, cfg.jerk_mps3 * dt_sec);
    tr.land_velocity += tr.land_acceleration * dt_sec;
    input.target_position.z = input.state.position.z;
    input.feedforward_velocity = Vector3D{0.0, 0.0, tr.land_velocity};
    // PX4 sends the ramp's acceleration with the descent speed (trajectory_setpoint.acceleration[2], up to
    // about 1.4 m/s^2 at the start of the descent). Without it a heavy vehicle lagged the descent and touched
    // down 0.2 s later than PX4 SITL (x500 0.09 s, 2026-10-17).
    input.feedforward_acceleration = Vector3D{0.0, 0.0, tr.land_acceleration};
}

PositionControl3DOutput Px4PositionControl3DBackend::run_velocity(
    const PositionControl3DVelocityInput& input,
    double dt_sec)
{
    trajectory_->started = false;  // a later position target starts its trajectory from the vehicle
    advance_trajectory_clock(dt_sec);
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
