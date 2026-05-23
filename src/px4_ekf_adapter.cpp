#include "hakoniwa/drone/control_adapter/px4_ekf_adapter.hpp"

#include "EKF/common.h"
#include "EKF/ekf.h"

#include <limits>

namespace hakoniwa::drone::control_adapter {

namespace {

double deg_e7_to_deg(std::int32_t deg_e7)
{
    return static_cast<double>(deg_e7) * 1.0e-7;
}

double mm_to_m(std::int32_t mm)
{
    return static_cast<double>(mm) * 1.0e-3;
}

float cmps_to_mps(std::int16_t cmps)
{
    return static_cast<float>(cmps) * 1.0e-2f;
}

float clamp_non_negative(double value)
{
    return static_cast<float>((value < 0.0) ? 0.0 : value);
}

} // namespace

Px4EkfAdapter::Px4EkfAdapter(const EkfAdapterConfig& config)
    : config_(config)
{
    reset();
}

Px4EkfAdapter::~Px4EkfAdapter()
{
    delete ekf_;
    ekf_ = nullptr;
}

void Px4EkfAdapter::reset()
{
    delete ekf_;
    ekf_ = new ::Ekf();
    initialized_ = false;
}

void Px4EkfAdapter::set_config(const EkfAdapterConfig& config)
{
    config_ = config;
}

void Px4EkfAdapter::ensure_initialized(std::uint64_t time_usec)
{
    if (initialized_) {
        return;
    }

    ekf_->init(time_usec);
    apply_sensor_policy();
    ekf_->set_in_air_status(false);
    ekf_->set_vehicle_at_rest(true);
    initialized_ = true;
}

void Px4EkfAdapter::apply_sensor_policy()
{
    auto* fusion = ekf_->getFusionControlHandle();
    fusion->gps.available = true;
    fusion->gps.enabled = true;
    fusion->baro.available = true;
    fusion->baro.enabled = true;
    fusion->mag.available = true;
    fusion->mag.enabled = true;
}

void Px4EkfAdapter::push_hil_sensor(const EkfHilSensorInput& input, double dt_sec)
{
    ensure_initialized(input.time_usec);

    estimator::imuSample imu{};
    imu.time_us = input.time_usec;
    imu.delta_ang = matrix::Vector3f(
        static_cast<float>(input.xgyro_rad_s * dt_sec),
        static_cast<float>(input.ygyro_rad_s * dt_sec),
        static_cast<float>(input.zgyro_rad_s * dt_sec));
    imu.delta_vel = matrix::Vector3f(
        static_cast<float>(input.xacc_mps2 * dt_sec),
        static_cast<float>(input.yacc_mps2 * dt_sec),
        static_cast<float>(input.zacc_mps2 * dt_sec));
    imu.delta_ang_dt = static_cast<float>(dt_sec);
    imu.delta_vel_dt = static_cast<float>(dt_sec);
    ekf_->setIMUData(imu);

    estimator::magSample mag{};
    mag.time_us = input.time_usec;
    mag.mag = matrix::Vector3f(
        static_cast<float>(input.xmag_gauss),
        static_cast<float>(input.ymag_gauss),
        static_cast<float>(input.zmag_gauss));
    ekf_->setMagData(mag);

    estimator::baroSample baro{};
    baro.time_us = input.time_usec;
    baro.hgt = static_cast<float>(input.pressure_alt_m);
    ekf_->setBaroData(baro);
}

void Px4EkfAdapter::push_hil_gps(const EkfHilGpsInput& input)
{
    ensure_initialized(input.time_usec);

    estimator::gnssSample gps{};
    gps.time_us = input.time_usec;
    gps.lat = input.lat_deg;
    gps.lon = input.lon_deg;
    gps.alt = static_cast<float>(input.alt_m);
    gps.vel = matrix::Vector3f(
        static_cast<float>(input.vn_mps),
        static_cast<float>(input.ve_mps),
        static_cast<float>(input.vd_mps));
    gps.hacc = clamp_non_negative(input.eph_m);
    gps.vacc = clamp_non_negative(input.epv_m);
    gps.sacc = clamp_non_negative(config_.gps_quality.sacc_mps);
    gps.fix_type = static_cast<std::uint8_t>(input.fix_type);
    gps.nsats = static_cast<std::uint8_t>((input.satellites_visible < 0) ? 0 : input.satellites_visible);
    gps.pdop = 0.0f;
    gps.yaw = std::numeric_limits<float>::quiet_NaN();
    gps.yaw_acc = 0.0f;
    gps.yaw_offset = 0.0f;
    gps.spoofed = false;
    gps.jammed = false;
    gps.pos_body = matrix::Vector3f(0.f, 0.f, 0.f);
    ekf_->setGpsData(gps);
}

void Px4EkfAdapter::update()
{
    if (!initialized_) {
        return;
    }
    ekf_->update();
}

EkfEstimatedState Px4EkfAdapter::get_estimated_state() const
{
    EkfEstimatedState state{};
    if (!initialized_) {
        return state;
    }

    const auto quat = ekf_->getQuaternion();
    const auto vel = ekf_->getVelocity();
    const auto pos = ekf_->getPosition();
    const auto lla = ekf_->getLatLonAlt();

    state.attitude_quaternion_wxyz = {
        static_cast<double>(quat(0)),
        static_cast<double>(quat(1)),
        static_cast<double>(quat(2)),
        static_cast<double>(quat(3)),
    };
    state.velocity_ned_mps = {
        static_cast<double>(vel(0)),
        static_cast<double>(vel(1)),
        static_cast<double>(vel(2)),
    };
    state.position_local_ned_m = {
        static_cast<double>(pos(0)),
        static_cast<double>(pos(1)),
        static_cast<double>(pos(2)),
    };
    state.lat_deg = lla.latitude_deg();
    state.lon_deg = lla.longitude_deg();
    state.alt_m_amsl = static_cast<double>(lla.altitude());

    state.attitude_valid = ekf_->attitude_valid();
    state.local_position_valid = ekf_->isLocalHorizontalPositionValid() && ekf_->isLocalVerticalPositionValid();
    state.global_position_valid = ekf_->isGlobalHorizontalPositionValid() && ekf_->isGlobalVerticalPositionValid();
    return state;
}

}  // namespace hakoniwa::drone::control_adapter
