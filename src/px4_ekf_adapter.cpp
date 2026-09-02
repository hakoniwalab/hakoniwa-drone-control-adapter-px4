#include "hakoniwa/drone/control_adapter/px4_ekf_adapter.hpp"

#include "EKF/common.h"
#include "EKF/ekf.h"

#include <cmath>
#include <limits>

namespace hakoniwa::drone::control_adapter {

float clamp_non_negative(double value)
{
    return static_cast<float>((value < 0.0) ? 0.0 : value);
}

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
    last_input_time_usec_ = 0;
}

void Px4EkfAdapter::set_config(const EkfAdapterConfig& config)
{
    config_ = config;
}

void Px4EkfAdapter::set_armed_status(bool armed)
{
    // PX4 EKF2 exposes in-air and at-rest status, but no independent arming
    // input at this adapter boundary. Keep the generic state explicit without
    // incorrectly mapping it to a different EKF2 flag.
    (void)armed;
}

void Px4EkfAdapter::set_in_air_status(bool in_air)
{
    if (!ekf_) {
        return;
    }
    ekf_->set_in_air_status(in_air);
}

void Px4EkfAdapter::set_vehicle_at_rest(bool at_rest)
{
    if (!ekf_) {
        return;
    }
    ekf_->set_vehicle_at_rest(at_rest);
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

    auto* params = ekf_->getParamHandle();
    params->ekf2_mag_decl = static_cast<float>(config_.mag_declination_deg);
    params->ekf2_decl_type = static_cast<int32_t>(GeoDeclinationMask::SAVE_GEO_DECL);

    const auto& px4 = config_.px4_params;
    if (px4.ekf2_gps_ctrl) {
        params->ekf2_gps_ctrl = *px4.ekf2_gps_ctrl;
    }
    if (px4.ekf2_gps_check) {
        params->ekf2_gps_check = *px4.ekf2_gps_check;
    }
    if (px4.ekf2_req_eph) {
        params->ekf2_req_eph = static_cast<float>(*px4.ekf2_req_eph);
    }
    if (px4.ekf2_req_epv) {
        params->ekf2_req_epv = static_cast<float>(*px4.ekf2_req_epv);
    }
    if (px4.ekf2_req_sacc) {
        params->ekf2_req_sacc = static_cast<float>(*px4.ekf2_req_sacc);
    }
    if (px4.ekf2_req_nsats) {
        params->ekf2_req_nsats = *px4.ekf2_req_nsats;
    }
    if (px4.ekf2_req_pdop) {
        params->ekf2_req_pdop = static_cast<float>(*px4.ekf2_req_pdop);
    }
    if (px4.ekf2_req_fix) {
        params->ekf2_req_fix = *px4.ekf2_req_fix;
    }
    if (px4.ekf2_gps_p_noise) {
        params->ekf2_gps_p_noise = static_cast<float>(*px4.ekf2_gps_p_noise);
    }
    if (px4.ekf2_gps_v_noise) {
        params->ekf2_gps_v_noise = static_cast<float>(*px4.ekf2_gps_v_noise);
    }
    if (px4.ekf2_gps_p_gate) {
        params->ekf2_gps_p_gate = static_cast<float>(*px4.ekf2_gps_p_gate);
    }
    if (px4.ekf2_gps_v_gate) {
        params->ekf2_gps_v_gate = static_cast<float>(*px4.ekf2_gps_v_gate);
    }
    if (px4.ekf2_hgt_ref) {
        params->ekf2_hgt_ref = *px4.ekf2_hgt_ref;
    }
    if (px4.ekf2_baro_ctrl) {
        params->ekf2_baro_ctrl = *px4.ekf2_baro_ctrl;
    }
    if (px4.ekf2_baro_noise) {
        params->ekf2_baro_noise = static_cast<float>(*px4.ekf2_baro_noise);
    }
    if (px4.ekf2_baro_gate) {
        params->ekf2_baro_gate = static_cast<float>(*px4.ekf2_baro_gate);
    }
    if (px4.ekf2_mag_decl) {
        params->ekf2_mag_decl = static_cast<float>(*px4.ekf2_mag_decl);
    }
    if (px4.ekf2_decl_type) {
        params->ekf2_decl_type = *px4.ekf2_decl_type;
    }
    if (px4.ekf2_mag_type) {
        params->ekf2_mag_type = static_cast<MagFuseType>(*px4.ekf2_mag_type);
    }
    if (px4.ekf2_head_noise) {
        params->ekf2_head_noise = static_cast<float>(*px4.ekf2_head_noise);
    }
    if (px4.ekf2_hdg_gate) {
        params->ekf2_hdg_gate = static_cast<float>(*px4.ekf2_hdg_gate);
    }
    if (px4.ekf2_mag_noise) {
        params->ekf2_mag_noise = static_cast<float>(*px4.ekf2_mag_noise);
    }
    if (px4.ekf2_mag_gate) {
        params->ekf2_mag_gate = static_cast<float>(*px4.ekf2_mag_gate);
    }
}

void Px4EkfAdapter::push_imu(const EkfImuInput& input, double dt_sec)
{
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0 || dt_sec > 1.0) {
        return;
    }

    ensure_initialized(input.time_usec);
    last_input_time_usec_ = input.time_usec;

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
}

void Px4EkfAdapter::push_mag(const EkfMagInput& input)
{
    ensure_initialized(input.time_usec);
    last_input_time_usec_ = input.time_usec;

    estimator::magSample mag{};
    mag.time_us = input.time_usec;
    mag.mag = matrix::Vector3f(
        static_cast<float>(input.xmag_gauss),
        static_cast<float>(input.ymag_gauss),
        static_cast<float>(input.zmag_gauss));
    ekf_->setMagData(mag);
}

void Px4EkfAdapter::push_baro(const EkfBaroInput& input)
{
    ensure_initialized(input.time_usec);
    last_input_time_usec_ = input.time_usec;

    estimator::baroSample baro{};
    baro.time_us = input.time_usec;
    baro.hgt = static_cast<float>(input.pressure_alt_m);
    ekf_->setBaroData(baro);
}

void Px4EkfAdapter::push_gps(const EkfHilGpsInput& input)
{
    ensure_initialized(input.time_usec);
    last_input_time_usec_ = input.time_usec;

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
    gps.sacc = clamp_non_negative(input.sacc_mps);
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

    state.time_usec = last_input_time_usec_;

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

    state.active_horizontal_aiding_sources = ekf_->getNumberOfActiveHorizontalAidingSources();
    state.active_horizontal_position_aiding_sources = ekf_->getNumberOfActiveHorizontalPositionAidingSources();
    state.active_horizontal_velocity_aiding_sources = ekf_->getNumberOfActiveHorizontalVelocityAidingSources();
    state.active_vertical_position_aiding_sources = ekf_->getNumberOfActiveVerticalPositionAidingSources();
    state.active_vertical_velocity_aiding_sources = ekf_->getNumberOfActiveVerticalVelocityAidingSources();

    const auto &control_flags = ekf_->control_status_flags();
    state.gnss_pos_fused = control_flags.gnss_pos;
    state.gnss_vel_fused = control_flags.gnss_vel;
    state.gps_hgt_fused = control_flags.gps_hgt;

    state.horizontal_velocity_innovation_test_ratio =
        static_cast<double>(ekf_->getHorizontalVelocityInnovationTestRatio());
    state.vertical_velocity_innovation_test_ratio =
        static_cast<double>(ekf_->getVerticalVelocityInnovationTestRatio());
    state.horizontal_position_innovation_test_ratio =
        static_cast<double>(ekf_->getHorizontalPositionInnovationTestRatio());
    state.vertical_position_innovation_test_ratio =
        static_cast<double>(ekf_->getVerticalPositionInnovationTestRatio());
    return state;
}

}  // namespace hakoniwa::drone::control_adapter
