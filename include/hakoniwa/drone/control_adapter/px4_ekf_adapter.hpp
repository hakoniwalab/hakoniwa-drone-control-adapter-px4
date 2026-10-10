#pragma once

#include "hakoniwa/drone/control_adapter/ekf_adapter.hpp"

class Ekf;

namespace hakoniwa::drone::control_adapter {

class Px4EkfAdapter final : public IEkfAdapter {
public:
    explicit Px4EkfAdapter(const EkfAdapterConfig& config = {});
    ~Px4EkfAdapter() override;

    void reset() override;
    void set_config(const EkfAdapterConfig& config) override;
    void set_armed_status(bool armed) override;
    void set_in_air_status(bool in_air) override;
    void set_vehicle_at_rest(bool at_rest) override;

    void push_imu(const EkfImuInput& input, double dt_sec) override;
    void push_mag(const EkfMagInput& input) override;
    void push_baro(const EkfBaroInput& input) override;
    void push_gps(const EkfHilGpsInput& input) override;

    void update() override;

    EkfEstimatedState get_estimated_state() const override;

    // PX4's expected sensor delays (SENS_GPS0_DELAY, EKF2_BARO_DELAY), PX4-specific and outside the
    // Control Link EKF config. PX4 stamps a GPS sample SENS_GPS0_DELAY earlier (VehicleGPSPosition)
    // and EKF2 shifts a baro sample by EKF2_BARO_DELAY. Not called: no delay, as before.
    void set_sensor_delays_ms(double gps_delay_ms, double baro_delay_ms);

private:
    void ensure_initialized(std::uint64_t time_usec);
    void apply_sensor_policy();

    EkfAdapterConfig config_{};
    ::Ekf* ekf_{nullptr};
    bool initialized_{false};
    std::uint64_t last_input_time_usec_{0};
    double gps_delay_ms_{0.0};
    double baro_delay_ms_{0.0};
};

}  // namespace hakoniwa::drone::control_adapter
