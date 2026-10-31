#pragma once

#include "hakoniwa/drone/control_adapter/ekf_adapter.hpp"
#include "hakoniwa/drone/control_adapter/px4_sensor_data_type.hpp"
#include "hakoniwa/drone/control_adapter/px4_sensor_model_v2.hpp"

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

    // The sensor data types of the PX4 controller config (px4_sensor_data_type.hpp): the IMU and GPS
    // as PX4 SITL receives them. Not called: exact, as before.
    void set_sensor_data_types(const Px4SensorDataTypes& types);
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    void set_sensor_model_v2_config(const Px4SensorModelV2Config& config);
#endif

private:
    void ensure_initialized(std::uint64_t time_usec);
    void apply_sensor_policy();

    EkfAdapterConfig config_{};
    ::Ekf* ekf_{nullptr};
    bool initialized_{false};
    std::uint64_t last_input_time_usec_{0};
    double gps_delay_ms_{0.0};
    double baro_delay_ms_{0.0};
    Px4SensorDataTypes sensor_data_types_{};
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    Px4SensorModelV2Config sensor_model_v2_{};
    // Barometer samples averaged until the next SENS_BARO_RATE publication (PX4 VehicleAirData).
    double baro_pressure_sum_pa_{0.0};
    std::uint64_t baro_time_sum_usec_{0};
    std::uint32_t baro_sample_count_{0};
    std::uint64_t baro_last_publication_usec_{0};
#endif
};

}  // namespace hakoniwa::drone::control_adapter
