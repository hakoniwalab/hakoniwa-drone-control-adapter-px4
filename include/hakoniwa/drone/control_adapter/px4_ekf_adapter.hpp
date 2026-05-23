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
    void set_in_air_status(bool in_air) override;
    void set_vehicle_at_rest(bool at_rest) override;

    void push_imu(const EkfImuInput& input, double dt_sec) override;
    void push_mag(const EkfMagInput& input) override;
    void push_baro(const EkfBaroInput& input) override;
    void push_gps(const EkfHilGpsInput& input) override;

    void update() override;

    EkfEstimatedState get_estimated_state() const override;

private:
    void ensure_initialized(std::uint64_t time_usec);
    void apply_sensor_policy();

    EkfAdapterConfig config_{};
    ::Ekf* ekf_{nullptr};
    bool initialized_{false};
    std::uint64_t last_input_time_usec_{0};
};

}  // namespace hakoniwa::drone::control_adapter
