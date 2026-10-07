#pragma once

// Minimal stand-in for PX4's px4_platform_common/module_params.h.
//
// The adapter compiles PX4 library code without the PX4 parameter system.
// ControlAllocationSequentialDesaturation reads only MC_AIRMODE through
// ModuleParams; this header provides just enough for that class, and the
// value is set by Px4ControlAllocationBackend before allocating
// (hakoniwa::drone::control_adapter::px4_stub::set_param_int).
// Keep PX4 upstream pristine: do not add this directory for other sources.

#include <cstdint>

namespace px4::params {
enum ParamId : int {
    MC_AIRMODE = 0,
    PARAM_COUNT
};
}  // namespace px4::params

namespace hakoniwa::drone::control_adapter::px4_stub {
int32_t& param_int_storage(int id);

inline void set_param_int(int id, int32_t value)
{
    param_int_storage(id) = value;
}
}  // namespace hakoniwa::drone::control_adapter::px4_stub

class ModuleParams {
public:
    explicit ModuleParams(ModuleParams* /*parent*/) {}
    virtual ~ModuleParams() = default;

    ModuleParams(const ModuleParams&) = delete;
    ModuleParams& operator=(const ModuleParams&) = delete;

protected:
    virtual void updateParams() {}
};

template <int ParamIdValue>
class ParamInt {
public:
    int32_t get() const
    {
        return hakoniwa::drone::control_adapter::px4_stub::param_int_storage(ParamIdValue);
    }
};

// PX4 writes DEFINE_PARAMETERS((ParamInt<px4::params::X>) _param_x, ...).
// Only the single-parameter form used by ControlAllocationSequentialDesaturation
// is supported.
#define HAKO_PX4_STUB_STRIP_PARENS(...) __VA_ARGS__
#define DEFINE_PARAMETERS(param) HAKO_PX4_STUB_STRIP_PARENS param;
