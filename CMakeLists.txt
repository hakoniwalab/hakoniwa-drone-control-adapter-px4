cmake_minimum_required(VERSION 3.16)

project(hakoniwa_drone_control_adapter_px4 LANGUAGES CXX)

# Default to an optimized build. Without a build type the PX4 EKF and
# controllers were compiled without optimization, which slowed every
# Drone PRO simulation that links this library.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

set(ADAPTER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hakoniwa-drone-control-adapter)
set(PX4_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/PX4-Autopilot)

# Keep PX4 upstream sources read-only in this repository. Any standalone build
# adaptation must live in local shim headers, wrapper sources, or CMake glue.

add_library(hakoniwa_drone_control_adapter_px4 STATIC
    src/px4_controller_config_loader.cpp
    src/px4_altitude_control_backend.cpp
    src/px4_allocation_feedback_policy.cpp
    src/px4_attitude_control_backend.cpp
    src/px4_control_allocation_hakoniwa_converter.cpp
    src/px4_control_allocation_backend.cpp
    src/px4_ekf_adapter.cpp
    src/px4_horizontal_position_control_backend.cpp
    src/px4_position_control_3d_backend.cpp
    src/px4_rate_control_backend.cpp
    ${PX4_ROOT}/src/lib/geo/geo.cpp
    ${PX4_ROOT}/src/lib/lat_lon_alt/lat_lon_alt.cpp
    ${PX4_ROOT}/src/lib/world_magnetic_model/geo_mag_declination.cpp
    ${PX4_ROOT}/src/lib/atmosphere/atmosphere.cpp
    ${PX4_ROOT}/src/lib/ringbuffer/Ringbuffer.cpp
    ${PX4_ROOT}/src/lib/control_allocation/control_allocation/ControlAllocation.cpp
    ${PX4_ROOT}/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp
    ${PX4_ROOT}/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp
    ${PX4_ROOT}/src/modules/mc_pos_control/PositionControl/ControlMath.cpp
    ${PX4_ROOT}/src/modules/mc_pos_control/PositionControl/PositionControl.cpp
    ${PX4_ROOT}/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
    ${PX4_ROOT}/src/lib/rate_control/rate_control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/bias_estimator/bias_estimator.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/covariance.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/ekf.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/ekf_helper.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/estimator_interface.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/height_control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/velocity_fusion.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/position_fusion.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/yaw_fusion.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/fake_height_control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/fake_pos_control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/ZeroGyroUpdate.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/ZeroVelocityUpdate.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/gnss/gnss_checks.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_fusion.cpp
    ${PX4_ROOT}/src/modules/ekf2/EKF/yaw_estimator/EKFGSF_yaw.cpp
)

# Keep PX4 upstream pristine. atmosphere.cpp uses powf() in standalone builds,
# so inject math.h only for this source file via build glue instead of patching
# the PX4 submodule.
if (MSVC)
    set_source_files_properties(
        ${PX4_ROOT}/src/lib/atmosphere/atmosphere.cpp
        PROPERTIES
            COMPILE_OPTIONS "/FImath.h"
    )
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    set_source_files_properties(
        ${PX4_ROOT}/src/lib/atmosphere/atmosphere.cpp
        PROPERTIES
            COMPILE_OPTIONS "-include;math.h"
    )
endif()

set_target_properties(hakoniwa_drone_control_adapter_px4 PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

add_library(hakoniwa_drone_control_adapter_px4::hakoniwa_drone_control_adapter_px4 ALIAS
    hakoniwa_drone_control_adapter_px4)

target_include_directories(hakoniwa_drone_control_adapter_px4
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${ADAPTER_ROOT}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        # Minimal px4_platform_common/module_params.h for
        # ControlAllocationSequentialDesaturation (PX4 parameters are not built).
        ${CMAKE_CURRENT_SOURCE_DIR}/px4_stubs
        ${PX4_ROOT}/src
        ${PX4_ROOT}/src/lib
        ${PX4_ROOT}/src/lib/control_allocation
        ${PX4_ROOT}/src/lib/control_allocation/control_allocation
        ${PX4_ROOT}/src/lib/atmosphere
        ${PX4_ROOT}/src/lib/geo
        ${PX4_ROOT}/src/lib/lat_lon_alt
        ${PX4_ROOT}/src/lib/matrix
        ${PX4_ROOT}/src/lib/rate_control
        ${PX4_ROOT}/src/lib/ringbuffer
        ${PX4_ROOT}/src/lib/world_magnetic_model
        ${PX4_ROOT}/src/modules/ekf2
        ${PX4_ROOT}/src/modules/ekf2/EKF
        ${PX4_ROOT}/src/modules/ekf2/EKF/python
        ${PX4_ROOT}/src/modules/mc_att_control/AttitudeControl
        ${PX4_ROOT}/src/modules/mc_pos_control/PositionControl
)

target_compile_features(hakoniwa_drone_control_adapter_px4
    PUBLIC
        cxx_std_17
)

target_compile_definitions(hakoniwa_drone_control_adapter_px4
    PRIVATE
        CONFIG_EKF2_GNSS=1
        CONFIG_EKF2_BAROMETER=1
        CONFIG_EKF2_MAGNETOMETER=1
)

# Drone PRO's HAKO_EKF_IMU_ACCELERATION: Drone PRO hands the controllers the IMU's rate and the gyro's
# raw derivative, and the rate stage applies PX4's IMU filters (IMU_GYRO_CUTOFF, IMU_DGYRO_CUTOFF).
# Build both with the same setting. Off: the inputs pass through unchanged, as before.
option(HAKO_EKF_IMU_ACCELERATION "Apply PX4's IMU filters to the rate stage inputs" OFF)
if(HAKO_EKF_IMU_ACCELERATION)
    target_compile_definitions(hakoniwa_drone_control_adapter_px4 PRIVATE HAKO_EKF_IMU_ACCELERATION)
    # PX4's Auto trajectory (FlightTaskAuto's PositionSmoothing and HeadingSmoothing) in the 3D
    # position stage, as PX4 SITL flies a reposition (px4_position_control_3d_backend.cpp).
    target_sources(hakoniwa_drone_control_adapter_px4 PRIVATE
        ${PX4_ROOT}/src/lib/motion_planning/PositionSmoothing.cpp
        ${PX4_ROOT}/src/lib/motion_planning/VelocitySmoothing.cpp
        ${PX4_ROOT}/src/lib/motion_planning/HeadingSmoothing.cpp)
endif()

install(
    TARGETS hakoniwa_drone_control_adapter_px4
    EXPORT hakoniwa_drone_control_adapter_px4Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/include/
        ${ADAPTER_ROOT}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_drone_control_adapter_px4ConfigVersion.cmake
    VERSION 0.1.0
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/hakoniwa_drone_control_adapter_px4Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_drone_control_adapter_px4Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hakoniwa_drone_control_adapter_px4
)

install(
    EXPORT hakoniwa_drone_control_adapter_px4Targets
    FILE hakoniwa_drone_control_adapter_px4Targets.cmake
    NAMESPACE hakoniwa_drone_control_adapter_px4::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hakoniwa_drone_control_adapter_px4
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_drone_control_adapter_px4Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_drone_control_adapter_px4ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hakoniwa_drone_control_adapter_px4
)

set(ADAPTER_TEST_ROOT ${ADAPTER_ROOT}/test)

add_executable(px4_rate_control_backend_smoke
    test/px4_rate_control_backend_smoke.cpp
)

target_link_libraries(px4_rate_control_backend_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_rate_control_backend_smoke
    COMMAND px4_rate_control_backend_smoke
)

add_executable(px4_attitude_control_backend_smoke
    test/px4_attitude_control_backend_smoke.cpp
)

target_link_libraries(px4_attitude_control_backend_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_attitude_control_backend_smoke
    COMMAND px4_attitude_control_backend_smoke
)

add_executable(px4_altitude_control_backend_smoke
    test/px4_altitude_control_backend_smoke.cpp
)

target_link_libraries(px4_altitude_control_backend_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_altitude_control_backend_smoke
    COMMAND px4_altitude_control_backend_smoke
)

add_executable(px4_horizontal_position_control_backend_smoke
    test/px4_horizontal_position_control_backend_smoke.cpp
)

target_link_libraries(px4_horizontal_position_control_backend_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_horizontal_position_control_backend_smoke
    COMMAND px4_horizontal_position_control_backend_smoke
)

add_executable(px4_position_control_3d_backend_smoke
    test/px4_position_control_3d_backend_smoke.cpp
)

target_link_libraries(px4_position_control_3d_backend_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_position_control_3d_backend_smoke
    COMMAND px4_position_control_3d_backend_smoke
)

add_executable(px4_manual_yaw_lock_test
    test/px4_manual_yaw_lock_test.cpp
)
target_link_libraries(px4_manual_yaw_lock_test
    PRIVATE hakoniwa_drone_control_adapter_px4
)
if(HAKO_EKF_IMU_ACCELERATION)
    target_compile_definitions(px4_manual_yaw_lock_test PRIVATE HAKO_EKF_IMU_ACCELERATION)
endif()
add_test(NAME px4_manual_yaw_lock_test COMMAND px4_manual_yaw_lock_test)

add_executable(px4_ekf_sensor_delay_smoke
    test/px4_ekf_sensor_delay_smoke.cpp
)

target_link_libraries(px4_ekf_sensor_delay_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_ekf_sensor_delay_smoke
    COMMAND px4_ekf_sensor_delay_smoke
)

add_executable(px4_control_allocation_backend_smoke
    test/px4_control_allocation_backend_smoke.cpp
)

target_link_libraries(px4_control_allocation_backend_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_control_allocation_backend_smoke
    COMMAND px4_control_allocation_backend_smoke
)

add_executable(px4_control_allocation_hakoniwa_converter_smoke
    test/px4_control_allocation_hakoniwa_converter_smoke.cpp
)

target_link_libraries(px4_control_allocation_hakoniwa_converter_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_control_allocation_hakoniwa_converter_smoke
    COMMAND px4_control_allocation_hakoniwa_converter_smoke
)

add_executable(px4_controller_config_loader_smoke
    test/px4_controller_config_loader_smoke.cpp
)

target_link_libraries(px4_controller_config_loader_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_controller_config_loader_smoke
    COMMAND px4_controller_config_loader_smoke
)
set_tests_properties(px4_controller_config_loader_smoke PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(px4_rate_control_config_only_smoke
    test/px4_rate_control_config_only_smoke.cpp
)

target_link_libraries(px4_rate_control_config_only_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_rate_control_config_only_smoke
    COMMAND px4_rate_control_config_only_smoke
)
set_tests_properties(px4_rate_control_config_only_smoke PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(px4_attitude_control_config_only_smoke
    test/px4_attitude_control_config_only_smoke.cpp
)

target_link_libraries(px4_attitude_control_config_only_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_attitude_control_config_only_smoke
    COMMAND px4_attitude_control_config_only_smoke
)
set_tests_properties(px4_attitude_control_config_only_smoke PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(px4_altitude_control_config_only_smoke
    test/px4_altitude_control_config_only_smoke.cpp
)

target_link_libraries(px4_altitude_control_config_only_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_altitude_control_config_only_smoke
    COMMAND px4_altitude_control_config_only_smoke
)
set_tests_properties(px4_altitude_control_config_only_smoke PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(px4_horizontal_position_control_config_only_smoke
    test/px4_horizontal_position_control_config_only_smoke.cpp
)

target_link_libraries(px4_horizontal_position_control_config_only_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_horizontal_position_control_config_only_smoke
    COMMAND px4_horizontal_position_control_config_only_smoke
)
set_tests_properties(px4_horizontal_position_control_config_only_smoke PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(px4_control_allocation_config_only_smoke
    test/px4_control_allocation_config_only_smoke.cpp
)

target_link_libraries(px4_control_allocation_config_only_smoke
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_control_allocation_config_only_smoke
    COMMAND px4_control_allocation_config_only_smoke
)
set_tests_properties(px4_control_allocation_config_only_smoke PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(px4_usage_examples
    ${ADAPTER_TEST_ROOT}/usage_examples.cpp
    test/px4_usage_examples.cpp
)

target_include_directories(px4_usage_examples
    PRIVATE
        ${ADAPTER_TEST_ROOT}
)

target_link_libraries(px4_usage_examples
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_usage_examples
    COMMAND px4_usage_examples
)
set_tests_properties(px4_usage_examples PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(px4_feedback_policy_examples
    ${ADAPTER_TEST_ROOT}/feedback_policy_examples.cpp
    test/px4_feedback_policy_examples.cpp
)

target_include_directories(px4_feedback_policy_examples
    PRIVATE
        ${ADAPTER_TEST_ROOT}
)

target_link_libraries(px4_feedback_policy_examples
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_feedback_policy_examples
    COMMAND px4_feedback_policy_examples
)
set_tests_properties(px4_feedback_policy_examples PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(px4_frequency_usage_examples
    ${ADAPTER_TEST_ROOT}/frequency_usage_examples.cpp
    test/px4_frequency_usage_examples.cpp
)

target_include_directories(px4_frequency_usage_examples
    PRIVATE
        ${ADAPTER_TEST_ROOT}
)

target_link_libraries(px4_frequency_usage_examples
    PRIVATE
        hakoniwa_drone_control_adapter_px4
)

add_test(
    NAME px4_frequency_usage_examples
    COMMAND px4_frequency_usage_examples
)
set_tests_properties(px4_frequency_usage_examples PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_test(
    NAME compose_px4_rate_control_config_smoke
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/compose_px4_rate_control_config_smoke.py
)

# Compile the rate backend itself twice so every test build checks both sides of
# HAKO_EKF_IMU_ACCELERATION, independently of the option used for the main library.
foreach(filter_mode IN ITEMS on off)
    set(filter_test px4_imu_velocity_filters_${filter_mode}_smoke)
    add_executable(${filter_test}
        test/px4_imu_velocity_filters_smoke.cpp
        src/px4_rate_control_backend.cpp
    )
    target_include_directories(${filter_test} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${PX4_ROOT}/src
        ${PX4_ROOT}/src/lib
        ${PX4_ROOT}/src/lib/matrix
        ${PX4_ROOT}/src/lib/rate_control
    )
    target_link_libraries(${filter_test} PRIVATE hakoniwa_drone_control_adapter_px4)
    if(filter_mode STREQUAL "on")
        target_compile_definitions(${filter_test} PRIVATE HAKO_EKF_IMU_ACCELERATION EXPECT_IMU_FILTERS)
    endif()
    add_test(NAME ${filter_test} COMMAND ${filter_test})
endforeach()
