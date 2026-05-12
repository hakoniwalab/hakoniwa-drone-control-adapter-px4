cmake_minimum_required(VERSION 3.16)

project(hakoniwa_drone_control_adapter_px4 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

set(ADAPTER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hakoniwa-drone-control-adapter)
set(PX4_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/PX4-Autopilot)

add_library(hakoniwa_drone_control_adapter_px4 STATIC
    src/px4_controller_config_loader.cpp
    src/px4_altitude_control_backend.cpp
    src/px4_allocation_feedback_policy.cpp
    src/px4_attitude_control_backend.cpp
    src/px4_control_allocation_hakoniwa_converter.cpp
    src/px4_control_allocation_backend.cpp
    src/px4_horizontal_position_control_backend.cpp
    src/px4_position_control_3d_backend.cpp
    src/px4_rate_control_backend.cpp
    ${PX4_ROOT}/src/lib/control_allocation/control_allocation/ControlAllocation.cpp
    ${PX4_ROOT}/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp
    ${PX4_ROOT}/src/modules/mc_pos_control/PositionControl/ControlMath.cpp
    ${PX4_ROOT}/src/modules/mc_pos_control/PositionControl/PositionControl.cpp
    ${PX4_ROOT}/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
    ${PX4_ROOT}/src/lib/rate_control/rate_control.cpp
)

add_library(hakoniwa_drone_control_adapter_px4::hakoniwa_drone_control_adapter_px4 ALIAS
    hakoniwa_drone_control_adapter_px4)

target_include_directories(hakoniwa_drone_control_adapter_px4
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${ADAPTER_ROOT}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${PX4_ROOT}/src
        ${PX4_ROOT}/src/lib
        ${PX4_ROOT}/src/lib/control_allocation
        ${PX4_ROOT}/src/lib/control_allocation/control_allocation
        ${PX4_ROOT}/src/lib/matrix
        ${PX4_ROOT}/src/lib/rate_control
        ${PX4_ROOT}/src/modules/mc_att_control/AttitudeControl
        ${PX4_ROOT}/src/modules/mc_pos_control/PositionControl
)

target_compile_features(hakoniwa_drone_control_adapter_px4
    PUBLIC
        cxx_std_17
)

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
