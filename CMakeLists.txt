cmake_minimum_required(VERSION 3.16)

project(hakoniwa_drone_control_adapter_px4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

set(ADAPTER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../hakoniwa-drone-control-adapter)
set(PX4_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/PX4-Autopilot)

add_library(hakoniwa_drone_control_adapter_px4
    src/px4_controller_config_loader.cpp
    src/px4_attitude_control_backend.cpp
    src/px4_rate_control_backend.cpp
    ${PX4_ROOT}/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
    ${PX4_ROOT}/src/lib/rate_control/rate_control.cpp
)

target_include_directories(hakoniwa_drone_control_adapter_px4
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ADAPTER_ROOT}/include
    PRIVATE
        ${PX4_ROOT}/src/lib
        ${PX4_ROOT}/src/lib/matrix
        ${PX4_ROOT}/src/lib/rate_control
        ${PX4_ROOT}/src/modules/mc_att_control/AttitudeControl
)

target_compile_features(hakoniwa_drone_control_adapter_px4
    PUBLIC
        cxx_std_17
)

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

find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_test(
    NAME compose_px4_rate_control_config_smoke
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/compose_px4_rate_control_config_smoke.py
)
