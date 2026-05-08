#!/bin/bash

set -euo pipefail

if [ $# -ne 1 ]
then
    echo "Usage: $0 {build|clean|test|install}"
    exit 1
fi

OPT=${1}
PROJECT_DIR=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR="${PROJECT_DIR}/build"
ADAPTER_SUBMODULE_DIR="${PROJECT_DIR}/thirdparty/hakoniwa-drone-control-adapter"
PX4_SUBMODULE_DIR="${PROJECT_DIR}/thirdparty/PX4-Autopilot"

if [ "${OPT}" = "clean" ]
then
    rm -rf "${BUILD_DIR}"
    exit 0
fi

if [ ! -d "${ADAPTER_SUBMODULE_DIR}" ]
then
    echo "Missing submodule: ${ADAPTER_SUBMODULE_DIR}"
    echo "Run: git submodule update --init --recursive"
    exit 1
fi

if [ ! -d "${PX4_SUBMODULE_DIR}" ]
then
    echo "Missing submodule: ${PX4_SUBMODULE_DIR}"
    echo "Run: git submodule update --init --recursive"
    exit 1
fi

mkdir -p "${BUILD_DIR}"

cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}"

if [ "${OPT}" = "test" ]
then
    cmake --build "${BUILD_DIR}" -j4
    ctest --test-dir "${BUILD_DIR}" --output-on-failure
    exit $?
fi

if [ "${OPT}" = "install" ]
then
    INSTALL_PREFIX="${INSTALL_PREFIX:-${PROJECT_DIR}/install}"
    cmake --build "${BUILD_DIR}" -j4
    cmake --install "${BUILD_DIR}" --prefix "${INSTALL_PREFIX}"
    echo "INSTALLED: ${INSTALL_PREFIX}"
    exit 0
fi

cmake --build "${BUILD_DIR}" -j4

echo "SUCCESS"
