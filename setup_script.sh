#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd ${SCRIPT_DIR}/vins_core
cmake -S . -B ./build               \
        -DCMAKE_BUILD_TYPE=Release   \
        -DCMAKE_INSTALL_PREFIX="./install"

wait

cmake --build ./build -j$(nproc)

wait

cmake --install ./build
