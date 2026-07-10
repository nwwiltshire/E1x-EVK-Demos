# Hardware build for the E1 acoustic sentinel — the Efficient SDK's
# CMake flow, mirroring e1x_examples/app_examples/CMakeLists.txt.
#
#   cd firmware && cmake -B bld && cmake --build bld
#
# then flash per the SDK quickstart.  Produces:
#   link_echo_scalar     — step-1 bring-up app (see host/echo_test.py)
#   hum_sentinel_scalar  — demo on the RISC-V control core (bring-up)
#   hum_sentinel_fabric  — demo with the DSP loops on the dataflow fabric
cmake_minimum_required(VERSION 3.15)

project(e1_hum_sentinel)

# EVK stdio is UART3 (same as the examples).  hal_e1.c claims UART_3
# for the binary chunk link and re-inits it in hal_init(); there is no
# printf in the firmware core, and any stray runtime print lands in
# the chunk stream where the host parser resyncs past it.
set(EFF_STDIO_PORT 3)

# Adjust if your SDK is installed elsewhere.
file(REAL_PATH "$ENV{HOME}/effcc/sdk" EFF_SDK_ROOT_DIR)
include(${EFF_SDK_ROOT_DIR}/setup_sdk.cmake)
add_subdirectory(${EFF_SDK_ROOT_DIR} sdk)

# Our targets only (the SDK subdirectory was added above this line).
add_compile_definitions(E1_TARGET)

add_eff_app(hum_sentinel
    TYPE exe
    ARCHS e1x
    TARGETS fabric scalar
    SOURCE main.c detector.c dsp.c fft_tables.c protocol.c hal_e1.c
    )

add_eff_app(link_echo
    TYPE exe
    ARCHS e1x
    TARGETS scalar
    SOURCE echo_main.c hal_e1.c
    )

# Optional link baud override, e.g. cmake -B bld -DE1_LINK_BAUD=434062 —
# moot on the EVK USB bridge, whose UART is fixed at ~115200 (measured;
# see hal_e1.c), but kept for direct-UART wiring.
if(E1_LINK_BAUD)
    add_compile_definitions(E1_LINK_BAUD=${E1_LINK_BAUD})
    set(EFF_BAUD_FLAG -DE1_LINK_BAUD=${E1_LINK_BAUD})
endif()

# belt-and-braces: make sure every subtarget sees E1_TARGET even if
# directory-level definitions don't reach the eff toolchain steps
eff_subtarget_compile_options(hum_sentinel_fabric PRIVATE -DE1_TARGET ${EFF_BAUD_FLAG})
eff_subtarget_compile_options(hum_sentinel_scalar PRIVATE -DE1_TARGET ${EFF_BAUD_FLAG})
eff_subtarget_compile_options(link_echo_scalar PRIVATE -DE1_TARGET ${EFF_BAUD_FLAG})
