cmake_minimum_required(VERSION 3.15)
project(e1_meter_reader)

set(EFF_STDIO_PORT 3)                                   # EVK stdio = UART3 = our link
file(REAL_PATH "$ENV{HOME}/effcc/sdk" EFF_SDK_ROOT_DIR) # adjust if SDK elsewhere
include(${EFF_SDK_ROOT_DIR}/setup_sdk.cmake)
add_subdirectory(${EFF_SDK_ROOT_DIR} sdk)

add_compile_definitions(E1_TARGET)

add_eff_app(meter_reader
    TYPE exe
    ARCHS e1x
    TARGETS fabric scalar
    SOURCE main.c gauge.c kernels.c ray_tables.c protocol.c hal_e1.c
    )

add_eff_app(link_echo
    TYPE exe
    ARCHS e1x
    TARGETS scalar
    SOURCE echo_main.c hal_e1.c
    )

if(E1_LINK_BAUD)
    add_compile_definitions(E1_LINK_BAUD=${E1_LINK_BAUD})
    set(EFF_BAUD_FLAG -DE1_LINK_BAUD=${E1_LINK_BAUD})
endif()

# belt-and-braces: force E1_TARGET onto every subtarget even if
# directory-level definitions don't reach the eff toolchain steps
eff_subtarget_compile_options(meter_reader_fabric PRIVATE -DE1_TARGET ${EFF_BAUD_FLAG})
eff_subtarget_compile_options(meter_reader_scalar PRIVATE -DE1_TARGET ${EFF_BAUD_FLAG})
eff_subtarget_compile_options(link_echo_scalar    PRIVATE -DE1_TARGET ${EFF_BAUD_FLAG})
