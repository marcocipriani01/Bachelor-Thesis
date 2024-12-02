cmake_minimum_required(VERSION 3.25)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

set(CMSISDSP ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP)
set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_6/CMSIS/Core)

add_compile_options(
    -mcpu=cortex-m4
    -std=gnu11
    -ffunction-sections
    -fdata-sections
    # --specs=nano.specs
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
    -Wsign-compare
    -Wdouble-promotion
    -Ofast -ffast-math
    -DNDEBUG
    -Wall -Wextra -Werror
    -fshort-enums
    # -fshort-wchar
)

add_link_options(
    -mfloat-abi=hard
    -mcpu=cortex-m4
    -Wl,--gc-sections
    -static
    -mfpu=fpv4-sp-d16
    -mthumb
)

# # Define the project
project(cmsis-dsp)

add_subdirectory(${CMSISDSP}/Source bin_dsp)
