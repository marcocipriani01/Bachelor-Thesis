cmake_minimum_required(VERSION 3.25)
set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)

include("./cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

option(FPU_HARD_ABI "Enable hardware float ABI" ON)
option(PRINTF_FLOAT "Enable float support for printf" ON)
option(DSP_ARM_LIB "Link the Arm CMSIS DSP library" ON)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F411CEUX_FLASH.ld)

project(Firmware C CXX ASM)

file(GLOB_RECURSE SOURCES
    "Core/*.*"
    "USB_DEVICE/*.*"
    "Drivers/*.*"
    "Middlewares/*.*"
)
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(Firmware PUBLIC
    "Core/Inc"
    "Drivers/CMSIS/Include"
    "Drivers/STM32F4xx_HAL_Driver/Inc"
    "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
    "Drivers/CMSIS/Device/ST/STM32F4xx/Include"
    "Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
    "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
    "USB_DEVICE/App"
    "USB_DEVICE/Target"
)

include(cmake/gcc-options.cmake)
set_gcc_options(${PROJECT_NAME})

add_custom_target(PostBuild ALL
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMENT "Executing size and objcopy"
)
add_dependencies(PostBuild ${PROJECT_NAME})

if (CMAKE_EXPORT_COMPILE_COMMANDS)
    add_custom_target(CopyCompileCommands ALL
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json ${CMAKE_CURRENT_BINARY_DIR}/../compile_commands.json
        COMMENT "Copying the compiler commands file"
    )
    add_dependencies(CopyCompileCommands ${PROJECT_NAME})
endif ()
