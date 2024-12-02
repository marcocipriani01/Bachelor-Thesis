function(set_gcc_options TARGET_NAME)
    target_compile_definitions(${TARGET_NAME} PRIVATE
        "$<$<CONFIG:Debug>:DEBUG>"
        "USE_HAL_DRIVER"
        "STM32F411xE"
    )

    target_compile_options(${TARGET_NAME} PRIVATE
        "-mcpu=cortex-m4"
        "-Wall"
        "-Wextra"
        "-fstack-usage"
        "-fdata-sections"
        "-ffunction-sections"
        "--specs=nano.specs"
        "-Wno-missing-field-initializers"
        "$<$<COMPILE_LANGUAGE:C>:--std=gnu11>"
        "$<$<COMPILE_LANGUAGE:CXX>:--std=gnu++17>"
        "$<$<COMPILE_LANGUAGE:ASM>:-xassembler-with-cpp>"
        "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>"
        "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>"
        "$<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>"
        "$<$<CONFIG:Debug>:-g3>"
        "$<$<CONFIG:Debug>:-O0>"
        "$<$<NOT:$<CONFIG:Debug>>:-g0>"
        "$<$<NOT:$<CONFIG:Debug>>:-Os>"
    )

    target_link_options(${TARGET_NAME} PRIVATE
        "-mcpu=cortex-m4"
        "-Wall"
        "--specs=nano.specs"
        "-T" "${LINKER_SCRIPT}"
        "-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map"
    )

    if (PRINTF_FLOAT)
        target_compile_options(${TARGET_NAME} PRIVATE
            "-u" "_printf_float"
        )

        target_link_options(${TARGET_NAME} PRIVATE
            "-u" "_printf_float"
        )
    endif ()

    if (FPU_HARD_ABI)
        target_compile_options(${TARGET_NAME} PRIVATE
            "-mfloat-abi=hard"
            "-mfpu=fpv4-sp-d16"
        )

        target_link_options(${TARGET_NAME} PRIVATE
            "-mfloat-abi=hard"
            "-mfpu=fpv4-sp-d16"
        )

        if (DSP_ARM_LIB)
            target_compile_definitions(${TARGET_NAME} PRIVATE
                "ARM_MATH_CM4"
            )

            target_include_directories(${TARGET_NAME} PUBLIC
                "Drivers/CMSIS/DSP/Include"
            )

            target_link_directories(${TARGET_NAME} PRIVATE
                "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Lib/GCC"
            )

            target_link_libraries(${TARGET_NAME} PRIVATE
                "CMSISDSP"
            )
        endif ()
    else ()
        target_compile_options(${TARGET_NAME} PRIVATE
            "-mfloat-abi=soft"
        )
    endif ()
endfunction()
