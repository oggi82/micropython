cmake_minimum_required(VERSION 3.13)

# Cross-compile with arm-none-eabi-gcc (must be set before project()).
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/xmc-toolchain.cmake)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE MinSizeRel)
endif()

set(MICROPY_TARGET firmware)
get_filename_component(MICROPY_DIR "../.." ABSOLUTE)
set(MICROPY_PORT_DIR ${CMAKE_CURRENT_LIST_DIR})

# Select the board, default to the Relax Lite-Kit.
if(NOT MICROPY_BOARD)
    set(MICROPY_BOARD RELAX_LITE_KIT)
endif()
if(NOT MICROPY_BOARD_DIR)
    set(MICROPY_BOARD_DIR ${MICROPY_PORT_DIR}/boards/${MICROPY_BOARD})
endif()
get_filename_component(MICROPY_BOARD_DIR ${MICROPY_BOARD_DIR} ABSOLUTE)
if(NOT EXISTS ${MICROPY_BOARD_DIR}/mpconfigboard.cmake)
    message(FATAL_ERROR "Invalid MICROPY_BOARD specified: ${MICROPY_BOARD}")
endif()
include(${MICROPY_BOARD_DIR}/mpconfigboard.cmake)

include(${MICROPY_DIR}/py/py.cmake)

project(${MICROPY_TARGET} C CXX ASM)

set(LIB_DIR ${MICROPY_PORT_DIR}/Libraries)
set(CMSIS_SERIES_DIR ${LIB_DIR}/CMSIS/Infineon/${CMSIS_MCU}_series)

set(MICROPY_QSTRDEFS_PORT
    ${MICROPY_PORT_DIR}/qstrdefsport.h
)

set(MICROPY_SOURCE_LIB
    ${MICROPY_DIR}/shared/libc/printf.c
    ${MICROPY_DIR}/shared/libc/string0.c
    ${MICROPY_DIR}/shared/readline/readline.c
    ${MICROPY_DIR}/shared/runtime/pyexec.c
    ${MICROPY_DIR}/shared/runtime/stdout_helpers.c
)

set(MICROPY_SOURCE_MOD
    ${MICROPY_PORT_DIR}/mod_example.c
    ${MICROPY_PORT_DIR}/modmachine.c
    ${MICROPY_PORT_DIR}/modutime.c
)

set(MICROPY_SOURCE_PORT
    ${MICROPY_PORT_DIR}/main.c
    ${MICROPY_PORT_DIR}/system_xmc.c
    ${MICROPY_PORT_DIR}/uart_core.c
    ${MICROPY_PORT_DIR}/help.c
    ${MICROPY_PORT_DIR}/irq.c
    ${MICROPY_PORT_DIR}/pin.c
    ${MICROPY_PORT_DIR}/timer.c
    ${MICROPY_PORT_DIR}/pin_defs_xmc.c
    ${MICROPY_PORT_DIR}/pin_named_pins.c
    ${MICROPY_PORT_DIR}/systick.c
    ${MICROPY_PORT_DIR}/xmc_it.c
    ${MICROPY_PORT_DIR}/rtc.c
    ${MICROPY_PORT_DIR}/gchelper.s
    ${MICROPY_SOURCE_MOD}
)

set(MICROPY_SOURCE_BOARD
    ${MICROPY_BOARD_DIR}/pins_${MICROPY_BOARD}.c
)

set(MICROPY_SOURCE_HAL
    ${CMSIS_SERIES_DIR}/Source/gcc/startup_${CMSIS_MCU}.S
    ${CMSIS_SERIES_DIR}/Source/system_${CMSIS_MCU}.c
    ${LIB_DIR}/Newlib/syscalls.c
    ${LIB_DIR}/USB/Class/Device/CDCClassDevice.c
    ${LIB_DIR}/USB/Common/memcpy.c
    ${LIB_DIR}/USB/Core/ConfigDescriptors.c
    ${LIB_DIR}/USB/Core/Events.c
    ${LIB_DIR}/USB/Core/XMC4000/Endpoint_XMC4000.c
    ${LIB_DIR}/USB/Core/XMC4000/USBController_XMC4000.c
    ${LIB_DIR}/USB/Core/XMC4000/EndpointStream_XMC4000.c
    ${LIB_DIR}/USB/Core/HostStandardReq.c
    ${LIB_DIR}/USB/Core/USBTask.c
    ${LIB_DIR}/VirtualSerial/VirtualSerial.c
    ${LIB_DIR}/VirtualSerial/Descriptors.c
    ${LIB_DIR}/XMCLib/src/xmc_usbd.c
    ${LIB_DIR}/XMCLib/src/xmc_common.c
    ${LIB_DIR}/XMCLib/src/xmc_gpio.c
    ${LIB_DIR}/XMCLib/src/xmc_uart.c
    ${LIB_DIR}/XMCLib/src/xmc_usic.c
    ${LIB_DIR}/XMCLib/src/xmc4_scu.c
    ${LIB_DIR}/XMCLib/src/xmc4_gpio.c
    ${LIB_DIR}/XMCLib/src/xmc_ccu4.c
    ${LIB_DIR}/XMCLib/src/xmc_ccu8.c
)

# Sources scanned for MP_QSTR_xxx, MP_REGISTER_MODULE etc.
set(MICROPY_SOURCE_QSTR
    ${MICROPY_SOURCE_PY}
    ${MICROPY_SOURCE_LIB}
    ${MICROPY_SOURCE_PORT}
    ${MICROPY_SOURCE_BOARD}
)

# The core is built for the Cortex-M4.
set(MICROPY_CROSS_FLAGS "-march=armv7em")

set(MCU_FLAGS
    -mthumb
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=softfp
)

add_executable(${MICROPY_TARGET}
    ${MICROPY_SOURCE_PY}
    ${MICROPY_SOURCE_LIB}
    ${MICROPY_SOURCE_PORT}
    ${MICROPY_SOURCE_BOARD}
    ${MICROPY_SOURCE_HAL}
)

set_target_properties(${MICROPY_TARGET} PROPERTIES SUFFIX ".elf")

target_include_directories(${MICROPY_TARGET} PRIVATE
    ${MICROPY_INC_CORE}
    ${MICROPY_PORT_DIR}
    ${MICROPY_BOARD_DIR}
    ${CMAKE_BINARY_DIR}
    ${LIB_DIR}/XMCLib/inc
    ${LIB_DIR}
    ${LIB_DIR}/VirtualSerial
    ${LIB_DIR}/CMSIS/Include
    ${CMSIS_SERIES_DIR}/Include
)

target_compile_definitions(${MICROPY_TARGET} PRIVATE
    ${MCU_DEFINE}
)

target_compile_options(${MICROPY_TARGET} PRIVATE
    ${MCU_FLAGS}
    -Wall
    -Werror
    -Wno-array-bounds
    -g
    -gdwarf-2
    -ffunction-sections
    -fdata-sections
    -fmessage-length=0
    $<$<COMPILE_LANGUAGE:C>:-std=gnu99>
)

target_link_options(${MICROPY_TARGET} PRIVATE
    ${MCU_FLAGS}
    -nostartfiles
    -specs=nosys.specs
    -L${MICROPY_PORT_DIR}/boards
    -T${LD_FILES}
    -Wl,--gc-sections
    -Wl,-Map=${MICROPY_TARGET}.map
    -g
    -gdwarf-2
)

target_link_libraries(${MICROPY_TARGET} m)

# Flags used by the qstr preprocessor step in mkrules.cmake.
set(MICROPY_CPP_FLAGS_EXTRA ${MCU_FLAGS})

include(${MICROPY_DIR}/py/mkrules.cmake)

# Additional firmware outputs: raw binary and DFU image.
set(DFU_TOOL ${MICROPY_DIR}/tools/dfu.py)
set(PYDFU_TOOL ${MICROPY_DIR}/tools/pydfu.py)

add_custom_command(TARGET ${MICROPY_TARGET} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${MICROPY_TARGET}>
    COMMAND ${CMAKE_OBJCOPY} -O binary -j .isr_vector -j .text -j .data
        $<TARGET_FILE:${MICROPY_TARGET}> ${CMAKE_BINARY_DIR}/${MICROPY_TARGET}.bin
    COMMAND ${Python3_EXECUTABLE} ${DFU_TOOL}
        -b ${TEXT0_ADDR}:${CMAKE_BINARY_DIR}/${MICROPY_TARGET}.bin
        ${CMAKE_BINARY_DIR}/${MICROPY_TARGET}.dfu
    BYPRODUCTS ${CMAKE_BINARY_DIR}/${MICROPY_TARGET}.bin ${CMAKE_BINARY_DIR}/${MICROPY_TARGET}.dfu
    COMMENT "Creating ${MICROPY_TARGET}.bin and ${MICROPY_TARGET}.dfu"
    VERBATIM
)

# Flash the board over DFU: cmake --build <dir> --target deploy
add_custom_target(deploy
    COMMAND ${Python3_EXECUTABLE} ${PYDFU_TOOL} -u ${CMAKE_BINARY_DIR}/${MICROPY_TARGET}.dfu
    DEPENDS ${MICROPY_TARGET}
    USES_TERMINAL
)
