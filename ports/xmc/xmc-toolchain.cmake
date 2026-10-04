# Toolchain file for building the XMC port with the GNU Arm Embedded toolchain.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CROSS_COMPILE arm-none-eabi- CACHE STRING "Cross-compiler prefix")
set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++)
set(CMAKE_ASM_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_OBJCOPY ${CROSS_COMPILE}objcopy CACHE INTERNAL "")
set(CMAKE_SIZE ${CROSS_COMPILE}size CACHE INTERNAL "")

# Don't try to link test executables, there is no startup code for them.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
