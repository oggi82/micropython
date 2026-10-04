# The XMC port

This port is intended to be a MicroPython port that actually runs on XMC controller.
For so long the XMC4500 and the Relax Lite-Kit is supported-

## Building

The port is built with CMake and the GNU Arm Embedded toolchain
(`arm-none-eabi-gcc`) and needs `cmake` (3.13 or newer) and optionally `ninja`:

    $ cmake -S . -B build-RELAX_LITE_KIT -G Ninja
    $ cmake --build build-RELAX_LITE_KIT

The board is selected with `-DMICROPY_BOARD=<name>` (default `RELAX_LITE_KIT`,
see `boards/`); a different toolchain prefix can be given with
`-DCROSS_COMPILE=<prefix>`. Use `-DCMAKE_BUILD_TYPE=Debug` for an unoptimised build.

Building produces `firmware.elf`, `firmware.bin` and `firmware.dfu` in the build
directory. The DFU image can be programmed to the MCU using:

    $ cmake --build build-RELAX_LITE_KIT --target deploy

This version of the build will work out-of-the-box on a Relax Lite-Kit,
and will give you a MicroPython REPL on USB VCOM at 115200
baud.

## Building without the built-in MicroPython compiler

This minimal port can be built with the built-in MicroPython compiler
disabled.  This will reduce the firmware by about 20k on a Thumb2 machine,
and by about 40k on 32-bit x86.  Without the compiler the REPL will be
disabled, but pre-compiled scripts can still be executed.

