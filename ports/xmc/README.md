# The XMC port

This port is intended to be a MicroPython port that actually runs on XMC controller.
For so long the XMC4500 and the Relax Lite-Kit is supported-

## Building and running Linux version

By default the port will be built for the host machine:

    $ make

To run the executable and get a basic working REPL do:

    $ make run

## Building for an XMC MCU

The Makefile has the ability to build for a Cortex-M CPU, and by default
includes some start-up code for an XMC45xx MCU and also enables a UART
for communication.  To build:

    $ make

Building will produce the build/firmware.dfu file which can be programmed
to an MCU using:

    $ make deploy

This version of the build will work out-of-the-box on a Relax Lite-Kit, 
and will give you a MicroPython REPL on USB VCOM at 115200
baud.

## Building without the built-in MicroPython compiler

This minimal port can be built with the built-in MicroPython compiler
disabled.  This will reduce the firmware by about 20k on a Thumb2 machine,
and by about 40k on 32-bit x86.  Without the compiler the REPL will be
disabled, but pre-compiled scripts can still be executed.

