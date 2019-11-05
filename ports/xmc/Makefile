include ../../py/mkenv.mk

CROSS = 0

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# include py core make definitions
include $(TOP)/py/py.mk

ifeq ($(CROSS), 1)
CROSS_COMPILE = arm-none-eabi-
endif

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -ILibraries/XMCLib/inc
INC += -ILibraries
INC += -ILibraries/VirtualSerial
INC += -ILibraries/CMSIS/Include
INC += -ILibraries/CMSIS/Infineon/XMC4500_series/Include

ifeq ($(CROSS), 1)
LD = $(CROSS_COMPILE)g++
DFU = $(TOP)/tools/dfu.py
PYDFU = $(TOP)/tools/pydfu.py
CFLAGS_CORTEX_M4 = -DXMC4500_F144x1024 -mthumb -mcpu=cortex-m4 -g -gdwarf-2 -pipe -c -fmessage-length=0 -mfpu=fpv4-sp-d16 -O0 -ffunction-sections -fdata-sections -mfloat-abi=softfp -Wa,-adhlns="$@.lst"
CFLAGS = $(INC) -Wall -Werror -std=gnu99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT)
LDFLAGS = -nostartfiles -T startup/XMC4500x1024.ld -Xlinker --gc-sections -specs=nosys.specs #--cref --gc-sections #-Wl,-Map="$@.map"
else
LD = gcc
CFLAGS = -m32 $(INC) -Wall -Werror -std=c99 $(COPT)
LDFLAGS = -m32 -Wl,-Map=$@.map,--cref -Wl,--gc-sections
endif

CSUPEROPT = -Os # save some code space

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb
else
CFLAGS += -Os -DNDEBUG
CFLAGS += -fdata-sections -ffunction-sections
endif

LIBS = 

SRC_C = \
	main.c \
	uart_core.c \
	lib/utils/printf.c \
	lib/utils/stdout_helpers.c \
	lib/utils/pyexec.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c \
	$(BUILD)/_frozen_mpy.c \
	startup/system_XMC4500.c \
	Libraries/Newlib/syscalls.c \
	Libraries/USB/Class/Device/CDCClassDevice.c \
	Libraries/USB/Common/memcpy.c \
	Libraries/USB/Core/ConfigDescriptors.c \
	Libraries/USB/Core/Events.c \
	Libraries/USB/Core/XMC4000/Endpoint_XMC4000.c \
	Libraries/USB/Core/XMC4000/USBController_XMC4000.c \
	Libraries/USB/Core/XMC4000/EndpointStream_XMC4000.c \
	Libraries/USB/Core/HostStandardReq.c \
	Libraries/USB/Core/USBTask.c \
	Libraries/VirtualSerial/VirtualSerial.c \
	Libraries/VirtualSerial/Descriptors.c \
	Libraries/XMCLib/src/xmc_usbd.c \
	Libraries/XMCLib/src/xmc_common.c \
	Libraries/XMCLib/src/xmc_gpio.c \
	Libraries/XMCLib/src/xmc_uart.c \
	Libraries/XMCLib/src/xmc_usic.c \
	Libraries/XMCLib/src/xmc4_scu.c \
	Libraries/XMCLib/src/xmc4_gpio.c \


SRC_S = \
	startup/startup_XMC4500.S \
    gchelper.S

OBJ = $(PY_CORE_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o) $(SRC_S:.S=.o))

ifeq ($(CROSS), 1)
all: $(BUILD)/firmware.dfu
else
all: $(BUILD)/firmware.elf
endif

$(BUILD)/_frozen_mpy.c: frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
	$(ECHO) "MISC freezing bytecode"
	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@

$(BUILD)/firmware.elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

$(BUILD)/firmware.bin: $(BUILD)/firmware.elf
	$(Q)$(OBJCOPY) -O binary -j .isr_vector -j .text -j .data $^ $(BUILD)/firmware.bin

$(BUILD)/firmware.dfu: $(BUILD)/firmware.bin
	$(ECHO) "Create $@"
	$(Q)$(PYTHON) $(DFU) -b 0x08000000:$(BUILD)/firmware.bin $@

deploy: $(BUILD)/firmware.dfu
	$(ECHO) "Writing $< to the board"
	$(Q)$(PYTHON) $(PYDFU) -u $<

# Run emulation build on a POSIX system with suitable terminal settings
run:
	stty raw opost -echo
	build/firmware.elf
	@echo Resetting terminal...
# This sleep is useful to spot segfaults
	sleep 1
	reset

test: $(BUILD)/firmware.elf
	$(Q)/bin/echo -e "print('hello world!', list(x+1 for x in range(10)), end='eol\\\\n')\\r\\n\\x04" | $(BUILD)/firmware.elf | tail -n2 | grep "^hello world! \\[1, 2, 3, 4, 5, 6, 7, 8, 9, 10\\]eol"

include $(TOP)/py/mkrules.mk
