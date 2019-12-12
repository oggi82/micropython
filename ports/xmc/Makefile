# Select the board to build for: if not given on the command line,
# then default to PYBV10.
BOARD ?= RELAX_LITE_KIT

# If the build directory is not given, make it reflect the board name.
BUILD ?= build-$(BOARD)

BOARD_DIR ?= boards/$(BOARD)
ifeq ($(wildcard $(BOARD_DIR)/.),)
$(error Invalid BOARD specified: $(BOARD_DIR))
endif

include ../../py/mkenv.mk
# -include mpconfigport.mk # no used for so long, there is no ethernet nor wifi
include $(BOARD_DIR)/mpconfigboard.mk

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h #$(BUILD)/pins_qstr.h
QSTR_GLOBAL_DEPENDENCIES = $(BOARD_DIR)/mpconfigboard.h

# include py core make definitions
include $(TOP)/py/py.mk

MCU_SERIES_UPPER = $(shell echo $(MCU_SERIES) | tr '[:lower:]' '[:upper:]')
CMSIS_MCU_LOWER = $(shell echo $(CMSIS_MCU) | tr '[:upper:]' '[:lower:]')

LD_DIR=boards
STARTUP_FILE ?= lib/xmclib/CMSIS/XMC$(MCU_SERIES_UPPER)xx/Source/Templates/gcc/startup_$(CMSIS_MCU).o
SYSTEM_FILE ?= lib/xmclib/CMSIS/XMC$(MCU_SERIES_UPPER)xx/Source/Templates/system_$(CMSIS_MCU).o

CROSS_COMPILE ?= arm-none-eabi-

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(BOARD_DIR)
INC += -ILibraries/XMCLib/inc
INC += -ILibraries
INC += -ILibraries/VirtualSerial
INC += -ILibraries/CMSIS/Include
INC += -ILibraries/CMSIS/Infineon/XMC4500_series/Include

LD = $(CROSS_COMPILE)g++
DFU = $(TOP)/tools/dfu.py
PYDFU = $(TOP)/tools/pydfu.py
CFLAGS_CORTEX_M4 = -DXMC4500_F144x1024 -mthumb -mcpu=cortex-m4 -g -gdwarf-2 -pipe -c -fmessage-length=0 -mfpu=fpv4-sp-d16 -O0 -ffunction-sections -fdata-sections -mfloat-abi=softfp -Wa,-adhlns="$@.lst"
CFLAGS = $(INC) -Wall -Werror -std=gnu99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT)
LDFLAGS = -nostartfiles -L $(LD_DIR) $(addprefix -T,$(LD_FILES)) -Xlinker --gc-sections -specs=nosys.specs -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mcpu=cortex-m4 -mthumb -g -gdwarf-2 #--cref --gc-sections #-Wl,-Map="$@.map"

CSUPEROPT = -Os # save some code space

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb
else
CFLAGS += -Os -DNDEBUG
CFLAGS += -fdata-sections -ffunction-sections
endif

LIBS = 

EXTMOD_SRC_C = \

SRC_MOD = \
	mod_example.c \
	modmachine.c \
	modutime.c \

SRC_C = \
	main.c \
	system_xmc.c \
	uart_core.c \
	help.c \
	lib/utils/printf.c \
	lib/utils/stdout_helpers.c \
	lib/utils/pyexec.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c \
	irq.c \
	machine_pin.c \
	machine_timer.c \
	pin_defs_xmc.c \
	pin_named_pins.c \
	systick.c \
	xmc_it.c \
	rtc.c \

SRC_HAL = \
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
	Libraries/XMCLib/src/xmc_ccu4.c \
	Libraries/XMCLib/src/xmc_ccu8.c \
	

SRC_O = \
	$(STARTUP_FILE) \
	$(SYSTEM_FILE) \
	gchelper.o    

PLLVALUES = boards/pllvalues.py
MAKE_PINS = boards/make-pins.py
BOARD_PINS = $(BOARD_DIR)/pins.csv
GEN_PINS_SRC = $(BUILD)/pins_$(BOARD).c
GEN_PINS_HDR = $(HEADER_BUILD)/pins.h
GEN_PINS_QSTR = $(BUILD)/pins_qstr.h

# List of sources for qstr extraction
SRC_QSTR += $(SRC_C)

OBJ = 
OBJ += $(PY_CORE_O)
OBJ += $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_O))
OBJ += $(addprefix $(BUILD)/, $(SRC_HAL:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_MOD:.c=.o))
OBJ += $(BOARD_DIR)/pins_$(BOARD).o

# Use a pattern rule here so that make will only call make-pins.py once to make
# both pins_$(BOARD).c and pins.h
$(BUILD)/%_$(BOARD).c $(HEADER_BUILD)/%.h $(HEADER_BUILD)/%_af_const.h $(HEADER_BUILD)/%_af_defs.h $(BUILD)/%_qstr.h: $(BOARD_DIR)/%.csv $(MAKE_PINS) $(AF_FILE) $(PREFIX_FILE) | $(HEADER_BUILD)
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(MAKE_PINS) --board $(BOARD_PINS) --af $(AF_FILE) \
		--prefix $(PREFIX_FILE) --hdr $(GEN_PINS_HDR) --hdr-obj-decls \
		--qstr $(GEN_PINS_QSTR) --af-const $(GEN_PINS_AF_CONST) \
		--af-defs $(GEN_PINS_AF_DEFS) --af-defs-cmp-strings \
		--af-py $(GEN_PINS_AF_PY) > $(GEN_PINS_SRC)

all: $(BUILD)/firmware.dfu

# $(BUILD)/_frozen_mpy.c: frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
# 	$(ECHO) "MISC freezing bytecode"
# 	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=mpz $< > $@

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
