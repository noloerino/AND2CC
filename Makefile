PROJECT_NAME = "ddd"

# Configurations
NRF_IC = nrf52832
SDK_VERSION = 15
SOFTDEVICE_MODEL = s132

# Source and header files
APP_HEADER_PATHS += ./src/
APP_SOURCE_PATHS += ./src/
APP_SOURCES = $(notdir $(wildcard ./src/*.c))

NRF_BASE_DIR ?= ./buckler/software/nrf52x-base/

# Include board Makefile (if any)
include ./buckler/software/boards/buckler_revC/Board.mk

# Needed after inclusion of board makefile...
BOARD_SOURCES += \
	nrf_atfifo.c\

# Include main Makefile
include $(NRF_BASE_DIR)/make/AppMakefile.mk

# Custom rules to support build changes
# Start ble server
.PHONY: ble_server
ble_server:
	python3 ble_server/ble_server.py

# The identifier distinguishing between the two robots is set by the SECONDARY macro.
# Invoking make with `CFLAGS=-DSECONDARY` is insufficient because the toolchain isn't intelligent
# enough to recompile our C sources; we therefore touch the only header file where this macro is
# used to trigger a rebuild of the rest of our sources.
.PHONY: flash_0 flash_1
flash_0:
	touch ./src/ddd_ble.h
	$(MAKE) CFLAGS=-USECONDARY flash

flash_1:
	touch ./src/ddd_ble.h
	$(MAKE) CFLAGS=-DSECONDARY flash
