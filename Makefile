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
BOARD_SOURCES += nrf_atfifo.c

# Include main Makefile
include $(NRF_BASE_DIR)/make/AppMakefile.mk

# Start ble server
.PHONY: ble_server
ble_server:
	python3 ble_server/ble_server.py


