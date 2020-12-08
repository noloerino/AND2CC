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

# Include main Makefile
include $(NRF_BASE_DIR)/make/AppMakefile.mk
