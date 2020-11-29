
.PHONY: build flash gdb clean
build:
	cargo build

flash:
	cargo embed

clean:
	cargo clean

doc:
	cargo doc --no-deps
erase:
	# Clears RAM. May be necessary to run in order to remove NRF softdevice.
	# It may also be necessary to flash something from the lab (such as romi_template)
	# that doesn't use softdevice to get things running again.
	nrf-recover

GDB ?= arm-none-eabi-gdb -tui
TARGET ?= target/thumbv7em-none-eabihf/debug/and_2_cc

UNAME_S := $(shell uname -s)

gdb:
ifeq ($(UNAME_S),Darwin)
	osascript -e 'tell application "Terminal" to do script "cd $(PWD) && openocd"'
else
	terminal -e openocd
endif
	$(GDB) -x .gdbinit $(TARGET)

