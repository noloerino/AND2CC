
.PHONY: build flash gdb clean
build:
	cargo build

flash:
	cargo embed

clean:
	cargo clean

doc:
	cargo doc --no-deps

GDB ?= arm-none-eabi-gdb -tui
TARGET ?= target/thumbv7em-none-eabihf/debug/and_2_cc

UNAME_S := $(shell uname -s)

gdb:
ifeq ($(UNAME_S),Darwin)
	# closes single quote
	osascript -e 'tell application "Terminal" to do script "cd $(PWD) && openocd"'
else
	terminal -e openocd
endif
	$(GDB) -x .gdbinit $(TARGET)

