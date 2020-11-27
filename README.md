# DDD
**D**etect, **D**ock, **D**rive

## Installation
Compiling this project requires Rust to be installed.

To produce code for the nRF52832, first run the following commands (see https://docs.rust-embedded.org/book/intro/install.html):
- `rustup target add thumbv7em-none-eabihf`
- `cargo install cargo-binutils cargo-embed`
- `rustup component add llvm-tools-preview`

For Macs, the following must also be run:
- `brew install armmbed/formulae/arm-none-eabi-gcc openocd`

The structure of this repository is loosely based on nrf-hal's examples, which can be found [here](https://github.com/nrf-rs/nrf-hal/tree/master/examples).

## Running and Flashing
`cargo embed` is aliased to `make flash`, and will rebuild and upload the binary and automatically start an RTT session.

To run GDB, simply run `make gdb` --- this will open a GDB server in a new terminal window, and GDB
itself in the terminal the command was run in.

## BLE
BLE functionality is provided by the [rubble](https://github.com/jonas-schievink/rubble) crate.
Most of the framework for setting up the bluetooh module comes from their demo, which can be found
here (https://github.com/jonas-schievink/rubble/tree/master/demos/nrf52-demo).