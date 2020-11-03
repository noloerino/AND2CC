# AND2CC

## Installation
Compiling this project requires Rust to be installed.

To produce code for the nRF52832, first run the following commands (see https://docs.rust-embedded.org/book/intro/install.html):
- `rustup target add thumbv7em-none-eabihf`
- `cargo install cargo-binutils`
- `rustup component add llvm-tools-preview`

For Macs, the following must also be run:
- `brew install armmbed/formulae/arm-none-eabi-gcc openocd qemu`

The structure of this repository is loosely based on the Rust Embedded WG's `cortex-m-quickstart`, which can be found [here](https://github.com/rust-embedded/cortex-m-quickstart).
