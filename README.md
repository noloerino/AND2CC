# DDD
**D**etect, **D**ock, **D**rive

README instructions tbd once we figure out the toolchain. Two things to note if cloning from scratch:

1. To enable SPI2, go into the buckler module and add the following lines to
`software/boards/buckler_revC/app_config.h`:
```
#define NRFX_SPIM2_ENABLED 1
#define NRFX_SPI2_ENABLED 1
#define SPI2_ENABLED 1
```
2. `make flash_0` (or `make flash`) will flash the "primary" robot, and `make flash_1` will flash
the "secondary" robot.
