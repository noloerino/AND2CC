# DDD
**D**etect, **D**ock, **D**rive

README instructions tbd once we figure out the toolchain. Two things to note if cloning from scratch:

1. To enable SPI2, go into the buckler module and add the following 3 lines to
`software/boards/buckler_revC/app_config.h`:
```
#define NRFX_SPIM2_ENABLED 1
#define NRFX_SPI2_ENABLED 1
#define SPI2_ENABLED 1
```
2. To compile code for the secondary robot, run `make CFLAGS=-DSECONDARY flash`