# DDD
**D**etect, **D**ock, **D**rive

## Building
1. To enable SPI2, go into the buckler module and add the following lines to
`software/boards/buckler_revC/app_config.h`:
```
#define NRFX_SPIM2_ENABLED 1
#define NRFX_SPI2_ENABLED 1
#define SPI2_ENABLED 1
```
2. `make flash_0` will flash the "primary" robot, and `make flash_1` will flash the "secondary" robot.
3. `make ble_server` will launch a REPL to control the two robots.

## BLE REPL Controls
The following commands are available:
- REPL controls
    - `quit` quits the REPL
    - `reconnect` attempts to reconnect a dropped BLE connection
- Synchronization
    - `nosync` disables the synchronization protocol for commands
    - `sync` reenables the synchronization protocol
    - `setdelay <ms>` toggles the scheduling delay on synchronization
- Forcing FSM state changes
    - `go` moves the robots from OFF to SPIN
    - `stop` moves both robots back to the OFF state and stops their motors
    - `d` moves both robots to DOCKED, where they become receptive to more commands
- Commands when docked
    - `on`/`off` toggle LED2 on both boards
    - `z` sets the motor speeds to 0, stopping both robots
    - `l`, `r`, `f`, `b` turn the robots left, right, and moves them forward and backward, respectively
