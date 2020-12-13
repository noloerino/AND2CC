#pragma once

#include "nrf_atfifo.h"

// Since this gets serialized as (uint8_t *), we need to encoruage all fields to be byte-aligned
// manually
typedef struct ddd_ble_state {
    // IDs a ddd_ble_cmd_t; used for predictable type width
  uint8_t cmd_id;
} ddd_ble_state_t;

#ifdef SECONDARY
  #define DDD_ROBOT_ID (1)
#else
  #define DDD_ROBOT_ID (0)
#endif

// https://stackoverflow.com/questions/5459868/concatenate-int-to-string-using-c-preprocessor
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define DDD_ROBOT_ID_STR STR(DDD_ROBOT_ID)

ddd_ble_state_t *get_ble_state();
void ddd_ble_init();

typedef enum ddd_ble_cmd {
    DDD_BLE_LED_ON = 0,
    DDD_BLE_LED_OFF,
    DDD_BLE_DRV_LEFT,
    DDD_BLE_DRV_RIGHT,
    DDD_BLE_DRV_FORWARD,
    DDD_BLE_DRV_BACKWARD,
    DDD_BLE_DRV_ZERO,
    DDD_BLE_DISCONNECT,
} ddd_ble_cmd_t;

// Returns the queue used to store BLE commands.
nrf_atfifo_t *get_ble_cmd_q();
