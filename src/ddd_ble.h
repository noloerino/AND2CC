#pragma once

// Since this gets serialized as (uint8_t *), we need to encoruage all fields to be byte-aligned
// manually
// TODO distinguish between internal state (like busy variable) and writeable state
typedef struct ddd_ble_state {
  uint8_t led_state;
  uint8_t : 3;
  float speed_left;
  float speed_right;
} ddd_ble_state_t;

#ifdef SECONDARY
  #define DDD_ROBOT_ID (0x0001)
#else
  #define DDD_ROBOT_ID (0x0000)
#endif

ddd_ble_state_t *get_ble_state();
void ddd_ble_init();
