#include <stdint.h>
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_atfifo.h"

#include "simple_ble.h"
#include "buckler.h"

#include "ddd_ble.h"

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
  // c0:98:e5:49:xx:xx
  .platform_id       = 0x49,    // used as 4th octect in device BLE address
  .device_id         = DDD_ROBOT_ID,
  .adv_name          = "EE149 | DDD " STR(DDD_ROBOT_ID), // used in advertisements if there is room
  .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
  .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
  .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
// The 128-bit UUID of the LED service from lab, which we're reusing for simplicity
static simple_ble_service_t led_service = {{
  .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
              0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
}};

static simple_ble_char_t led_state_char = {.uuid16 = 0x108a};

static ddd_ble_state_t ble_state = { 0 };

simple_ble_app_t *simple_ble_app;

// Don't need to buffer all that many commands since the server will block
NRF_ATFIFO_DEF(ble_cmd_q, ddd_ble_cmd_t, 2);

nrf_atfifo_t *get_ble_cmd_q() {
  return ble_cmd_q;
}

ddd_ble_state_t *get_ble_state() {
  return &ble_state;
}

void ble_evt_write(ble_evt_t const *p_ble_evt) {
  if (simple_ble_is_char_event(p_ble_evt, &led_state_char)) {
    ddd_ble_cmd_t cmd = (ddd_ble_cmd_t) ble_state.cmd_id;
    APP_ERROR_CHECK(
      nrf_atfifo_alloc_put(ble_cmd_q, &cmd, sizeof(ddd_ble_cmd_t), NULL)
    );
  }
}

void ble_evt_disconnected(ble_evt_t const*p_ble_evt) {
  // Strictly speaking a disconnect should be prioritized over others so we should pop if
  // the queue is full, but whatever  
  ddd_ble_cmd_t cmd = DDD_BLE_DISCONNECT;
  APP_ERROR_CHECK(
    nrf_atfifo_alloc_put(ble_cmd_q, &cmd, sizeof(ddd_ble_cmd_t), NULL)
  );
}

void ddd_ble_init() {
  simple_ble_app = simple_ble_init(&ble_config);
  simple_ble_add_service(&led_service);
  simple_ble_add_characteristic(1, 1, 0, 0,
    sizeof(ble_state), (uint8_t*) &ble_state,
    &led_service, &led_state_char);
  APP_ERROR_CHECK(
    NRF_ATFIFO_INIT(ble_cmd_q)
  );
  simple_ble_adv_only_name();
  printf("Initialized DDD BLE");
}
