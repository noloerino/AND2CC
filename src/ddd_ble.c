#include <stdint.h>
#include "nrf_gpio.h"

#include "simple_ble.h"
#include "buckler.h"

#include "ddd_ble.h"

// https://stackoverflow.com/questions/5459868/concatenate-int-to-string-using-c-preprocessor
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

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

ddd_ble_state_t *get_ble_state() {
  return &ble_state;
}

void ble_evt_write(ble_evt_t const *p_ble_evt) {
  if (simple_ble_is_char_event(p_ble_evt, &led_state_char)) {
    if (ble_state.led_state) {
      printf("Turning on LED\n");
      nrf_gpio_pin_clear(BUCKLER_LED2);
    } else {
      printf("Turning off LED!\n");
      nrf_gpio_pin_set(BUCKLER_LED2);
    }
  }
}

void ddd_ble_init() {
  simple_ble_app = simple_ble_init(&ble_config);
  simple_ble_add_service(&led_service);
  simple_ble_add_characteristic(1, 1, 0, 0,
    sizeof(ble_state), (uint8_t*) &ble_state,
    &led_service, &led_state_char);
  simple_ble_adv_only_name();
}
