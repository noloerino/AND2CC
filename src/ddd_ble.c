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

// LED was 108a
static simple_ble_char_t req_state_char = {.uuid16 = 0x108B};
static simple_ble_char_t resp_state_char = {.uuid16 = 0x108C};

static ddd_ble_req_t ble_req = { 0 };
// Strictly speaking, this isn't a response - rather, it's a read-only characteristic
// set after req is updated
static ddd_ble_resp_t ble_resp = { 0 };

simple_ble_app_t *simple_ble_app;

// Don't need to buffer all that many commands since the server will block
NRF_ATFIFO_DEF(ble_cmd_q, ddd_ble_cmd_t, 2);

nrf_atfifo_t *get_ble_cmd_q() {
  return ble_cmd_q;
}

void ble_evt_write(ble_evt_t const *p_ble_evt) {
  static ddd_ble_cmd_t prepared_cmd = DDD_BLE_INVALID;
  if (simple_ble_is_char_event(p_ble_evt, &req_state_char)) {
    uint8_t seq_no = ble_req.seq_no;
    switch (ble_req.sync_req_id) {
      case SYNC_2PC_CMD_PREPARE: {
        prepared_cmd = (ddd_ble_cmd_t) ble_req.cmd_id;
        printf("[sync] Received 2PC prepare %hhu\n", prepared_cmd);
        ble_resp.t2 = 0; // TODO
        ble_resp.sync_resp_id = (uint8_t) SYNC_2PC_RESP_VOTE_COMMIT;
        ble_resp.seq_no = seq_no;
        break;
      }
      case SYNC_2PC_CMD_COMMIT: {
        // TODO check PTP time sync; if time too long then just don't do the job
        APP_ERROR_CHECK(
          nrf_atfifo_alloc_put(ble_cmd_q, &prepared_cmd, sizeof(ddd_ble_cmd_t), NULL)
        );
        printf("[sync] Acknowledging 2PC commit\n");
        // Strictly speaking we should send a fail response if alloc_put fails
        ble_resp.t2 = 0;
        ble_resp.sync_resp_id = (uint8_t) SYNC_2PC_RESP_ACK;
        ble_resp.seq_no = seq_no;
        break;
      }
      case SYNC_2PC_CMD_ABORT: {
        prepared_cmd = DDD_BLE_INVALID;
        printf("[sync] Acknowledging 2PC abort\n");
        ble_resp.t2 = 0;
        ble_resp.sync_resp_id = (uint8_t) SYNC_2PC_RESP_ACK;
        ble_resp.seq_no = seq_no;
        break;
      }
      default: {
        printf("[sync] Invalid 2PC command %hhu\n", ble_req.sync_req_id);
        ble_resp.t2 = 0;
        ble_resp.sync_resp_id = (uint8_t) SYNC_2PC_RESP_INVALID;
        ble_resp.seq_no = seq_no;
        break;
      }
    }
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
    sizeof(ble_req), (uint8_t*) &ble_req,
    &led_service, &req_state_char);
  simple_ble_add_characteristic(1, 0, 0, 0,
    sizeof(ble_resp), (uint8_t*) &ble_resp,
    &led_service, &resp_state_char);
  APP_ERROR_CHECK(
    NRF_ATFIFO_INIT(ble_cmd_q)
  );
  simple_ble_adv_only_name();
  printf("Initialized DDD BLE\n");
}
