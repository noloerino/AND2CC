#include <stdint.h>
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_atfifo.h"

#include "simple_ble.h"
#include "display.h"
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
static simple_ble_char_t nosync_state_char = {.uuid16 = 0x108D};

static ddd_ble_req_t ble_req = { 0 };
// Strictly speaking, this isn't a response - rather, it's a read-only characteristic
// set after req is updated
static ddd_ble_resp_t ble_resp = { 0 };
static uint8_t ble_nosync_cmd_id = 0;

simple_ble_app_t *simple_ble_app;

// Don't need to buffer all that many commands since the server will block
NRF_ATFIFO_DEF(ble_cmd_q, ddd_ble_timed_cmd_t, 4);

nrf_atfifo_t *get_ble_cmd_q() {
  return ble_cmd_q;
}

uint32_t ddd_ble_now_ms() {
  NRF_TIMER4->TASKS_CAPTURE[1] = 1;
  // timer runs in us
  return NRF_TIMER4->CC[1] / 1000;
}

void ble_evt_write(ble_evt_t const *p_ble_evt) {
  static ddd_ble_cmd_t prepared_cmd = DDD_BLE_INVALID;
  static uint32_t target_time = 0;
  if (simple_ble_is_char_event(p_ble_evt, &req_state_char)) {
    uint8_t seq_no = ble_req.seq_no;
    switch (ble_req.sync_req_id) {
      case SYNC_2PC_CMD_PREPARE: {
        prepared_cmd = (ddd_ble_cmd_t) ble_req.cmd_id;
        printf("[sync] Received 2PC prepare (cmd=%hhu, seq=%hhu)\n", prepared_cmd, seq_no);
        ble_resp.t2 = ddd_ble_now_ms();
        ble_resp.sync_resp_id = (uint8_t) SYNC_2PC_RESP_VOTE_COMMIT;
        ble_resp.seq_no = seq_no;
        target_time = ble_req.ts.leader_target_ms;
        break;
      }
      case SYNC_2PC_CMD_COMMIT: {
        // Target time is of the central device, so we add the error
        uint32_t when = ble_req.ts.e > 0
          ? target_time + (uint32_t) ble_req.ts.e
          : target_time - (uint32_t) (-ble_req.ts.e);
        uint32_t now = ddd_ble_now_ms();
        ddd_ble_timed_cmd_t timed_cmd;
        timed_cmd.cmd = prepared_cmd;
        timed_cmd.target_ms = when;
        APP_ERROR_CHECK(
          nrf_atfifo_alloc_put(ble_cmd_q, &timed_cmd, sizeof(timed_cmd), NULL)
        );
        printf(
          "[sync] Acknowledging 2PC commit (seq=%hhu), should run in %lu ms\n",
          seq_no,
          when > now ? when - now : 0
        );
        // Strictly speaking we should send a fail response if alloc_put fails
        ble_resp.t2 = 0;
        ble_resp.sync_resp_id = (uint8_t) SYNC_2PC_RESP_ACK;
        ble_resp.seq_no = seq_no;
        break;
      }
      case SYNC_2PC_CMD_ABORT: {
        prepared_cmd = DDD_BLE_INVALID;
        printf("[sync] Acknowledging 2PC abort (seq=%hhu)\n", seq_no);
        ble_resp.t2 = 0;
        ble_resp.sync_resp_id = (uint8_t) SYNC_2PC_RESP_ACK;
        ble_resp.seq_no = seq_no;
        break;
      }
      default: {
        printf("[sync] Invalid 2PC command %hhu (seq=%hhu)\n", ble_req.sync_req_id, seq_no);
        ble_resp.t2 = 0;
        ble_resp.sync_resp_id = (uint8_t) SYNC_2PC_RESP_INVALID;
        ble_resp.seq_no = seq_no;
        break;
      }
    }
  } else if (simple_ble_is_char_event(p_ble_evt, &nosync_state_char)) {
    printf("[nosync] Directly queuing command %hhu\n", ble_nosync_cmd_id);
    ddd_ble_timed_cmd_t timed_cmd;
    timed_cmd.cmd = (ddd_ble_cmd_t) ble_nosync_cmd_id;
    timed_cmd.target_ms = 0;
    APP_ERROR_CHECK(
      nrf_atfifo_alloc_put(ble_cmd_q, &timed_cmd, sizeof(timed_cmd), NULL)
    );
  }
}

static ddd_ble_timed_cmd_t disconnect_cmd = {
  .cmd = DDD_BLE_DISCONNECT,
  .target_ms = 0
};

void ble_evt_disconnected(ble_evt_t const *p_ble_evt) {
  nrf_atfifo_clear(ble_cmd_q);
  APP_ERROR_CHECK(
    nrf_atfifo_alloc_put(ble_cmd_q, &disconnect_cmd, sizeof(disconnect_cmd), NULL)
  );
}

void ble_evt_connected(ble_evt_t const *p_ble_evt) {
  display_write("[ble] Connected!", 1);
}

void empty_callback(void *p_context) { }

void ddd_ble_init() {
  static bool has_ble_init = false;
  if (!has_ble_init) {
    // Initialize virtual timer
    NRF_TIMER4->BITMODE = 3; // 32-bit
    NRF_TIMER4->PRESCALER = 4; // 1 MHz
    NRF_TIMER4->TASKS_CLEAR = 1;
    NRF_TIMER4->TASKS_START = 1;
    // Initialize softdevice stuff
    simple_ble_app = simple_ble_init(&ble_config);
    simple_ble_add_service(&led_service);
    simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(ble_req), (uint8_t *) &ble_req,
      &led_service, &req_state_char);
    simple_ble_add_characteristic(1, 0, 0, 0,
      sizeof(ble_resp), (uint8_t *) &ble_resp,
      &led_service, &resp_state_char);
    simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(uint8_t), (uint8_t *) &ble_nosync_cmd_id,
      &led_service, &nosync_state_char);
    APP_ERROR_CHECK(
      NRF_ATFIFO_INIT(ble_cmd_q)
    );
    simple_ble_adv_only_name();
    has_ble_init = true;
    printf("Initialized DDD BLE " DDD_ROBOT_ID_STR "\n");
  }
}
