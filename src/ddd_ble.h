#pragma once

#include "nrf_atfifo.h"

typedef enum {
  SYNC_2PC_CMD_INVALID = 0,
  SYNC_2PC_CMD_PREPARE,
  SYNC_2PC_CMD_COMMIT,
  SYNC_2PC_CMD_ABORT,
} sync_2pc_cmd_t;

typedef enum {
  SYNC_2PC_RESP_INVALID = 0,
  SYNC_2PC_RESP_VOTE_COMMIT,
  SYNC_2PC_RESP_VOTE_ABORT,
  SYNC_2PC_RESP_ACK,
} sync_2pc_resp_t;

typedef enum {
  DDD_BLE_INVALID = 0,
  DDD_BLE_LED_ON,
  DDD_BLE_LED_OFF,
  DDD_BLE_DRV_LEFT,
  DDD_BLE_DRV_RIGHT,
  DDD_BLE_DRV_FORWARD,
  DDD_BLE_DRV_BACKWARD,
  DDD_BLE_DRV_ZERO,
  DDD_BLE_DISCONNECT,
} ddd_ble_cmd_t;

typedef struct {
  ddd_ble_cmd_t cmd;
  uint32_t target_ms;
} ddd_ble_timed_cmd_t;

// A request issued to the BLE GATT peripheral (this device) from the central (a laptop).
//
// Since this gets serialized as (uint8_t *), we need to encourage all fields to be byte-aligned
// manually. No unions are used to make the type layout easier to reason about on the server side.
// Enums are replaced by uint types to make width predictable as well.
typedef struct {
  uint32_t t1; // PTP t1 on a 2PC prepare
  union {
    uint32_t delay; // target delay on a 2PC prepare
    int32_t e; // clock error on a 2PC commit
  } ts;
  // IDs whether this is a 2PC prepare, commit, or abort command.
  uint8_t sync_req_id;
  // IDs a ddd_ble_cmd_t (should be checked only on 2PC prepare).
  uint8_t cmd_id;
  uint8_t seq_no; // Counter to match command to ack
} ddd_ble_req_t;

// A response to a GATT request.
typedef struct {
  uint32_t t2; // t2 on a 2PC vote
  uint8_t sync_resp_id; // IDs a 2PC vote commit, vote abort, or ack
  uint8_t seq_no; // number of command being acknowledged or voted on
  uint8_t : 2;
} ddd_ble_resp_t;

#ifdef SECONDARY
  #define DDD_ROBOT_ID (1)
#else
  #define DDD_ROBOT_ID (0)
#endif

// https://stackoverflow.com/questions/5459868/concatenate-int-to-string-using-c-preprocessor
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define DDD_ROBOT_ID_STR STR(DDD_ROBOT_ID)

uint32_t ddd_ble_now_ms();

void ddd_ble_init();

// Returns the queue used to store BLE commands.
nrf_atfifo_t *get_ble_cmd_q();
