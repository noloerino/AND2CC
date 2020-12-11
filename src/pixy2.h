#ifndef PIXY2_H
#define PIXY2_H

#include <stdbool.h>
#include <stdint.h>

#include "nrf_drv_spi.h"

//#define PIXY_DEBUG

#define PIXY_BUFFERSIZE 0x104
#define PIXY_CHECKSUM_SYNC 0xc1af
#define PIXY_NO_CHECKSUM_SYNC 0xc1ae
#define PIXY_SEND_HEADER_SIZE 4
#define PIXY_MAX_PROGNAME 33

#define PIXY_TYPE_REQUEST_CHANGE_PROG 0x02
#define PIXY_TYPE_REQUEST_RESOLUTION 0x0c
#define PIXY_TYPE_RESPONSE_RESOLUTION 0x0f
#define PIXY_TYPE_REQUEST_VERSION 0x0e
#define PIXY_TYPE_RESPONSE_VERSION 0x0f
#define PIXY_TYPE_RESPONSE_RESULT 0x00
#define PIXY_TYPE_RESPONSE_ERROR 0x03
#define PIXY_TYPE_REQUEST_BRIGHTNESS 0x10
#define PIXY_TYPE_REQUEST_SERVO 0x12
#define PIXY_TYPE_REQUEST_LED 0x14
#define PIXY_TYPE_REQUEST_LAMP 0x16
#define PIXY_TYPE_REQUEST_FPS 0x18

#define PIXY_RESULT_OK 0
#define PIXY_RESULT_ERROR -1
#define PIXY_RESULT_BUSY -2
#define PIXY_RESULT_CHECKSUM_ERROR -3
#define PIXY_RESULT_TIMEOUT -4
#define PIXY_RESULT_BUTTON_OVERRIDE -5
#define PIXY_RESULT_PROG_CHANGING -6

// RC-servo values
#define PIXY_RCS_MIN_POS 0
#define PIXY_RCS_MAX_POS 1000L
#define PIXY_RCS_CENTER_POS ((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2)

#define PIXY_PROG_COLOR_CODE "color_connected_components"
#define PIXY_PROG_LINE_FOLLOW "line_tracking"
#define PIXY_PROG_VIDEO "video"

#define CCC_MAX_SIGNATURE 7

#define CCC_RESPONSE_BLOCKS 0x20
#define CCC_REQUEST_BLOCKS 0x20

#define CCC_MAX_BLOCKS                                                         \
  (PIXY_BUFFERSIZE - PIXY_SEND_HEADER_SIZE) / sizeof(pixy_block_t)

// Defines for sigmap:
// You can bitwise "or" these together to make a custom sigmap.
// For example if you're only interested in receiving blocks
// with signatures 1 and 5, you could use a sigmap of
// PIXY_SIG1 | PIXY_SIG5
#define CCC_SIG1 1
#define CCC_SIG2 2
#define CCC_SIG3 4
#define CCC_SIG4 8
#define CCC_SIG5 16
#define CCC_SIG6 32
#define CCC_SIG7 64
#define CCC_COLOR_CODES 128

#define CCC_SIG_ALL 0xff // all bits or'ed together

typedef struct {
  uint16_t hardware;
  uint8_t firmware_major;
  uint8_t firmware_minor;
  uint16_t firmware_build;
  char firmware_type[10];
} pixy_version_t;

typedef struct {
  uint16_t m_signature;
  uint16_t m_x;
  uint16_t m_y;
  uint16_t m_width;
  uint16_t m_height;
  int16_t m_angle;
  uint8_t m_index;
  uint8_t m_age;
} pixy_block_t;

typedef struct {
  pixy_version_t *version;
  uint16_t frame_width;
  uint16_t frame_height;

  uint8_t *m_buf;
  uint8_t *m_buf_payload;
  uint8_t m_type;
  uint8_t m_length;
  bool m_cs;

  pixy_block_t *blocks;
  int8_t num_blocks;

  nrf_drv_spi_t *spi;
} pixy_t;

int8_t pixy_init(pixy_t **pixy_handle, nrf_drv_spi_t *spi);
void pixy_free(pixy_t *pixy);

int8_t pixy_get_version(pixy_t *pixy);
void pixy_print_version(pixy_version_t *version);
int8_t pixy_set_camera_brightness(pixy_t *pixy, uint8_t brightness);
int8_t pixy_set_led(pixy_t *pixy, uint8_t r, uint8_t g, uint8_t b);
int8_t pixy_set_lamp(pixy_t *pixy, uint8_t upper, uint8_t lower);
int8_t pixy_get_resolution(pixy_t *pixy);
int8_t pixy_get_fps(pixy_t *pixy);
int8_t pixy_get_blocks(pixy_t *pixy, bool wait, uint8_t sigmap,
                       uint8_t max_blocks);
void pixy_print_block(pixy_block_t *block);

#endif // PIXY2_H
