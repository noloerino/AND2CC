#include "nrf_delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pixy2.h"

// Helper functions for abstracting SPI communications.
ret_code_t recv(nrf_drv_spi_t *spi, uint8_t *buf, uint8_t len);
void send(nrf_drv_spi_t *spi, uint8_t *buf, uint8_t len);
int16_t get_sync(pixy_t *pixy);
int16_t recv_packet(pixy_t *pixy);
void send_packet(pixy_t *pixy);

int8_t pixy_init(pixy_t **pixy_handle, nrf_drv_spi_t *spi) {
  pixy_t *pixy = (pixy_t *)malloc(sizeof(pixy_t));
  *pixy_handle = pixy;

  pixy->m_buf = (uint8_t *)malloc(PIXY_BUFFERSIZE);
  pixy->m_buf_payload = pixy->m_buf + PIXY_SEND_HEADER_SIZE;
  pixy->frame_width = pixy->frame_height = 0;
  pixy->version = NULL;

  pixy->spi = spi;

  pixy->blocks = NULL;
  pixy->num_blocks = 0;

  for (size_t i = 0; i < 20; ++i) {
    if (pixy_get_version(pixy) == PIXY_RESULT_OK) {
      pixy_get_resolution(pixy);
      return PIXY_RESULT_OK;
    }
    nrf_delay_ms(250);
  }

  return PIXY_RESULT_TIMEOUT;
}

void pixy_free(pixy_t *pixy) {
  free(pixy->m_buf);
  free(pixy);
}

int16_t get_sync(pixy_t *pixy) {
  uint8_t i, j, c, cprev;
  int16_t res;
  uint16_t start;

  // parse bytes until we find sync
  for (i = j = 0, cprev = 0; true; ++i) {
    res = recv(pixy->spi, &c, 1);
    if (res == NRF_SUCCESS) {
      start = cprev;
      start |= c << 8;
      cprev = c;
      if (start == PIXY_CHECKSUM_SYNC) {
        pixy->m_cs = true;
        return PIXY_RESULT_OK;
      }
      if (start == PIXY_NO_CHECKSUM_SYNC) {
        pixy->m_cs = false;
        return PIXY_RESULT_OK;
      }
    }

    if (i >= 4) {
      if (j >= 4) {
        return PIXY_RESULT_TIMEOUT;
      }
      nrf_delay_ms(25);
      ++j;
      i = 0;
    }
  }
}

int16_t recv_packet(pixy_t *pixy) {
  uint16_t cs_calc, cs_serial;
  int16_t res;

  // clear out any stale data
  res = get_sync(pixy);
  if (res == PIXY_RESULT_ERROR)
    return res;

  if (pixy->m_cs) {
    res = recv(pixy->spi, pixy->m_buf, 4);
    if (res != NRF_SUCCESS)
      return res;

    pixy->m_type = pixy->m_buf[0];
    pixy->m_length = pixy->m_buf[1];

    cs_serial = *(uint16_t *)&pixy->m_buf[2];

    res = recv(pixy->spi, pixy->m_buf, pixy->m_length);
    if (res != NRF_SUCCESS)
      return res;

    cs_calc = 0;
    for (uint8_t i = 0; i < pixy->m_length; ++i) {
      cs_calc += pixy->m_buf[i];
    }

    if (cs_serial != cs_calc) {
      return PIXY_RESULT_CHECKSUM_ERROR;
    }
  } else {
    res = recv(pixy->spi, pixy->m_buf, 2);
    if (res != NRF_SUCCESS)
      return res;

    pixy->m_type = pixy->m_buf[0];
    pixy->m_length = pixy->m_buf[1];

    res = recv(pixy->spi, pixy->m_buf, pixy->m_length);
    if (res != NRF_SUCCESS)
      return res;
  }

  return PIXY_RESULT_OK;
}

void send_packet(pixy_t *pixy) {
  pixy->m_buf[0] = PIXY_NO_CHECKSUM_SYNC & 0xff;
  pixy->m_buf[1] = PIXY_NO_CHECKSUM_SYNC >> 8;
  pixy->m_buf[2] = pixy->m_type;
  pixy->m_buf[3] = pixy->m_length;
  send(pixy->spi, pixy->m_buf, pixy->m_length + PIXY_SEND_HEADER_SIZE);
}

int8_t pixy_get_version(pixy_t *pixy) {
  int32_t res;

  pixy->m_length = 0;
  pixy->m_type = PIXY_TYPE_REQUEST_VERSION;
  send_packet(pixy);
  if ((res = recv_packet(pixy)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (pixy->m_type == PIXY_TYPE_RESPONSE_VERSION) {
    pixy->version = (pixy_version_t *)pixy->m_buf;
    return pixy->m_length;
  } else if (pixy->m_type == PIXY_TYPE_RESPONSE_ERROR)
    return PIXY_RESULT_BUSY;
  else
    return PIXY_RESULT_ERROR;
}

int8_t pixy_get_resolution(pixy_t *pixy) {
  int32_t res;

  pixy->m_length = 1;
  pixy->m_buf_payload[0] = 0; // for future types of queries
  pixy->m_type = PIXY_TYPE_REQUEST_RESOLUTION;
  send_packet(pixy);
  if ((res = recv_packet(pixy)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (pixy->m_type == PIXY_TYPE_RESPONSE_RESOLUTION) {
    pixy->frame_width = ((uint16_t *)pixy->m_buf)[0];
    pixy->frame_height = ((uint16_t *)pixy->m_buf)[1];
    return PIXY_RESULT_OK; // success
  } else {
    return PIXY_RESULT_ERROR;
  }
}

int8_t pixy_set_camera_brightness(pixy_t *pixy, uint8_t brightness) {
  uint32_t res;

  pixy->m_buf_payload[0] = brightness;
  pixy->m_length = 1;
  pixy->m_type = PIXY_TYPE_REQUEST_BRIGHTNESS;
  send_packet(pixy);
  // && pixy->m_type==PIXY_TYPE_RESPONSE_RESULT && pixy->m_length==4)
  if ((res = recv_packet(pixy)) == PIXY_RESULT_OK) {
    res = *(uint32_t *)pixy->m_buf;
    return (int8_t)res;
  } else
    return res; // some kind of bitstream error
}

int8_t pixy_set_led(pixy_t *pixy, uint8_t r, uint8_t g, uint8_t b) {
  uint32_t res;

  pixy->m_buf_payload[0] = r;
  pixy->m_buf_payload[1] = g;
  pixy->m_buf_payload[2] = b;
  pixy->m_length = 3;
  pixy->m_type = PIXY_TYPE_REQUEST_LED;
  send_packet(pixy);
  if ((res = recv_packet(pixy)) != PIXY_RESULT_OK)
    return res;
  if (pixy->m_type == PIXY_TYPE_RESPONSE_RESULT && pixy->m_length == 4) {
    res = *(uint32_t *)pixy->m_buf;
    return (int8_t)res;
  } else
    return PIXY_RESULT_ERROR;
}

int8_t pixy_set_lamp(pixy_t *pixy, uint8_t upper, uint8_t lower) {
  uint32_t res;

  pixy->m_buf_payload[0] = upper;
  pixy->m_buf_payload[1] = lower;
  pixy->m_length = 2;
  pixy->m_type = PIXY_TYPE_REQUEST_LAMP;
  send_packet(pixy);
  if ((res = recv_packet(pixy)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (pixy->m_type == PIXY_TYPE_RESPONSE_RESULT && pixy->m_length == 4) {
    res = *(uint32_t *)pixy->m_buf;
    return (int8_t)res;
  } else
    return PIXY_RESULT_ERROR;
}

int8_t pixy_get_fps(pixy_t *pixy) {
  uint32_t res;

  pixy->m_length = 0; // no args
  pixy->m_type = PIXY_TYPE_REQUEST_FPS;
  send_packet(pixy);
  if ((res = recv_packet(pixy)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (pixy->m_type == PIXY_TYPE_RESPONSE_RESULT && pixy->m_length == 4) {
    res = *(uint32_t *)pixy->m_buf;
    return (int8_t)res;
  } else
    return PIXY_RESULT_ERROR; // some kind of bitstream error
}

void send(nrf_drv_spi_t *spi, uint8_t *data, uint8_t len) {
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, data, len, NULL, 0));
}

ret_code_t recv(nrf_drv_spi_t *spi, uint8_t *data, uint8_t len) {
  ret_code_t error_code = nrf_drv_spi_transfer(spi, NULL, 0, data, len);
  return error_code;
}

void pixy_print_version(pixy_version_t *version) {
  printf("hw version: 0x%x fw version: %d.%d.%d %s\n", version->hardware,
         version->firmware_major, version->firmware_minor,
         version->firmware_build, version->firmware_type);
}

// print block structure!
void pixy_print_block(pixy_block_t *b) {
  int i, j;
  char sig[6], d;
  bool flag;
  if (b->m_signature > CCC_MAX_SIGNATURE) { // color code! (CC)
    // convert signature number to an octal string
    for (i = 12, j = 0, flag = false; i >= 0; i -= 3) {
      d = (b->m_signature >> i) & 0x07;
      if (d > 0 && !flag)
        flag = true;
      if (flag)
        sig[j++] = d + '0';
    }
    sig[j] = '\0';
    printf("CC block sig: %s (%d decimal) x: %d y: %d width: %d height: %d "
           "angle: %d index: %d age: %d\n",
           sig, b->m_signature, b->m_x, b->m_y, b->m_width, b->m_height,
           b->m_angle, b->m_index, b->m_age);
  } else // regular block.  Note, angle is always zero, so no need to print
    printf("sig: %d x: %d y: %d width: %d height: %d index: %d age: %d\n",
           b->m_signature, b->m_x, b->m_y, b->m_width, b->m_height, b->m_index,
           b->m_age);
}

int8_t pixy_get_blocks(pixy_t *pixy, bool wait, uint8_t sigmap,
                       uint8_t max_blocks) {

  while (true) {
    // fill in request data
    pixy->m_buf_payload[0] = sigmap;
    pixy->m_buf_payload[1] = max_blocks;
    pixy->m_length = 2;
    pixy->m_type = CCC_REQUEST_BLOCKS;

    // send request
    send_packet(pixy);
    if (recv_packet(pixy) == PIXY_RESULT_OK) {
      if (pixy->m_type == CCC_RESPONSE_BLOCKS) {
        pixy->blocks = (pixy_block_t *)pixy->m_buf;
        pixy->num_blocks = pixy->m_length / sizeof(pixy_block_t);
        return pixy->num_blocks;
      }
      // deal with busy and program changing states from Pixy (we'll wait)
      else if (pixy->m_type == PIXY_TYPE_RESPONSE_ERROR) {
        if ((int8_t)pixy->m_buf[0] == PIXY_RESULT_BUSY) {
          if (!wait)
            return PIXY_RESULT_BUSY; // new data not available yet
        } else if ((int8_t)pixy->m_buf[0] != PIXY_RESULT_PROG_CHANGING)
          return pixy->m_buf[0];
      }
    } else
      return PIXY_RESULT_ERROR; // some kind of bitstream error

    // If we're waiting for frame data, don't thrash Pixy with requests.
    // We can give up half a millisecond of latency (worst case)
    nrf_delay_ms(500);
  }
}
