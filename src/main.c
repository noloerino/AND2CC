
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "pixy2.h"

#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"

void check_status(int8_t code, const char *label, bool print_on_success) {
  if (code != PIXY_RESULT_OK)
    printf("%s failed with %d\n", label, code);
  else if (print_on_success)
    printf("%s succeeded\n", label);
}

pixy_block_t *select_block(pixy_block_t *blocks, int8_t num_blocks) {
  if (num_blocks <= 0)
    return NULL;
  // pick oldest one
  uint8_t max_age = 0;
  pixy_block_t *block = NULL;
  for (int8_t i = 0; i < num_blocks; ++i) {
    if (blocks[i].m_age >= max_age) {
      max_age = blocks[i].m_age;
      block = &blocks[i];
    }
  }
  return block;
}

pixy_block_t *get_block(pixy_block_t *blocks, int8_t num_blocks, uint8_t index) {
  for (int8_t i = 0; i < num_blocks; ++i) {
    printf("block index: %d\n", blocks[i].m_index);
    if (blocks[i].m_index == index) {
      return &blocks[i];
    }
  }
  return NULL;
}

float clip(float f, float low, float high) {
  if (f < low)
    f = low;
  if (f > high)
    f = high;
  return f;
}

pixy_t *pixy;

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");

  // initialize spi master
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };

  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  nrf_delay_ms(10);

  nrf_drv_spi_t pixy_spi = NRF_DRV_SPI_INSTANCE(2);
  nrf_drv_spi_config_t pixy_spi_config = {
    .sck_pin = BUCKLER_SD_SCLK,
    .mosi_pin = BUCKLER_SD_MOSI,
    .miso_pin = BUCKLER_SD_MISO,
    .ss_pin = BUCKLER_SD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_3,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };

  error_code = nrf_drv_spi_init(&pixy_spi, &pixy_spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  nrf_delay_ms(10);

  // initialize display driver
   display_init(&spi_instance);
   printf("Display initialized\n");
   nrf_delay_ms(10);
   display_write("disp init", 0);

  check_status(pixy_init(&pixy, &pixy_spi), "initialize", true);
  pixy_print_version(pixy->version);

  check_status(pixy_set_led(pixy, 0, 255, 0), "set led", true);

  check_status(pixy_get_resolution(pixy), "get resolution", true);
  printf("resolution: %d x %d\n", pixy->frame_width, pixy->frame_height);
  
  check_status(pixy_set_lamp(pixy, 255, 255), "set lamp", true);

  kobukiInit();
  printf("Kobuki initialized\n");

  KobukiSensors_t sensors = {0};

  const float speed_target = 100;
  float speed_left = 0;
  float speed_right = 0;
  const float base_width = 140;
  const float k_p = 2.0;
  const float decay =  0.95;

  float angle = 0;
  float new_factor = 0.5;
  
  while(true) {
    kobukiSensorPoll(&sensors);
    kobukiDriveDirect((int16_t)speed_left, (int16_t)speed_right);

    int8_t ec = pixy_get_blocks(pixy, false, CCC_SIG_ALL, CCC_MAX_BLOCKS);
    if (ec < 0) {
      //printf("get blocks error: %d\n", ec);
    } else {
      //printf("got %d blocks\n", ec);
    }
    if (pixy->num_blocks > 0) {
      pixy_block_t *block = select_block(pixy->blocks, pixy->num_blocks);
      if (block->m_x <= pixy->frame_width && block->m_y <= pixy->frame_height) {
        const float new_angle = ((M_PI / 3.0) / pixy->frame_width) * block->m_x - (M_PI / 6.0);
        angle = (1-new_factor) * angle + new_factor * new_angle;
        float delta = (base_width / 2.0) * k_p * angle;
        speed_left = -speed_target + delta;
        speed_right = -speed_target - delta; 
      }
    }

    speed_left *= decay;
    speed_right *= decay;

    int16_t v_left = (int16_t)clip(speed_left, INT16_MIN, INT16_MAX);
    int16_t v_right = (int16_t)clip(speed_right, INT16_MIN, INT16_MAX);
    printf("angle: %f    speed: %d    %d\n", angle * 180.0 / M_PI, v_left, v_right);

    nrf_delay_ms(10);
  }
}

