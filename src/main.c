
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_atfifo.h"
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


#define ANGLE_DECAY  0.4
#define ANGLE_K_P    2.0
#define SPEED_TARGET 60  // mm/s
#define CHASSIS_BASE_WIDTH 140 // mm
#define TARGET_FAIL_COUNT_THRESHOLD 50

#define DOCK_POWER NRF_GPIO_PIN_MAP(0, 3)
#define DOCK_DETECT NRF_GPIO_PIN_MAP(0, 4)

typedef enum {
  OFF,
  SPIN,
  TARGET,
  DOCKED,
} robot_state_t;

void pixy_error_check(int8_t code, const char *label, bool print_on_success) {
  if (code != PIXY_RESULT_OK)
    printf("%s failed with %d\n", label, code);
  else if (print_on_success)
    printf("%s succeeded\n", label);
}

pixy_block_t *select_block(pixy_block_t *blocks, int8_t num_blocks, uint16_t frame_width, uint16_t frame_height) {
  if (num_blocks <= 0)
    return NULL;
  // Prioritize 
  uint8_t max_age = 0;
  pixy_block_t *block = NULL;
  uint8_t sig = 0;
  for (int8_t i = 0; i < num_blocks; ++i) {
    pixy_print_block(&blocks[i]);
    if (sig == CCC_SIG2 && blocks[i].m_signature != CCC_SIG2) {
      continue;
    }
    if (blocks[i].m_age >= max_age && blocks[i].m_x <= frame_width && blocks[i].m_y <= frame_height) {
      max_age = blocks[i].m_age;
      block = &blocks[i];
      sig = blocks[i].m_signature;
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
  display_write("Hello, I'm " DDD_ROBOT_ID_STR, 0);

  pixy_t *pixy;
  pixy_error_check(pixy_init(&pixy, &pixy_spi), "initialize", true);
  pixy_print_version(pixy->version);

  pixy_error_check(pixy_set_led(pixy, 0, 255, 0), "set led", true);

  pixy_error_check(pixy_get_resolution(pixy), "get resolution", true);
  printf("resolution: %d x %d\n", pixy->frame_width, pixy->frame_height);
  
  pixy_error_check(pixy_set_lamp(pixy, 100, 100), "set lamp", true);

  kobukiInit();
  printf("Kobuki initialized\n");

  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};
  float speed_left = 0;
  float speed_right = 0;
  float angle = 0;
  uint32_t target_fail_count = 0;

  // Initialize all 3 LEDs
  // LED 0 (25) will be used for reaching of initial docking state
  // LED 1 (24) will display continuity
  // LED 2 (23) will be a BLE test thing
  nrf_gpio_cfg_output(BUCKLER_LED0);
  nrf_gpio_cfg_output(BUCKLER_LED1);
  nrf_gpio_cfg_output(BUCKLER_LED2);
  // Need to set them high to turn them off
  nrf_gpio_pin_set(BUCKLER_LED0);
  nrf_gpio_pin_set(BUCKLER_LED1);
  nrf_gpio_pin_set(BUCKLER_LED2);

  // Initialize docking continuity pins
  nrf_gpio_cfg_output(DOCK_POWER);
  nrf_gpio_pin_clear(DOCK_POWER);
  nrf_gpio_cfg_input(DOCK_DETECT, NRF_GPIO_PIN_PULLUP);
  
  while(true) {
    kobukiSensorPoll(&sensors);

    // Set speeds based on speed_left and speed_right.
    int16_t v_left = 0;
    int16_t v_right = 0;
    //if (fabs(speed_left) > 0)
    v_left = (int16_t)clip(speed_left, INT16_MIN, INT16_MAX);
    //if (fabs(speed_right) > 30) 
    v_right = (int16_t)clip(speed_right, INT16_MIN, INT16_MAX);
    kobukiDriveDirect(v_left, v_right);

    // Input configured to pull up, so pin reads zero when docked
    bool docked = !nrf_gpio_pin_read(DOCK_DETECT);
    if (docked) {
      nrf_gpio_pin_clear(BUCKLER_LED0);
    } else {
      nrf_gpio_pin_set(BUCKLER_LED0);
    }

    switch (state) {
      case OFF: {
        display_write("OFF", 0);
        speed_left = 0;
        speed_right = 0;
        
        if (button_pressed) {
          state = SPIN;
          printf("OFF -> SPIN\n");
        }
        break;
      }
      case SPIN: {
        display_write("SPIN", 0);
        speed_left = -60;
        speed_right = 60;
        int8_t ec = pixy_get_blocks(pixy, false, CCC_SIG1 | CCC_SIG2, CCC_MAX_BLOCKS);
        if (ec < 0) {
          printf("failed to get blocks with error code %d\n", ec);
        } else {
          //printf("got %d blocks\n", ec);
        }
        pixy_block_t *block = select_block(pixy->blocks, pixy->num_blocks, pixy->frame_width, pixy->frame_height);
        if (block != NULL) {
          state = TARGET;
          target_fail_count = 0;
          printf("SPIN -> TARGET\n");
        }
        break;
      }
      case TARGET: {
        display_write("TARGET", 0);
        pixy_get_blocks(pixy, false, CCC_SIG1 | CCC_SIG2, CCC_MAX_BLOCKS);
        pixy_block_t *block = select_block(pixy->blocks, pixy->num_blocks, pixy->frame_width, pixy->frame_height);
        if (docked) {
          speed_left = 0;
          speed_right = 0;
          // Turn on LED1 to indicate that we've at least docked once now
          nrf_gpio_pin_clear(BUCKLER_LED1);
          ddd_ble_init();
          nrf_atfifo_t *ble_cmd_q = get_ble_cmd_q();
          state = DOCKED;
          printf("TARGET -> DOCKED\n");
        } else if (block != NULL) {
          const float new_angle = ((M_PI / 3.0) / pixy->frame_width) * block->m_x - (M_PI / 6.0);
          angle = ANGLE_DECAY * angle + (1 - ANGLE_DECAY) * new_angle;
          const float delta = (CHASSIS_BASE_WIDTH / 2.0) * ANGLE_K_P * angle;
          speed_left = -SPEED_TARGET + delta;
          speed_right = -SPEED_TARGET - delta; 
          target_fail_count = 0;
        } else {
          ++target_fail_count;
          if (target_fail_count > TARGET_FAIL_COUNT_THRESHOLD) {
            state = SPIN;
            printf("TARGET -> SPIN\n");
          }
        }
        break;
      }
      case DOCKED: {
        display_write("DOCKED", 0);
        // Check the command queue for a message
        // TODO for the time being, the arrival of a BLE command will override whatever's happening
        // in the rest of the main loop
        // Eventually, these will need to be integrated together
        ddd_ble_cmd_t cmd = { 0 };
        const int16_t TEST_DRV_SPD = 70;
        if (nrf_atfifo_get_free(ble_cmd_q, &cmd, sizeof(ddd_ble_cmd_t), NULL) != NRF_ERROR_NOT_FOUND) {
          switch (cmd) {
            case DDD_BLE_LED_ON: {
              display_write("[ble] LED ON", 1);
              nrf_gpio_pin_clear(BUCKLER_LED2);
              break;
            }
            case DDD_BLE_LED_OFF: {
              display_write("[ble] LED OFF", 1);
              nrf_gpio_pin_set(BUCKLER_LED2);
              break;
            }
            case DDD_BLE_DRV_LEFT: {
              display_write("[ble] LEFT", 1);
              speed_left = -TEST_DRV_SPD;
              speed_right = TEST_DRV_SPD;
              break;
            }
            case DDD_BLE_DRV_RIGHT: {
              display_write("[ble] RIGHT", 1);
              speed_left = TEST_DRV_SPD;
              speed_right = -TEST_DRV_SPD;
              break;
            }
            case DDD_BLE_DRV_FORWARD: {
              display_write("[ble] FORWARD", 1);
              if (DDD_ROBOT_ID == 0) {
                speed_left = TEST_DRV_SPD;
                speed_right = TEST_DRV_SPD;
              } else {
                speed_left = -TEST_DRV_SPD;
                speed_right = -TEST_DRV_SPD;
              }
              break;
            }
            case DDD_BLE_DRV_BACKWARD: {
              display_write("[ble] BACKWARD", 1);
              if (DDD_ROBOT_ID == 0) {
                speed_left = -TEST_DRV_SPD;
                speed_right = -TEST_DRV_SPD;
              } else {
                speed_left = TEST_DRV_SPD;
                speed_right = TEST_DRV_SPD;
              }
              break; 
            }
            case DDD_BLE_DRV_ZERO: {
              display_write("[ble] ZERO", 1);
              speed_left = 0.0;
              speed_right = 0.0;
              break;
            }
            default:
              printf("Unhandled command\n");
              display_write("[ble] INVALID", 1);
              speed_left = 0.0;
              speed_right = 0.0;
              break;
          }
        }
      }
      default: {
        display_write("INVALID STATE", 0);
        printf("error: default state\n");
      }
    }

    nrf_delay_ms(10);
  }
}

