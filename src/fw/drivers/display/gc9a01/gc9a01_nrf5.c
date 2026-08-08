/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "drivers/display/display.h"
#include "gc9a01.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "applib/graphics/gtypes.h"
#include "board/board.h"
#include "drivers/gpio.h"
#include "kernel/events.h"
#include "kernel/util/stop.h"
#include "os/mutex.h"
#include "system/passert.h"
#include "util/reverse.h"

#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_rtc.h>
#include <nrfx_gppi.h>
#include <nrfx_spim.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define DISP_MODE_WRITE 0x2CU
#define DISP_MODE_CLEAR 0x04U

static uint8_t s_buf[((DISP_LINE_BYTES*3/2))];
static bool s_updating_single_byte;
static bool s_updating;
static NextRowCallback s_nrcb;
static UpdateCompleteCallback s_uccb;
static SemaphoreHandle_t s_sem;

// watch rotation
static bool s_rotated_180 = false;


static inline void prv_enable_spim(void) {
  nrf_spim_enable(BOARD_CONFIG_DISPLAY.spi.p_reg);
}

static inline void prv_disable_spim(void) {
  nrf_spim_disable(BOARD_CONFIG_DISPLAY.spi.p_reg);

  // Workaround for nRF52840 anomaly 195
  if (BOARD_CONFIG_DISPLAY.spi.p_reg == NRF_SPIM3) {
    *(volatile uint32_t *)0x4002F004 = 1;
  }
}

// the gc9a01 cs pin is active low, so we have to reverse these functions
static inline void prv_enable_chip_select(void){
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs,false);
}
static inline void prv_disable_chip_select(void){
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs,true);
}
// reset is also active low
static inline void prv_enable_reset(void){
  gpio_output_set(&BOARD_CONFIG_DISPLAY.rst,false);
}
static inline void prv_disable_reset(void){
  gpio_output_set(&BOARD_CONFIG_DISPLAY.rst,true);
}

static inline void prv_select_data_mode(void){
  gpio_output_set(&BOARD_CONFIG_DISPLAY.dc,true);
}
static inline void prv_select_command_mode(void){
  gpio_output_set(&BOARD_CONFIG_DISPLAY.dc,false);
}

// our functions
void GC9A01_write_command(uint8_t cmd){
  prv_enable_spim();
  prv_enable_chip_select();
  prv_select_command_mode();
  uint8_t buf[] = {cmd};
  nrfx_spim_xfer_desc_t desc = {.p_tx_buffer = buf, .tx_length = sizeof(buf)};
  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi,&desc,0);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  xSemaphoreTake(s_sem, portMAX_DELAY);
  prv_disable_chip_select();
  prv_disable_spim();
}
void GC9A01_write_byte(uint8_t cmd){
  prv_enable_spim();
  prv_enable_chip_select();
  prv_select_data_mode();
  uint8_t buf[] = {cmd};
  nrfx_spim_xfer_desc_t desc = {.p_tx_buffer = buf, .tx_length = sizeof(buf)};
  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi,&desc,0);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  xSemaphoreTake(s_sem, portMAX_DELAY);
  prv_disable_chip_select();
  prv_disable_spim();
}

static void prv_gc9a01_init(void) {
  prv_disable_chip_select();
  prv_enable_reset();
  vTaskDelay(pdMS_TO_TICKS(100));
  prv_disable_reset();
  vTaskDelay(pdMS_TO_TICKS(120));
  GC9A01_write_command(0xFE); 
  GC9A01_write_command(0xEF); 

  GC9A01_write_command(0xEB); 
  GC9A01_write_byte(0x14); 

  GC9A01_write_command(0x84); 
  GC9A01_write_byte(0x40);

  GC9A01_write_command(0x85); 
  GC9A01_write_byte(0xFF); 

  GC9A01_write_command(0x86); 
  GC9A01_write_byte(0xFF); 

  GC9A01_write_command(0x87); 
  GC9A01_write_byte(0xFF); 

  GC9A01_write_command(0x88); 
  GC9A01_write_byte(0x0A); 

  GC9A01_write_command(0x89); 
  GC9A01_write_byte(0x21); 

  GC9A01_write_command(0x8A); 
  GC9A01_write_byte(0x00); 

  GC9A01_write_command(0x8B); 
  GC9A01_write_byte(0x80); 

  GC9A01_write_command(0x8C); 
  GC9A01_write_byte(0x01); 

  GC9A01_write_command(0x8D); 
  GC9A01_write_byte(0x01); 

  GC9A01_write_command(0x8E); 
  GC9A01_write_byte(0xFF); 

  GC9A01_write_command(0x8F); 
  GC9A01_write_byte(0xFF); 

  GC9A01_write_command(0xB6); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x00); 

  //orientation
  GC9A01_write_command(0x36); 
  GC9A01_write_byte(0x48); 

  GC9A01_write_command(0x3a); 
  GC9A01_write_byte(0x03); 

  GC9A01_write_command(0x90); 
  GC9A01_write_byte(0x08); 
  GC9A01_write_byte(0x08); 
  GC9A01_write_byte(0x08); 
  GC9A01_write_byte(0x08); 

  GC9A01_write_command(0xBD); 
  GC9A01_write_byte(0x06); 

  GC9A01_write_command(0xBC); 
  GC9A01_write_byte(0x00); 

  GC9A01_write_command(0xFF); 
  GC9A01_write_byte(0x60); 
  GC9A01_write_byte(0x01); 
  GC9A01_write_byte(0x04); 

  GC9A01_write_command(0xC3); 
  GC9A01_write_byte(0x13); 
  GC9A01_write_command(0xC4); 
  GC9A01_write_byte(0x13); 

  GC9A01_write_command(0xC9); 
  GC9A01_write_byte(0x22); 

  GC9A01_write_command(0xBE); 
  GC9A01_write_byte(0x11); 

  GC9A01_write_command(0xE1); 
  GC9A01_write_byte(0x10); 
  GC9A01_write_byte(0x0E); 

  GC9A01_write_command(0xDF); 
  GC9A01_write_byte(0x21); 
  GC9A01_write_byte(0x0c); 
  GC9A01_write_byte(0x02); 

  GC9A01_write_command(0xF0); 
  GC9A01_write_byte(0x45); 
  GC9A01_write_byte(0x09); 
  GC9A01_write_byte(0x08); 
  GC9A01_write_byte(0x08); 
  GC9A01_write_byte(0x26); 
  GC9A01_write_byte(0x2A); 

  GC9A01_write_command(0xF1); 
  GC9A01_write_byte(0x43); 
  GC9A01_write_byte(0x70); 
  GC9A01_write_byte(0x72); 
  GC9A01_write_byte(0x36); 
  GC9A01_write_byte(0x37); 
  GC9A01_write_byte(0x6F); 

  GC9A01_write_command(0xF2); 
  GC9A01_write_byte(0x45); 
  GC9A01_write_byte(0x09); 
  GC9A01_write_byte(0x08); 
  GC9A01_write_byte(0x08); 
  GC9A01_write_byte(0x26); 
  GC9A01_write_byte(0x2A); 

  GC9A01_write_command(0xF3); 
  GC9A01_write_byte(0x43); 
  GC9A01_write_byte(0x70); 
  GC9A01_write_byte(0x72); 
  GC9A01_write_byte(0x36); 
  GC9A01_write_byte(0x37); 
  GC9A01_write_byte(0x6F); 

  GC9A01_write_command(0xED); 
  GC9A01_write_byte(0x1B); 
  GC9A01_write_byte(0x0B); 

  GC9A01_write_command(0xAE); 
  GC9A01_write_byte(0x77); 

  GC9A01_write_command(0xCD); 
  GC9A01_write_byte(0x63); 

  GC9A01_write_command(0x70); 
  GC9A01_write_byte(0x07); 
  GC9A01_write_byte(0x07); 
  GC9A01_write_byte(0x04); 
  GC9A01_write_byte(0x0E); 
  GC9A01_write_byte(0x0F); 
  GC9A01_write_byte(0x09); 
  GC9A01_write_byte(0x07); 
  GC9A01_write_byte(0x08); 
  GC9A01_write_byte(0x03); 

  GC9A01_write_command(0xE8); 
  GC9A01_write_byte(0x34); 

  GC9A01_write_command(0x62); 
  GC9A01_write_byte(0x18); 
  GC9A01_write_byte(0x0D); 
  GC9A01_write_byte(0x71); 
  GC9A01_write_byte(0xED); 
  GC9A01_write_byte(0x70); 
  GC9A01_write_byte(0x70); 
  GC9A01_write_byte(0x18); 
  GC9A01_write_byte(0x0F); 
  GC9A01_write_byte(0x71); 
  GC9A01_write_byte(0xEF); 
  GC9A01_write_byte(0x70); 
  GC9A01_write_byte(0x70); 

  GC9A01_write_command(0x63); 
  GC9A01_write_byte(0x18); 
  GC9A01_write_byte(0x11); 
  GC9A01_write_byte(0x71); 
  GC9A01_write_byte(0xF1); 
  GC9A01_write_byte(0x70); 
  GC9A01_write_byte(0x70); 
  GC9A01_write_byte(0x18); 
  GC9A01_write_byte(0x13); 
  GC9A01_write_byte(0x71); 
  GC9A01_write_byte(0xF3); 
  GC9A01_write_byte(0x70); 
  GC9A01_write_byte(0x70); 

  GC9A01_write_command(0x64); 
  GC9A01_write_byte(0x28); 
  GC9A01_write_byte(0x29); 
  GC9A01_write_byte(0xF1); 
  GC9A01_write_byte(0x01); 
  GC9A01_write_byte(0xF1); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x07); 

  GC9A01_write_command(0x66); 
  GC9A01_write_byte(0x3C); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0xCD); 
  GC9A01_write_byte(0x67); 
  GC9A01_write_byte(0x45); 
  GC9A01_write_byte(0x45); 
  GC9A01_write_byte(0x10); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x00); 

  GC9A01_write_command(0x67); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x3C); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x01); 
  GC9A01_write_byte(0x54); 
  GC9A01_write_byte(0x10); 
  GC9A01_write_byte(0x32); 
  GC9A01_write_byte(0x98); 

  GC9A01_write_command(0x74); 
  GC9A01_write_byte(0x10); 
  GC9A01_write_byte(0x85); 
  GC9A01_write_byte(0x80); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x00); 
  GC9A01_write_byte(0x4E); 
  GC9A01_write_byte(0x00); 

  GC9A01_write_command(0x98); 
  GC9A01_write_byte(0x3e); 
  GC9A01_write_byte(0x07); 

  GC9A01_write_command(0x35);
  GC9A01_write_command(0x21);
  vTaskDelay(pdMS_TO_TICKS(120));
  GC9A01_write_command(0x11);
  vTaskDelay(pdMS_TO_TICKS(20));
  GC9A01_write_command(0x53);
  GC9A01_write_byte(0x2C);
  GC9A01_write_command(0x2C);
  for (int i = 0; i < 240*240*12/8; i++){
    GC9A01_write_byte(0x00);
  }
  GC9A01_write_command(0x29);

  //set frame to 180x180. I know this isn't Pebble-compliant code but hey, this is my repo i do whatever i want
  uint8_t data[4];

  GC9A01_write_command(0x2a);
  data[0] = (30 >> 8) & 0xFF;
  data[1] = 30 & 0xFF;
  data[2] = (209 >> 8) & 0xFF;
  data[3] = 209 & 0xFF;
  for (int i=0;i<4;i++){
    GC9A01_write_byte(data[i]);
  }

  GC9A01_write_command(0x2b);

  for (int i=0;i<4;i++){
    GC9A01_write_byte(data[i]);
  }
}

static void prv_terminate_single_transfer(void *data) {
  s_updating_single_byte = false;

  prv_disable_chip_select();
  prv_disable_spim();

  //s_uccb();
}
static void prv_terminate_transfer(void *data) {
  s_updating = false;

  prv_disable_chip_select();
  prv_disable_spim();

  s_uccb();
}

static void prv_transfer_next_row(void *data){
  DisplayRow row;
  uint8_t *pbuf = s_buf;
  nrfx_spim_xfer_desc_t desc = {.p_tx_buffer = pbuf};
  if (!s_nrcb(&row)){
    prv_terminate_transfer(NULL);
    return;
  }
  const GBitmapDataRowInfoInternal *row_infos = g_gbitmap_spalding_data_row_infos;
  for (int i = 0; i < DISP_LINE_BYTES; i++) {
    uint8_t r,g,b;
    if (i < row_infos[row.address].min_x || i > row_infos[row.address].max_x) {
      r = 0;
      g = 0;
      b = 0;
    } else {
      r = (row.data[i] >> 4) & 0b11;
      g = (row.data[i] >> 2) & 0b11;
      b = (row.data[i]) & 0b11;
    }
    if (i % 2 == 0){
      *pbuf++ = (r << 6) | (r << 4) | (g << 2) | g;
      *pbuf = (b << 6) | (b << 4);
    } else {
      *pbuf++ |= (r << 2) | r;
      *pbuf++ = (g << 6) | (g << 4) | (b << 2) | b;
    }
  }
  desc.tx_length = DISP_LINE_BYTES * 3 / 2;
  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &desc, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);
}

static void prv_spim_evt_handler(nrfx_spim_evt_t const *evt, void *ctx) {
  portBASE_TYPE woken = pdFALSE;

  if (s_updating_single_byte) {
    PebbleEvent e = {
        .type = PEBBLE_CALLBACK_EVENT,
        .callback =
            {
                .callback = prv_terminate_single_transfer,
            },
    };

    woken = event_put_isr(&e) ? pdTRUE : pdFALSE;
  } else if (s_updating) {
    PebbleEvent e = {
        .type = PEBBLE_CALLBACK_EVENT,
        .callback =
            {
                .callback = prv_transfer_next_row,
            },
    };

    woken = event_put_isr(&e) ? pdTRUE : pdFALSE;
  } else {
    xSemaphoreGiveFromISR(s_sem, &woken);
  }

  portEND_SWITCHING_ISR(woken);
}

void display_init(void) {
  nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(
      BOARD_CONFIG_DISPLAY.clk.gpio_pin, BOARD_CONFIG_DISPLAY.mosi.gpio_pin,
      NRF_SPIM_PIN_NOT_CONNECTED, NRF_SPIM_PIN_NOT_CONNECTED);
  config.frequency = NRFX_MHZ_TO_HZ(16);
  config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  nrfx_err_t err = nrfx_spim_init(&BOARD_CONFIG_DISPLAY.spi, &config, prv_spim_evt_handler, NULL);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  gpio_output_init(&BOARD_CONFIG_DISPLAY.cs, GPIO_OType_PP, GPIO_Speed_50MHz);
  gpio_output_init(&BOARD_CONFIG_DISPLAY.dc, GPIO_OType_PP, GPIO_Speed_50MHz);
  gpio_output_init(&BOARD_CONFIG_DISPLAY.rst, GPIO_OType_PP, GPIO_Speed_50MHz);

  s_sem = xSemaphoreCreateBinary();

  prv_gc9a01_init();
}

void display_clear(void) {
}

void display_set_enabled(bool enabled) {
}

void display_set_rotated(bool rotated) {
  s_rotated_180 = rotated;
}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  PBL_ASSERTN(!s_updating);
  s_uccb = uccb;
  s_nrcb = nrcb;

  // write command (write)
  GC9A01_write_command(DISP_MODE_WRITE);
  s_updating = true;
  prv_enable_spim();
  prv_enable_chip_select();
  prv_select_data_mode(); 
  prv_transfer_next_row(NULL);
}

bool display_update_in_progress(void) {
  return s_updating;
}

/* stubs */

uint32_t display_baud_rate_change(uint32_t new_frequency_hz) {
  return new_frequency_hz;
}

void display_pulse_vcom(void) {}

void display_show_splash_screen(void) {}

void display_set_offset(GPoint offset) {}

GPoint display_get_offset(void) {
  return GPointZero;
}
