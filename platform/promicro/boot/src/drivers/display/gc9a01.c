#include "drivers/dbgserial.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <board.h>
#include <util/misc.h>

#include <nrfx_spim.h>

// Bootloader images
#include "drivers/display/resources/hex_digits.h"
#include "drivers/display/resources/dead_face.xbm"
#include "drivers/display/resources/empty_bar.xbm"
#include "drivers/display/resources/pebbleos_logo.xbm"
#include "nrf52840.h"

#define DISP_COLS 180
#define DISP_ROWS 180
#define PBL_DISP_SHAPE_ROUND

#define DISP_LINE_BYTES (23) // 180 pixels go into 23 bytes, with the last byte only carrying 4 bits of data MSB
#define DISP_LINE_WORDS (((DISP_COLS - 1) / 32) + 1)

// GPIO constants
#define DISP_MODE_WRITE (0x2C)

static uint8_t line_buffer[300];

static nrfx_spim_t spim = NRFX_SPIM_INSTANCE(3);
static nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(
	BOARD_DISP_SCK_PIN, BOARD_DISP_MOSI_PIN, NRF_SPIM_PIN_NOT_CONNECTED,
	NRF_SPIM_PIN_NOT_CONNECTED
);

static void prv_enable_chip_select(void) {
  //GPIO_WriteBit(DISP_GPIO, DISP_PIN_SCS, Bit_SET);
  //BOARD_DISP_CS_PORT->OUTSET = (1 << BOARD_DISP_CS_PIN);
  // required setup time > 3us
  //NRFX_DELAY_US(7);
}

static void prv_disable_chip_select(void) {
  // delay while last byte is emitted by the SPI peripheral
  //NRFX_DELAY_US(7);

  //GPIO_WriteBit(DISP_GPIO, DISP_PIN_SCS, Bit_RESET);
  //BOARD_DISP_CS_PORT->OUTCLR = (1 << BOARD_DISP_CS_PIN);
  // hold time > 1us
  // produce a delay 4ms
  //NRFX_DELAY_US(4);
}

//! These functions needed to be called around any commands that
//! are sent to the display. NOINLINE only for code size savings.
static void prv_enable_display_access(void) {
  prv_enable_chip_select();
}

static void prv_disable_display_access(void) {
  prv_disable_chip_select();
}

static void prv_enable_reset(void){
  BOARD_DISP_RST_PORT->OUTCLR = (1 << BOARD_DISP_RST_PIN);
}
static void prv_disable_reset(void){
  BOARD_DISP_RST_PORT->OUTSET = (1 << BOARD_DISP_RST_PIN);
}
static void prv_enable_command(void){
  BOARD_DISP_DC_PORT->OUTCLR = (1 << BOARD_DISP_DC_PIN);
}
static void prv_enable_data(void){
  BOARD_DISP_DC_PORT->OUTSET = (1 << BOARD_DISP_DC_PIN);
}

//! Write a single byte synchronously to the display. This is the only practical
//! way to write to the display in the bootloader since we don't have interrupts.
static void prv_display_write_byte(uint8_t d) {
  prv_enable_data();
  nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(&d, 1);
  nrfx_spim_xfer(&spim, &desc, 0);
  // Block until the tx buffer is empty
  //SPI_I2S_SendData(DISP_SPI, d);
  //while (!SPI_I2S_GetFlagStatus(DISP_SPI, SPI_I2S_FLAG_TXE)) {}
}
static void prv_display_write_command(uint8_t d) {
  prv_enable_command();
  nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(&d, 1);
  nrfx_spim_xfer(&spim, &desc, 0);
  // Block until the tx buffer is empty
  //SPI_I2S_SendData(DISP_SPI, d);
  //while (!SPI_I2S_GetFlagStatus(DISP_SPI, SPI_I2S_FLAG_TXE)) {}
}

// Since all these values are constant we can save code space
// by storing the initialized struct in memory rather than
// needing to copy in each value
/*
static SPI_InitTypeDef s_disp_spi_init = {
  .SPI_Direction = SPI_Direction_1Line_Tx, // Write-only SPI
  .SPI_Mode = SPI_Mode_Master,
  .SPI_DataSize = SPI_DataSize_8b,
  .SPI_CPOL = SPI_CPOL_Low,
  .SPI_CPHA = SPI_CPHA_1Edge,
  .SPI_NSS = SPI_NSS_Soft,
  // We want the SPI clock to run at 2MHz
  .SPI_BaudRatePrescaler = DISPLAY_PERIPH_PRESCALER,
  // MSB order allows us to write pixels out without reversing bytes, but command bytes
  // have to be reversed
  .SPI_FirstBit = SPI_FirstBit_MSB,
  .SPI_CRCPolynomial = 7 // default
};
*/

static void prv_display_start(void) {
  // Hold LCD on
  // GPIO_WriteBit(DISP_LCD_GPIO, DISP_PIN_LCD, Bit_SET);
  //BOARD_DISP_D  // All of this is required for the display to be properly initialised. Yeah, I know.
  prv_enable_reset();
  NRFX_DELAY_US(100000);
  prv_disable_reset();
  NRFX_DELAY_US(120000);
  prv_display_write_command(0xFE); 
  prv_display_write_command(0xEF); 

  prv_display_write_command(0xEB); 
  prv_display_write_byte(0x14); 

  prv_display_write_command(0x84); 
  prv_display_write_byte(0x40);

  prv_display_write_command(0x85); 
  prv_display_write_byte(0xFF); 

  prv_display_write_command(0x86); 
  prv_display_write_byte(0xFF); 

  prv_display_write_command(0x87); 
  prv_display_write_byte(0xFF); 

  prv_display_write_command(0x88); 
  prv_display_write_byte(0x0A); 

  prv_display_write_command(0x89); 
  prv_display_write_byte(0x21); 

  prv_display_write_command(0x8A); 
  prv_display_write_byte(0x00); 

  prv_display_write_command(0x8B); 
  prv_display_write_byte(0x80); 

  prv_display_write_command(0x8C); 
  prv_display_write_byte(0x01); 

  prv_display_write_command(0x8D); 
  prv_display_write_byte(0x01); 

  prv_display_write_command(0x8E); 
  prv_display_write_byte(0xFF); 

  prv_display_write_command(0x8F); 
  prv_display_write_byte(0xFF); 

  prv_display_write_command(0xB6); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x00); 

  //orientation
  prv_display_write_command(0x36); 
  prv_display_write_byte(0x48); 

  prv_display_write_command(0x3a); 
  prv_display_write_byte(0x03); 

  prv_display_write_command(0x90); 
  prv_display_write_byte(0x08); 
  prv_display_write_byte(0x08); 
  prv_display_write_byte(0x08); 
  prv_display_write_byte(0x08); 

  prv_display_write_command(0xBD); 
  prv_display_write_byte(0x06); 

  prv_display_write_command(0xBC); 
  prv_display_write_byte(0x00); 

  prv_display_write_command(0xFF); 
  prv_display_write_byte(0x60); 
  prv_display_write_byte(0x01); 
  prv_display_write_byte(0x04); 

  prv_display_write_command(0xC3); 
  prv_display_write_byte(0x13); 
  prv_display_write_command(0xC4); 
  prv_display_write_byte(0x13); 

  prv_display_write_command(0xC9); 
  prv_display_write_byte(0x22); 

  prv_display_write_command(0xBE); 
  prv_display_write_byte(0x11); 

  prv_display_write_command(0xE1); 
  prv_display_write_byte(0x10); 
  prv_display_write_byte(0x0E); 

  prv_display_write_command(0xDF); 
  prv_display_write_byte(0x21); 
  prv_display_write_byte(0x0c); 
  prv_display_write_byte(0x02); 

  prv_display_write_command(0xF0); 
  prv_display_write_byte(0x45); 
  prv_display_write_byte(0x09); 
  prv_display_write_byte(0x08); 
  prv_display_write_byte(0x08); 
  prv_display_write_byte(0x26); 
  prv_display_write_byte(0x2A); 

  prv_display_write_command(0xF1); 
  prv_display_write_byte(0x43); 
  prv_display_write_byte(0x70); 
  prv_display_write_byte(0x72); 
  prv_display_write_byte(0x36); 
  prv_display_write_byte(0x37); 
  prv_display_write_byte(0x6F); 

  prv_display_write_command(0xF2); 
  prv_display_write_byte(0x45); 
  prv_display_write_byte(0x09); 
  prv_display_write_byte(0x08); 
  prv_display_write_byte(0x08); 
  prv_display_write_byte(0x26); 
  prv_display_write_byte(0x2A); 

  prv_display_write_command(0xF3); 
  prv_display_write_byte(0x43); 
  prv_display_write_byte(0x70); 
  prv_display_write_byte(0x72); 
  prv_display_write_byte(0x36); 
  prv_display_write_byte(0x37); 
  prv_display_write_byte(0x6F); 

  prv_display_write_command(0xED); 
  prv_display_write_byte(0x1B); 
  prv_display_write_byte(0x0B); 

  prv_display_write_command(0xAE); 
  prv_display_write_byte(0x77); 

  prv_display_write_command(0xCD); 
  prv_display_write_byte(0x63); 

  prv_display_write_command(0x70); 
  prv_display_write_byte(0x07); 
  prv_display_write_byte(0x07); 
  prv_display_write_byte(0x04); 
  prv_display_write_byte(0x0E); 
  prv_display_write_byte(0x0F); 
  prv_display_write_byte(0x09); 
  prv_display_write_byte(0x07); 
  prv_display_write_byte(0x08); 
  prv_display_write_byte(0x03); 

  prv_display_write_command(0xE8); 
  prv_display_write_byte(0x34); 

  prv_display_write_command(0x62); 
  prv_display_write_byte(0x18); 
  prv_display_write_byte(0x0D); 
  prv_display_write_byte(0x71); 
  prv_display_write_byte(0xED); 
  prv_display_write_byte(0x70); 
  prv_display_write_byte(0x70); 
  prv_display_write_byte(0x18); 
  prv_display_write_byte(0x0F); 
  prv_display_write_byte(0x71); 
  prv_display_write_byte(0xEF); 
  prv_display_write_byte(0x70); 
  prv_display_write_byte(0x70); 

  prv_display_write_command(0x63); 
  prv_display_write_byte(0x18); 
  prv_display_write_byte(0x11); 
  prv_display_write_byte(0x71); 
  prv_display_write_byte(0xF1); 
  prv_display_write_byte(0x70); 
  prv_display_write_byte(0x70); 
  prv_display_write_byte(0x18); 
  prv_display_write_byte(0x13); 
  prv_display_write_byte(0x71); 
  prv_display_write_byte(0xF3); 
  prv_display_write_byte(0x70); 
  prv_display_write_byte(0x70); 

  prv_display_write_command(0x64); 
  prv_display_write_byte(0x28); 
  prv_display_write_byte(0x29); 
  prv_display_write_byte(0xF1); 
  prv_display_write_byte(0x01); 
  prv_display_write_byte(0xF1); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x07); 

  prv_display_write_command(0x66); 
  prv_display_write_byte(0x3C); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0xCD); 
  prv_display_write_byte(0x67); 
  prv_display_write_byte(0x45); 
  prv_display_write_byte(0x45); 
  prv_display_write_byte(0x10); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x00); 

  prv_display_write_command(0x67); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x3C); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x01); 
  prv_display_write_byte(0x54); 
  prv_display_write_byte(0x10); 
  prv_display_write_byte(0x32); 
  prv_display_write_byte(0x98); 

  prv_display_write_command(0x74); 
  prv_display_write_byte(0x10); 
  prv_display_write_byte(0x85); 
  prv_display_write_byte(0x80); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x00); 
  prv_display_write_byte(0x4E); 
  prv_display_write_byte(0x00); 

  prv_display_write_command(0x98); 
  prv_display_write_byte(0x3e); 
  prv_display_write_byte(0x07); 

  prv_display_write_command(0x35);
  prv_display_write_command(0x21);
  NRFX_DELAY_US(120000);
  prv_display_write_command(0x11);
  NRFX_DELAY_US(20000);
  prv_display_write_command(0x53);
  prv_display_write_byte(0x2C);
  prv_display_write_command(0x2C);
  for (int i = 0; i < 240*240*12/8; i++){
    prv_display_write_byte(0x00);
  }
  prv_display_write_command(0x29);

  //set frame to 200x200. I know this isn't Pebble-compliant code but hey, this is my repo i do whatever i want
  uint8_t data[4];

  prv_display_write_command(0x2a);
  data[0] = (20 >> 8) & 0xFF;
  data[1] = 20 & 0xFF;
  data[2] = (219 >> 8) & 0xFF;
  data[3] = 219 & 0xFF;
  for (int i=0;i<4;i++){
    prv_display_write_byte(data[i]);
  }

  prv_display_write_command(0x2b);

  for (int i=0;i<4;i++){
    prv_display_write_byte(data[i]);
  }
}

// Clear-all mode is entered by sending 0x04 to the panel
void display_clear(void) {
  prv_enable_display_access();

  //prv_display_write_byte(DISP_MODE_CLEAR);
  //prv_display_write_byte(0x00);

  prv_disable_display_access();
}

//! Static mode is entered by sending 0x00 to the panel
//! This stops any further updates being registered by
//! the display, preventing corruption on shutdown / boot
static void prv_display_enter_static(void) {
  prv_enable_display_access();

  //prv_display_write_byte(DISP_MODE_STATIC);

  prv_disable_display_access();
}

// Helper to reverse command bytes
static uint8_t prv_reverse_bits(uint8_t input) {
  uint8_t result;
  __asm__ ("rev  %[result], %[input]\n\t"
           "rbit %[result], %[result]"
           : [result] "=r" (result)
           : [input] "r" (input));
  return result;
}

static void prv_display_start_write(void) {
  prv_enable_display_access();

  prv_display_write_command(DISP_MODE_WRITE);
}

static void prv_display_write_line(const uint8_t *line) {
  // 1-indexed (ugh) 8bit line address (1-168)
  //prv_display_write_byte(prv_reverse_bits(line_addr + 1));

  for (int i = 0; i < 300; ++i) {
    prv_display_write_byte(line[i]);
  }

  //prv_display_write_byte(0x00);
}

static void prv_display_end_write(void) {


  prv_disable_display_access();
}

// Round a bit offset to a byte offset
static unsigned prv_round_to_byte(unsigned x) {
  return (x + 7) >> 3;
}

// Draw bitmap onto buffer.
static void prv_draw_bitmap(const uint8_t *bitmap, unsigned x_offset, unsigned y_offset,
                            unsigned width, unsigned height,
                            uint8_t buffer[DISP_ROWS][DISP_LINE_BYTES]) {
  // Need to convert offsets to bytes for the horizontal dimensions
  x_offset = prv_round_to_byte(x_offset);
  width = prv_round_to_byte(width);

  for (unsigned i = 0; i < height; i++) {
    memcpy(buffer[i + y_offset] + x_offset, bitmap + i * width, width);
  }
}

static void prv_display_buffer(uint8_t buffer[DISP_ROWS][DISP_LINE_BYTES]) {
  prv_display_start_write();
  uint8_t a;
  for (int i = 0; i < DISP_ROWS; i++) {
    uint8_t * pbuf = line_buffer;
    a = 0;
    for (int byte = 0; byte < 23; byte++){
      for (int bit = 0; bit < (byte == 22 ? 4 : 8); bit++){
        uint8_t r = 0,g = 0,b = 0;
        if ((buffer[i][byte] >> (bit)) & 0x1) {
          r = g = b = 0b11;
        }
        if ((8*byte+bit % 9) == 4){
          a += 2;
          *pbuf++ = (r << 6) | (r << 4) | (g << 2) | g;
          *pbuf = (b << 6) | (b << 4);
          *pbuf++ |= (r << 2) | r;
          *pbuf++ = (g << 6) | (g << 4) | (b << 2) | b;
        } else {
          a++;
          if ((a - 1) % 2 == 0){
            *pbuf++ = (r << 6) | (r << 4) | (g << 2) | g;
            *pbuf = (b << 6) | (b << 4);
          } else {
            *pbuf++ |= (r << 2) | r;
            *pbuf++ = (g << 6) | (g << 4) | (b << 2) | b;
          }
        }
      }
    }
    if (i % 9 == 4){
      prv_display_write_line(line_buffer);
    }
    prv_display_write_line(line_buffer);
  }
  prv_display_end_write();
}

void display_boot_splash(void) {
  uint8_t buffer[DISP_ROWS][DISP_LINE_BYTES];
  // Draw black
  memset(buffer, 0x00, sizeof(buffer));

  prv_draw_bitmap(pebbleos_logo_bits, 34, 69, pebbleos_logo_width, pebbleos_logo_height, buffer);

  prv_display_buffer(buffer);
}

static void prv_set_bit(uint8_t x, uint8_t y, uint8_t buffer[DISP_ROWS][DISP_LINE_BYTES]) {
  buffer[y][prv_round_to_byte(x)] |= (1 << (x % 8));
}

static void prv_render_char(unsigned digit, uint8_t x_offset_bits, uint8_t y_offset,
                            uint8_t buffer[DISP_ROWS][DISP_LINE_BYTES]) {
  const unsigned char_rows = 18, char_cols = 9;
  const uint8_t * char_data = hex_digits_bits[digit];

  // Each character requires 2 bytes of storage
  for (unsigned y = 0; y < char_rows; y++) {
    unsigned cur_y = y_offset + y;
    uint8_t first_byte = char_data[2 * y];

    for (unsigned x = 0; x < char_cols; x++) {
      bool pixel;
      if (x < 8) { // Pixel is in first byte
        pixel = first_byte & (1 << x);
      }
      else { // Last pixel is in second byte
        pixel = char_data[2 * y + 1] & 1;
      }

      // The buffer is already all black, so just set the white pixels
      if (pixel) {
        prv_set_bit(x_offset_bits + x, cur_y, buffer);
      }
    }
  }
}

static void prv_draw_code(uint32_t code, uint8_t buffer[DISP_ROWS][DISP_LINE_BYTES]) {
  const unsigned y_offset = 116; // beneath sad face, above url
  unsigned x_offset = 28; // Aligned with sad face

  // Extract and print digits
  for (int i = 7; i >= 0; i--) {
    // Mask off 4 bits at a time
    uint32_t mask = (0xf << (i * 4));
    unsigned digit = ((code & mask) >> (i * 4));
    prv_render_char(digit, x_offset, y_offset, buffer);

    // Each character is 9px wide plus 2px of padding
    x_offset += 11;
  }
}

void display_error_code(uint32_t code) {
  dbgserial_putstr("drawing error code");
  uint8_t buffer[DISP_ROWS][DISP_LINE_BYTES];
  memset(buffer, 0x00, sizeof(buffer));

  prv_draw_bitmap(dead_face_bits, (140 - dead_face_width) / 2, 24, dead_face_width, dead_face_height, buffer);

  prv_draw_code(code, buffer);

  prv_display_buffer(buffer);
}

//! Do whatever is necessary to prevent visual artifacts when resetting
//! the watch.
void display_prepare_for_reset(void) {
  //prv_display_enter_static();
}

//! Display the progress of a firmware update.
//!
//! The progress is expressed as a rational number less than or equal to 1.
//! When numerator == denominator, the progress indicator shows that the update
//! is complete.
void display_firmware_update_progress(uint32_t numerator, uint32_t denominator) {
  // Dimensions for progress bar
  const unsigned x_offset = 24, y_offset = 106,
                 inner_bar_width = 94, inner_bar_height = 6;

  static unsigned s_prev_num_pixels = -1;
  // Calculate number of pixels to fill in
  unsigned num_pixels = inner_bar_width * numerator / denominator;

  if (num_pixels == s_prev_num_pixels) {
    return;
  }
  s_prev_num_pixels = num_pixels;

  uint8_t buffer[DISP_ROWS][DISP_LINE_BYTES];
  memset(buffer, 0x00, sizeof(buffer));

  prv_draw_bitmap(pebbleos_logo_bits, 16, 69, pebbleos_logo_width, pebbleos_logo_height, buffer);


  prv_draw_bitmap(empty_bar_bits, x_offset, y_offset, empty_bar_width, empty_bar_height, buffer);

  for (unsigned y = 0; y < inner_bar_height; y++) {
    for (unsigned x = 0; x < num_pixels; x++) {
      // Add 1 to offsets so we don't write into outer box
      prv_set_bit(x + x_offset + 1, y_offset + y + 1, buffer);
    }
  }

  prv_display_buffer(buffer);
}

void display_init(void) {
  config.frequency = NRFX_MHZ_TO_HZ(32);
  config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
  dbgserial_putstr("attempting spim init");

  nrfx_err_t err = nrfx_spim_init(&spim, &config, NULL, NULL);
  if (err != NRFX_SUCCESS){
    char error_text[20];
    itoa_hex(err, error_text, 20);
    dbgserial_putstr("spim init failure!");
    dbgserial_putstr(error_text);
  } else {
    dbgserial_putstr("spim init did not fail. what the fuck?");
  }
  bool inited = nrfx_spim_init_check(&spim);
  if (!inited){
    dbgserial_putstr("nrfx_spim_init_check says it aint work. get ready to assert");
  }
  
  // These pins are straight up not used in the ProMicro setup atm.
  // I'm not deleting them outright though because who knows
  /*
  BOARD_DISP_CS_PORT->PIN_CNF[BOARD_DISP_CS_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
			     (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
			     (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
			     (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
			     (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
  BOARD_DISP_DISP_PORT->PIN_CNF[BOARD_DISP_DISP_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
				 (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
				 (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
				 (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
				 (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
  */
  BOARD_DISP_DC_PORT->PIN_CNF[BOARD_DISP_DC_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
           (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
           (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
           (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
           (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
  BOARD_DISP_RST_PORT->PIN_CNF[BOARD_DISP_RST_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
           (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
           (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
           (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
           (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

  prv_display_start();
  dbgserial_putstr("survived display start");
}

void display_deinit(void) {
  nrfx_spim_uninit(&spim);
}
