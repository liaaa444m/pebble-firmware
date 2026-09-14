#pragma once

#include <stdint.h>

static const uint8_t ADXL345_DEVID = 0x00;
// 0x01 - 0x1C: reserved
static const uint8_t ADXL345_THRESH_TAP = 0x1D;
static const uint8_t ADXL345_OFSX = 0x1E;
static const uint8_t ADXL345_OFSY = 0x1F;
static const uint8_t ADXL345_OFSZ = 0x20;
static const uint8_t ADXL345_DUR = 0x21;
static const uint8_t ADXL345_LATENT = 0x22;
static const uint8_t ADXL345_WINDOW = 0x23;
static const uint8_t ADXL345_THRESH_ACT = 0x24;
static const uint8_t ADXL345_THRESH_INACT = 0x25;
static const uint8_t ADXL345_TIME_INACT = 0x26;
static const uint8_t ADXL345_ACT_INACT_CTL = 0x27;
static const uint8_t ADXL345_THRESH_FF = 0x28;
static const uint8_t ADXL345_TIME_FF = 0x29;
static const uint8_t ADXL345_TAP_AXES = 0x2A;
static const uint8_t ADXL345_ACT_TAP_STATUS = 0x2B;
static const uint8_t ADXL345_BW_RATE = 0x2C;
static const uint8_t ADXL345_POWER_CTL = 0x2D;
static const uint8_t ADXL345_INT_ENABLE = 0x2E;
static const uint8_t ADXL345_INT_MAP = 0x2F;
static const uint8_t ADXL345_INT_SOURCE = 0x30;
static const uint8_t ADXL345_DATA_FORMAT = 0x31;
static const uint8_t ADXL345_DATAX0 = 0x32;
static const uint8_t ADXL345_DATAX1 = 0x33;
static const uint8_t ADXL345_DATAY0 = 0x34;
static const uint8_t ADXL345_DATAY1 = 0x35;
static const uint8_t ADXL345_DATAZ0 = 0x36;
static const uint8_t ADXL345_DATAZ1 = 0x37;
static const uint8_t ADXL345_FIFO_CTL = 0x38;
static const uint8_t ADXL345_FIFO_STATUS = 0x039;
