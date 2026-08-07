#pragma once

#define BOARD_LSE_MODE RCC_LSE_Bypass // TODO: Figure out what this means

static const BoardConfig BOARD_CONFIG = {
  .ambient_light_dark_threshold = 0,
  .ambient_k_delta_threshold = 0,
  .photo_en = {GPIOC, GPIO_Pin_0, true}, // this appears to be for the ambient sensor
  .als_always_on = false, // als = ambient light sensor. lmao.
  .dbgserial_int = { EXTI_PortSourceGPIOB, 5 }, //i dont actually know if we have an interrupt for this
  
  .lcd_com = { 0 },

  .backlight_on_percent = 25,
  .backlight_max_duty_cycle_percent = 67,

  // display 5v output?
  .power_5v0_options = OptionNotPresent,
  .power_ctl_5v0 = { 0 },

  .has_mic = false,
};

static const BoardConfigButton BOARD_CONFIG_BUTTON = {
  .buttons = {
    [BUTTON_ID_BACK]    = { "Back",   GPIOC, GPIO_Pin_3, { EXTI_PortSourceGPIOC, 3 }, GPIO_PuPd_NOPULL },
    [BUTTON_ID_UP]      = { "Up",     GPIOA, GPIO_Pin_2, { EXTI_PortSourceGPIOA, 2 }, GPIO_PuPd_NOPULL },
    [BUTTON_ID_SELECT]  = { "Select", GPIOC, GPIO_Pin_6, { EXTI_PortSourceGPIOC, 6 }, GPIO_PuPd_NOPULL },
    [BUTTON_ID_DOWN]    = { "Down",   GPIOA, GPIO_Pin_1, { EXTI_PortSourceGPIOA, 1 }, GPIO_PuPd_NOPULL },
  },

  .button_com = { 0 }, // button COMMON pin. WHAT DOES THAT MEAN
  .active_high = false,
};

extern DMARequest * const GC9A01_SPI_TX_DMA;

extern UARTDevice * const DBG_UART;
