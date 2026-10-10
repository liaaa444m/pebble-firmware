#include "board/board.h"
#include "drivers/button.h"
//#include "drivers/rtc.h"
#include "drivers/dbgserial.h"
#include "nrf_gpio.h"
#include "system/reset.h"
#include <nrf_power.h>
#include <system/bootbits.h>

static void prv_wait_until_buttons_are_released(void) {
  for (int bounce_count = 0; bounce_count < 10; ++bounce_count) {
    uint8_t button_state;

    // First, see if the buttons are all released for a period of time.
    for (int i = 0; i < 10000; ++i) {
      button_state = button_get_state_bits();
      if (button_state != 0) {
        // Someone pushed a button!
        break;
      }
    }

    if (button_state == 0) {
      // We made it through with all the buttons released. We're good.
      return;
    }

    // Alright, so either the button is held down or we hit a bounce. Wait
    // for all the buttons to release again.
    // 100000 is about a second in practice
    for (int i = 0; i < 100000; ++i) {
      if (button_get_state_bits() == 0) {
        // All the buttons are released!
        break;
      }
    }
  }
}

void enter_standby_mode(void){
	prv_wait_until_buttons_are_released();

	nrf_gpio_cfg_sense_set(11, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_set(32+4, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_set(32+6, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_set(31, NRF_GPIO_PIN_SENSE_LOW);
	boot_bit_clear(BOOT_BIT_STANDBY_MODE_REQUESTED);
	boot_bit_set(BOOT_BIT_STANDBY_MODE_ENTERED);
	dbgserial_putstr("Entering standby");
	nrf_power_system_off(NRF_POWER);	
	__WFI();
}

bool should_leave_standby_mode(void){
  // Make sure a button is held down before waking up
  for (int i = 0; i < 10000000; ++i) {
		dbgserial_putstr("testing button");
    if (button_get_state_bits() == 0) {
      // stop waiting if not held down any longer and go back to sleep
      return false;
    }
  }

  return true;
}
