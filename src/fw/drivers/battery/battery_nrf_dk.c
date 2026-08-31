#include "drivers/battery.h"
#include "kernel/events.h"
#include "nrf52840.h"
#include "services/common/new_timer/new_timer.h"
#include "services/common/system_task.h"
#include "system/passert.h"
#include <nrf_power.h>
#include <nrfx_power.h>

static const uint32_t USB_CONN_DEBOUNCE_MS = 400;
static TimerID s_debounced_timer_handle = TIMER_INVALID_ID;
static bool s_debounced_is_usb_connected = false;

static void battery_conn_debounce_callback(void * data) {
  s_debounced_is_usb_connected = !s_debounced_is_usb_connected;
  PebbleEvent event = {
    .type = PEBBLE_BATTERY_CONNECTION_EVENT,
    .battery_connection = {
      .is_connected = s_debounced_is_usb_connected,
    }
  };
  event_put(&event);
};

static void prv_start_timer_sys_task_callback(void * data){
  new_timer_start(s_debounced_timer_handle, USB_CONN_DEBOUNCE_MS, battery_conn_debounce_callback, NULL, 0);
}

static void battery_vusb_interrupt_handler(nrfx_power_usb_evt_t event){
  bool should_context_switch = false;
  system_task_add_callback_from_isr(prv_start_timer_sys_task_callback, NULL, &should_context_switch);
} 

void battery_init(void) {
  s_debounced_timer_handle = new_timer_create();
  nrfx_power_config_t power_config = {
    .dcdcen = false,
    .dcdcenhv = false,
  };
  nrfx_err_t err = nrfx_power_init(&power_config);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  nrfx_power_usbevt_config_t config = {
    .handler = battery_vusb_interrupt_handler,  
  };
  nrfx_power_usbevt_init(&config);
  nrfx_power_usbevt_enable();
}

bool battery_is_present(void) {
  return true;
}

int battery_get_millivolts(void) {
#if defined(BOARD_PROMICRO)
  ADCVoltageMonitorReading info = battery_read_voltage_monitor();
  return battery_convert_reading_to_millivolts(info, 0, 0);
#else
  battery_vusb_interrupt_handler(0);
  return 4200;
#endif
}

int battery_get_constants(BatteryConstants *constants) {
  constants->v_mv = 4000;
  constants->i_ua = 100;
  constants->t_mc = 25000;
  return 0;
}

int battery_charge_status_get(BatteryChargeStatus *status) {
  *status = BatteryChargeStatusUnknown;
  return 0;
}

bool battery_charge_controller_thinks_we_are_charging_impl(void) {
  return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
}

bool battery_is_usb_connected_impl(void) {
  return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
}

void battery_set_charge_enable(bool charging_enabled) {
}

void battery_set_fast_charge(bool fast_charge_enabled) {
}
