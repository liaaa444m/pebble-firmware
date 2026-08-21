#include "drivers/gpio.h"
#include "drivers/nrfx_errors.h"
#include "drivers/periph_config.h"
#include "drivers/voltage_monitor.h"
#include "kernel/util/delay.h"
#include "nrf_saadc.h"
#include "nrfx_config.h"
#include "os/mutex.h"
#include "system/passert.h"

#include <nrfx_saadc.h>

static PebbleMutex *s_adc_mutex;

void voltage_monitor_init(void) {
	s_adc_mutex = mutex_create();
}

void voltage_monitor_device_init(const VoltageMonitorDevice *device){
	nrfx_err_t err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
	PBL_ASSERTN(err == NRFX_SUCCESS);
	nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(device->input,0);
	channel.channel_config.acq_time = NRF_SAADC_ACQTIME_40US;
	channel.channel_config.gain = NRF_SAADC_GAIN1_6;
	channel.channel_config.burst = NRF_SAADC_BURST_ENABLED;
	nrfx_saadc_channels_config(&channel, 1);
	nrfx_saadc_simple_mode_set(1 << 0, NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_16X, NULL);
}

void voltage_monitor_read(const VoltageMonitorDevice *device, VoltageReading *reading_out){
	mutex_lock(s_adc_mutex);
	int16_t value = 0;
	nrfx_saadc_buffer_set(&value, 1);
	nrfx_saadc_mode_trigger();
	reading_out->vref_total = value;
	mutex_unlock(s_adc_mutex);
}
