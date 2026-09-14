#include "board/board_nrf5.h"
#include "drivers/accel.h"
#include "drivers/i2c.h"
#include "drivers/vibe.h"
#include "kernel/util/sleep.h"
#include "services/common/regular_timer.h"
#include "services/common/vibe_pattern.h"
#include "system/logging.h"
#include "util/math.h"
#include "adxl345_regs.h"
#include "adxl345.h"
#include <stdint.h>

// Forward declaration of private functions defined below public functions
static int32_t prv_adxl345_read(uint8_t reg_addr, uint8_t *buffer, uint16_t read_size);
static int32_t prv_adxl345_write(uint8_t reg_addr, const uint8_t *buffer, uint16_t write_size);
static void prv_adxl345_mdelay(uint32_t ms);
static void prv_adxl345_init(void);
static void prv_adxl345_chase_target_state(void);
static void prv_adxl345_configure_interrupts(void);
static void prv_adxl345_configure_fifo(bool enable);
static void prv_adxl345_configure_double_tap(bool enable);
static void prv_adxl345_configure_shake(bool enable, bool sensitivity_high);
static void prv_adxl345_interrupt_handler(bool *should_context_switch);
static void prv_adxl345_process_interrupts(void);
static void prv_adxl345_interrupt_watchdog_callback(void *data);
static bool prv_adxl345_force_reinit(void);
static bool prv_is_vibing(void);
static int32_t prv_adxl345_set_sampling_interval(uint32_t interval_us);
static void prv_adxl345_read_samples(AccelDriverSample *sample);
static uint8_t prv_adxl345_read_sample(AccelDriverSample *data);
static void prv_note_new_sample(const AccelDriverSample *sample);
static void prv_note_new_sample_mg(int16_t x_mg, int16_t y_mg, int16_t z_mg);
static uint32_t prv_compute_age_ms(uint64_t now_ms, uint64_t then_ms);
typedef enum {
  X_AXIS = 0,
  Y_AXIS = 1,
  Z_AXIS = 2,
} axis_t;
static int16_t prv_get_axis_projection_mg(axis_t axis, int16_t *raw_vector);
static uint64_t prv_get_timestamp_ms(void);

static bool s_adxl345_initialized = false;
static bool s_adxl345_enabled = true;
static bool s_adxl345_running = false;
typedef struct {
	uint32_t sampling_interval_us;
	uint32_t num_samples;
	bool shake_detection_enabled;
	bool shake_sensitivity_high;
	bool double_tap_detection_enabled;
} adxl345_state_t;
adxl345_state_t s_adxl345_state = {0};
adxl345_state_t s_adxl345_state_target = {0};
static uint32_t s_tap_threshold = BOARD_CONFIG_ACCEL.accel_config.double_tap_threshold / 1250;
static bool s_fifo_in_use = false;
static uint32_t s_last_vibe_detected = 0;

static uint8_t s_user_sensitivity_percent = 100;

static uint32_t s_i2c_error_count = 0;
static uint32_t s_last_successful_read_ms = 0;
static uint32_t s_consecutive_errors = 0;
static bool s_sensor_health_ok = true;
static int16_t s_last_sample_mg[3] = {0};
static uint64_t s_last_sample_timestamp_ms = 0;
// Interrupt activity instrumentation so we can spot when the sensor stops firing INT1.
static uint64_t s_last_interrupt_ms = 0;
static uint64_t s_last_wake_event_ms = 0;
static uint64_t s_last_double_tap_ms = 0;
static uint32_t s_interrupt_count = 0;
static uint32_t s_wake_event_count = 0;
static uint32_t s_double_tap_event_count = 0;
static uint32_t s_watchdog_recovery_attempts = 0;

// Interrupt watchdog timer
static RegularTimerInfo s_interrupt_watchdog_timer = {
	.cb = prv_adxl345_interrupt_watchdog_callback,
	.cb_data = NULL,
};

// watch rotation
static bool s_rotated_180 = false;

// Maximum FIFO watermark supported by hardware
#define ADXL345_FIFO_MAX_WATERMARK 32

// Maximum allowed sampling interval for tap detection (in microseconds)
#define ADXL345_TAP_DETECTION_MAX_INTERVAL_US 159375

#define ADXL345_VIBE_COOLDOWN_MS 50

#define ADXL345_MAX_CONSECUTIVE_FAILURES 3
#define ADXL345_IONTERRUPT_GAP_LOG_THRESHOLD_MS 3000
#define ADXL345_INTERRUPT_WATCHDOG_MS 10000
#define ADXL345_INTERRUPT_WATCHDOG_TIMEOUT_MS 5000
#define ADXL345_INTERRUPT_WATCHDOG_MS_NO_SAMPLES 600000

void adxl345_init(void) {
	prv_adxl345_init();
}

void adxl345_power_up(void){
	s_adxl345_enabled = true;
	prv_adxl345_chase_target_state();
}

void adxl345_power_down(void) {
	s_adxl345_enabled = false;
	prv_adxl345_chase_target_state();
}

const AccelDriverInfo ACCEL_DRIVER_INFO = {
	.sample_interval_max = 10000000,
	.sample_interval_low_power = 80000,
	.sample_interval_ui = 80000,
	.sample_interval_game = 20000,
	.sample_interval_min = 312,
};

uint32_t accel_set_sampling_interval(uint32_t interval_us) {
	PBL_LOG(LOG_LEVEL_DEBUG, "ADXL345: Requesting update of sampling interval to %lu us", interval_us);
	s_adxl345_state_target.sampling_interval_us = interval_us;
	prv_adxl345_chase_target_state();
	return s_adxl345_state.sampling_interval_us;
}

uint32_t accel_get_sampling_interval(){
	return s_adxl345_state.sampling_interval_us;
}

void accel_set_num_samples(uint32_t num_samples){
	PBL_LOG(LOG_LEVEL_DEBUG,"ADXL345: Setting number of samples to %lu", num_samples);
	s_adxl345_state_target.num_samples = num_samples;
	prv_adxl345_chase_target_state();
}

int accel_peek(AccelDriverSample *data){
	return prv_adxl345_read_sample(data);
}

void accel_enable_shake_detection(bool on){
	PBL_LOG(LOG_LEVEL_DEBUG, "ADXL345: %s shake detection.", on ? "Enabling" : "Disabling");
	s_adxl345_state_target.shake_detection_enabled = on;
	prv_adxl345_chase_target_state();
}

bool accel_get_shake_detection_enabled(void) {
	return s_adxl345_state.shake_detection_enabled;
}

void accel_enable_double_tap_detection(bool on){
	PBL_LOG(LOG_LEVEL_DEBUG, "ADXL345: %s double tap detection", on ? "Enabling" : "Disabling");
	s_adxl345_state_target.double_tap_detection_enabled = on;
	prv_adxl345_chase_target_state();
}

bool accel_get_double_tap_detection_enabled(void) {
	return s_adxl345_state.double_tap_detection_enabled;
}

void accel_set_shake_sensitivity_high(bool sensitivity_high){
	PBL_LOG(LOG_LEVEL_DEBUG, "ADXL345: Setting shake sensitivity to %s.", sensitivity_high ? "high" : "normal");
	s_adxl345_state_target.shake_sensitivity_high = sensitivity_high;
	prv_adxl345_chase_target_state();
}

void accel_set_shake_sensitivity_percent(uint8_t percent){
	if (percent > 100) {
		percent = 100;
	}
	
	s_user_sensitivity_percent = percent;

	if (s_adxl345_state.shake_detection_enabled) {
		prv_adxl345_configure_shake(true, s_adxl345_state.shake_sensitivity_high);
	}

	PBL_LOG(LOG_LEVEL_DEBUG, "ADXL345: User sensitivity set to %u percent", percent);
}

static int32_t prv_adxl345_read(uint8_t reg_addr, uint8_t *buffer, uint16_t read_size) {
	i2c_use(I2C_ADXL345);
	bool result = i2c_write_block(I2C_ADXL345, 1, &reg_addr);
	if (result) result = i2c_read_block(I2C_ADXL345, read_size, buffer);
	i2c_release(I2C_ADXL345);

  if (!result) {
    s_i2c_error_count++;
    s_consecutive_errors++;
    PBL_LOG(LOG_LEVEL_ERROR, "ADXL345: I2C read failed (reg=0x%02x, count=%lu)", 
            reg_addr, s_consecutive_errors);
    if (s_consecutive_errors >= ADXL345_MAX_CONSECUTIVE_FAILURES) {
      s_sensor_health_ok = false;
      PBL_LOG(LOG_LEVEL_ERROR, "ADXL345: Sensor health degraded after %lu failures", 
              s_consecutive_errors);
    }
    return -1;
  } else {
    s_consecutive_errors = 0;
    s_last_successful_read_ms = prv_get_timestamp_ms();
    s_sensor_health_ok = true;
  }
  
  return 0;
}

static int32_t prv_adxl345_write(uint8_t reg_addr, const uint8_t *buffer,
                                 uint16_t write_size) {
  i2c_use(I2C_ADXL345);
  uint8_t d[write_size + 1];
  d[0] = reg_addr;
  memcpy(&d[1], buffer, write_size);
  bool result = i2c_write_block(I2C_ADXL345, write_size + 1, d);
  i2c_release(I2C_ADXL345);
  
  if (!result) {
    s_i2c_error_count++;
    s_consecutive_errors++;
    PBL_LOG(LOG_LEVEL_ERROR, "ADXL345: I2C write failed (reg=0x%02x)", reg_addr);
    return -1;
  } else {
    s_consecutive_errors = 0;
  }
  
  return 0;
}

static void prv_adxl345_mdelay(uint32_t ms) {
	psleep(ms);
}

// Initialization

static void prv_adxl345_init(void) {
	if (s_adxl345_initialized){
		return;
	}
	s_i2c_error_count = 0;
	s_last_successful_read_ms = 0;
	s_sensor_health_ok = true;

	//verify sensor
	uint8_t whoami;
	if (prv_adxl345_read(ADXL345_DEVID, &whoami, 1)) {
		PBL_LOG(LOG_LEVEL_ERROR,"ADXL345: Failed to read WHO_AM_I (DEVID) register");
		return;
	}
	if (whoami != 0xE5){
		PBL_LOG(LOG_LEVEL_ERROR,"ADXL345: Sensor not detected or malfunctioning (WHO_AM_I=0x%02x, expecting 0x%02x)",whoami,0xE5);
		return;
	}

	PBL_LOG(LOG_LEVEL_DEBUG, "ADXL345: Sensor detected successfully (WHO_AM_I=0x%02x)",whoami);
	
	//configure DATA_FORMAT: +/- 4g sensitivity
	uint8_t data_format;
	prv_adxl345_read(ADXL345_DATA_FORMAT, &data_format, 1);
	data_format &= (uint8_t)(~(0xff));
	data_format |= (1 << 0 | 1 << 3);
	prv_adxl345_write(ADXL345_DATA_FORMAT, &data_format, 1);

	s_adxl345_initialized = true;
	s_consecutive_errors = 0;
	PBL_LOG(LOG_LEVEL_DEBUG, "ADXL345: Initialisation \"complete\"");
}


static uint64_t prv_get_timestamp_ms(void){
	return 0;
}
static void prv_adxl345_chase_target_state(void){
	if (!s_adxl345_initialized){
		PBL_LOG(LOG_LEVEL_DEBUG, "ADXL345: Cannot chase target state before initialisation");
		return;
	}

	bool update_interrupts = false;
	bool should_be_running = s_adxl345_state_target.sampling_interval_us > 0 ||
													 s_adxl345_state_target.num_samples > 0 ||
													 s_adxl345_state_target.shake_detection_enabled ||
													 s_adxl345_state_target.double_tap_detection_enabled;

	if (!should_be_running || !s_adxl345_enabled){
		if (s_adxl345_running){
			PBL_LOG(LOG_LEVEL_DEBUG,"ADXL345: Stopping accelerometer");
			uint8_t power_ctl;
			prv_adxl345_read(ADXL345_POWER_CTL, &power_ctl, 1);
			power_ctl &= ~(1<<3);
			prv_adxl345_write(ADXL345_POWER_CTL, &power_ctl, 1);
			// TODO: configure interrupts
		}
		return;
	} else if (!s_adxl345_running) {
		s_adxl345_running = true;
		update_interrupts = true;
			uint8_t power_ctl;
			prv_adxl345_read(ADXL345_POWER_CTL, &power_ctl, 1);
			power_ctl |= (1<<3);
			prv_adxl345_write(ADXL345_POWER_CTL, &power_ctl, 1);
		// TODO: interrupt stuff
	}

	// Update number of samples
	if (s_adxl345_state_target.num_samples != s_adxl345_state.num_samples){
		s_adxl345_state.num_samples = s_adxl345_state_target.num_samples;
		update_interrupts = true;
	}
	// Update shake detection
	if (s_adxl345_state_target.shake_detection_enabled != s_adxl345_state.shake_detection_enabled ||
			s_adxl345_state_target.shake_sensitivity_high != s_adxl345_state.shake_sensitivity_high){
		s_adxl345_state_target.shake_detection_enabled = s_adxl345_state.shake_detection_enabled;
		s_adxl345_state_target.shake_sensitivity_high = s_adxl345_state.shake_sensitivity_high;
		prv_adxl345_configure_shake(s_adxl345_state.shake_detection_enabled, s_adxl345_state.shake_sensitivity_high);	
		update_interrupts = true;
	}
	// Update double tap detection
	if (s_adxl345_state_target.double_tap_detection_enabled != 
			s_adxl345_state.double_tap_detection_enabled){
		prv_adxl345_configure_double_tap(s_adxl345_state_target.double_tap_detection_enabled);
		s_adxl345_state_target.double_tap_detection_enabled = s_adxl345_state.double_tap_detection_enabled;
		update_interrupts = true;
	}
	// Updste smapling interval
	
}
static void prv_adxl345_configure_interrupts(void){

}
static void prv_adxl345_configure_fifo(bool enable){

}
static void prv_adxl345_configure_double_tap(bool enable){

}
static void prv_adxl345_configure_shake(bool enable, bool sensitivity_high){
	uint8_t act_inact_enable;
	prv_adxl345_read(ADXL345_ACT_INACT_CTL, &act_inact_enable, 1);

	uint8_t int_enable;
	prv_adxl345_read(ADXL345_INT_ENABLE, &int_enable, 1);
	if (!enable){
		int_enable &= ~(1 << 4);
		prv_adxl345_write(ADXL345_INT_ENABLE, &int_enable, 1);
		act_inact_enable &= ~(1 << 4 | 1 << 5 | 1 << 6);
		prv_adxl345_write(ADXL345_ACT_INACT_CTL, &act_inact_enable, 1);
		return;
	}
	int_enable |= (1 << 4);
	prv_adxl345_write(ADXL345_INT_ENABLE, &int_enable, 1);
	act_inact_enable |= (1 << 4 | 1 << 5 | 1 << 6);
	prv_adxl345_write(ADXL345_ACT_INACT_CTL, &act_inact_enable, 1);

	uint8_t sensitivity = 1;
	if (sensitivity_high){
		sensitivity = BOARD_CONFIG_ACCEL.accel_config.shake_thresholds[AccelThresholdHigh];
	} else {
		sensitivity = BOARD_CONFIG_ACCEL.accel_config.shake_thresholds[AccelThresholdLow];
	}
	prv_adxl345_write(ADXL345_THRESH_ACT, &sensitivity, 1);

}
static void prv_adxl345_interrupt_handler(bool *should_context_switch){

}
static void prv_adxl345_process_interrupts(void){

}
static void prv_adxl345_interrupt_watchdog_callback(void *data){

}
static bool prv_adxl345_force_reinit(void){
	return false;
}
static uint8_t prv_adxl345_read_sample(AccelDriverSample *sample){
	if (!s_adxl345_initialized){
		PBL_LOG(LOG_LEVEL_ERROR, "ADXL345: Not initialised, cannot read sample");
		return -1;
	}

	uint8_t raw[6];
	int16_t accel_raw[3];
		prv_adxl345_read(ADXL345_DATAX0, raw, 6);
	accel_raw[0] = ((raw[1] << 8) | raw[0]);
	accel_raw[1] = ((raw[3] << 8) | raw[2]);
	accel_raw[2] = ((raw[5] << 8) | raw[4]);

	// for an adxl345 chip positioned with the Pin 1 dot located on the top left
	sample->y = accel_raw[0]*4;
	sample->x = accel_raw[1]*4;
	sample->z = accel_raw[2]*4;
	sample->timestamp_us = prv_get_timestamp_ms() * 1000;

	//prv_note_new_sample(sample);

	if (s_adxl345_state.num_samples > 0){
		//accel_cb_new_sample(sample);
	}
	PBL_LOG(LOG_LEVEL_DEBUG,"ADXL345: Raw data: %hd %hd %hd",accel_raw[0],accel_raw[1],accel_raw[2]);
	return 0;
}
