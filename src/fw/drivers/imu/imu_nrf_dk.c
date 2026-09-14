#include "board/board.h"
#include "board/board_nrf5.h"
#include "drivers/accel.h"
#include "drivers/imu.h"
#include "drivers/imu/adxl345/adxl345.h"
#include "kernel/util/delay.h"
#include "system/logging.h"

void imu_init(void) {
  adxl345_init();
}

void imu_power_up(void) {
}

void imu_power_down(void) {
}
