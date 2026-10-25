// What the firmware uses from ESP-IDF and FreeRTOS, for translate-c. The
// build turns this into the `idf` module.
#include <stdio.h>
#include <stdlib.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "idf_config.h"
