#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include"as5600_i2c_esp32.h"

int app_main()
{
    ESP_ERROR_CHECK(i2c_master_init());
    get_raw_angle();
    return 1;
}