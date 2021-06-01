/*
Test code for i2c communication with hull encoder and mpu

*/

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define hall_sensor 0x36 //address hall sensors

#define ADDRESS_ANGLE (int) 0x0D

#define I2C_MASTER_SDA_IO 21 //gpio pin for sda
#define I2C_MASTER_SCL_IO 22 //gpio pin for scl
#define I2C_MASTER_FREQ_HZ 1000 //herz 

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


SemaphoreHandle_t print_mux = NULL;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,//set up master for asking data
        .sda_io_num = I2C_MASTER_SDA_IO,         
        .sda_pullup_en = GPIO_PULLUP_ENABLE, //usato per evitare l'alimentazione con un sistema esterno dato che son con il drain diretto a terra
        .scl_io_num = I2C_MASTER_SCL_IO,         
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  // select frequency specific to your project
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t  i2c_master_read_sensor_angle(i2c_port_t i2c_num, uint8_t *data_rd)
{
    int ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd); //start comunicazione
    
    i2c_master_write_byte(cmd, (hall_sensor << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADDRESS_ANGLE, ACK_CHECK_EN);
    //i2c_master_write(cmd, ADDRESS_ANGLE, size, ACK_CHECK_EN);

    i2c_master_start(cmd);  //restart, and 

    //read 0x0C register
    i2c_master_write_byte(cmd, (hall_sensor << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_rd, NACK_VAL);
    
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    return ret;
}


static void i2c_test_task(void *arg)
{
    uint32_t task_idx = (uint32_t)arg;
    uint8_t sensor_data_h;
    char * TAG = "test";

    int ret;
    int cnt=0;
    while(1)
    {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = i2c_master_read_sensor_angle(0, &sensor_data_h);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ SENSOR\n", task_idx);
            printf("*******************\n");
            printf("data: %d\n", sensor_data_h);
            
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay((1000 * (task_idx + 1)) / portTICK_RATE_MS);

    }
}

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex(); // permette di accedere ad un sistema a memoria limitata in un ordine definito dall'utente

    ESP_ERROR_CHECK(i2c_master_init());
    
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL); //crea il task 
    //xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);

/*
xTaskCreate:
    pvTaskCode: pointer to task entry function should never return
    pcName: name task
    usStackDepth: n words! to allocate -> salva tot byte per il task usStackDepth*(16||32)/8 
    pvParameters: paramater to the created task
    uxPriority: priorità
    pxCreatedTask: Used to pass a handle to the created task out of the xTaskCreate() function
*/
}
