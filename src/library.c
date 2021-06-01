#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define hall_sensor 0x36 //address hall sensors
/********************
 * config registri
 * ******************/
#define ZMCO_ADD 0x00 // R 1:0
#define ZPOS_ADD 0x01 // R/W/P 11:8
#define ZPOS_LONG_ADD 0x02 // R/W/P 7:0
#define MPOS_ADD 0x03 // R/W/P 11:8
#define MPOS_LONG_ADD 0x04 // R/W/P 7:0
#define MANG_ADD 0x05 // R/W/P 11:8
#define MANG_LONG_ADD 0x06 // R/W/P 7:0 
#define CONF1_ADD 0x07 // R/W/P WD(5) FTH(4:2) SF(1:0)
#define CONF2_ADD 0x08 // R/W/P PWMF(7:6) OUTS(5:4) HYST(3:2) PM(1:0)


/********************
 * output registri
 * ******************/

#define RAW_ANGLE_ADD 0x0C // R 11:8
#define RAW_ANGLE_LONG_ADD 0x0D // R 7:0
#define ANGLE_ADD 0x0E // R 11:8
#define ANGLE_LONG_ADD 0x0F // R 7:0
/********************
 * status registri
 * ******************/


/********************
 * burn registri
 * ******************/

#define I2C_MASTER_SDA_IO 21 //gpio pin for sda
#define I2C_MASTER_SCL_IO 22 //gpio pin for scl
#define I2C_MASTER_FREQ_HZ 1000 //herz 

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


/*
test
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


void send_receive_data(i2c_port_t i2c_num, int adress, uint8_t * value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd); //start comunicazione
    
    i2c_master_write_byte(cmd, (hall_sensor << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, adress, ACK_CHECK_EN);

    i2c_master_start(cmd);  //restart, and 

    //read register
    i2c_master_write_byte(cmd, (hall_sensor << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, value, NACK_VAL);
    
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
}

int get_raw_angle()
{
    uint8_t  angle;
    send_receive_data(0, RAW_ANGLE_LONG_ADD, &angle);
    return (int) angle;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    while(1)
    {
        printf("angolo: %d\n",get_raw_angle());
    }
}