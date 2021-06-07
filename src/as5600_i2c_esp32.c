#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include"as5600_i2c_esp32.h"


#define hall_sensor 0x36 //address hall sensors

/********************
 * config registri
 * ******************/
#define ZMCO_ADD 0x00 // R 1:0  f ZPOS and MPOS have never been permanently written 
#define ZPOS_ADD 0x01 // R/W/P 11:8   start position
//#define ZPOS_LONG_ADD 0x02 // R/W/P 7:0
#define MPOS_ADD 0x03 // R/W/P 11:8   stop position  
//#define MPOS_LONG_ADD 0x04 // R/W/P 7:0
#define MANG_ADD 0x05 // R/W/P 11:8
//#define MANG_LONG_ADD 0x06 // R/W/P 7:0 
#define CONF_ADD 0x07 // R/W/P WD(5) FTH(4:2) SF(1:0)
        //watchdogs - fast filter threshold - slow filter 
//#define CONF2_ADD 0x08 // R/W/P PWMF(7:6) OUTS(5:4) HYST(3:2) PM(1:0)
        //PWM frequency - output stage - hysteresis - power mode

/********************
 * output registri
 * ******************/

#define RAW_ANGLE_ADD 0x0C // R 11:8
//#define RAW_ANGLE_LONG_ADD 0x0D // R 7:0
#define ANGLE_ADD 0x0E // R 11:8
//#define ANGLE_LONG_ADD 0x0F // R 7:0

/********************
 * status registri
 * ******************/

#define STATUS_ADD 0x0B // MD(5) ML(4) MH(3)
        /*
        MH AGC minimum gain overflow, magnet too strong
        ML AGC maximum gain overflow, magnet too weak
        MD Magnet detected
        */

#define AGC_ADD 0x1A // R 7:0
        //s Automatic Gain Control 
#define MAGNITUDE_ADD 0x1B // R 11:8
        // magnitude value of the internal CORDIC
//#define MAGNITUDE_LONG_ADD 0x1C // R 7:0


/********************
 * burn registri
 * ******************/

#define BURN_ADD 0xFF // W : Burn_Angle = 0x80; Burn_Setting = 0x40

/*******************
 * config parameters
 * ********************/

//PM(1:0) Power Mode 
//Polling time current 
#define NOM 0b00
#define LPM1 0b01
#define LPM2 0b10
#define LPM3 0b11

//HYST(1:0) 3:2 Hysteresis 

#define OFF 0b00
#define LSB 0b01
#define LSBs1 0b10
#define LSBs2 0b11

//OUTS(1:0) 5:4 Output Stage

#define ANAL_FULL 0b00 //analogic full range from 0% to 100% between GND and VDD
#define ANAL_RED 0b01 //analogic reduced range from 10% to 90% between GND and VDD
#define PWM 0b10 //digital PWM

//PWMF(1:0) 7:6 PWM Frequency 

#define PWM_115 0b00
#define PWM_230 0b01
#define PWM_460 0b10
#define PWM_920 0b11

//SF (1:0) 9:8 Slow Filter 

#define SL_16x 0b00
#define SL_8x 0b01
#define SL_4x 0b10
#define SL_2x 0b11

//FTH (2:0) 12:10 Fast Filter Threshold 

#define FTH_SLOW_ONLY 0b000
#define FTH_6LSB 0b001
#define FTH_7LSB 0b010
#define FTH_9LSB 0b011
#define FTH_18LSB 0b100
#define FTH_21LSB 0b101
#define FTH_24LSB 0b110
#define FTH_10LSB 0b111

//WD (13) Watchdog 

#define WDOG_OFF 0b0
#define WDOG_ON 0b1


/**************************************************/
#define I2C_MASTER_SDA_IO 21 //gpio pin for sda
#define I2C_MASTER_SCL_IO 22 //gpio pin for scl
#define I2C_MASTER_FREQ_HZ 1000 //herz 

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */



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


void send_receive_data(i2c_port_t i2c_num, int adress, uint8_t * value_lsb, uint8_t * value_msb)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd); //start comunicazione
    
    i2c_master_write_byte(cmd, (hall_sensor << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, adress, ACK_CHECK_EN);
//***********
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
//***********
    i2c_master_start(cmd);  //restart, and 

    //read register
    i2c_master_write_byte(cmd, (hall_sensor << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    
    //uint8_t value_lsb;
    if (adress==ZPOS_ADD||adress==MPOS_ADD||adress==MANG_ADD||adress==RAW_ANGLE_ADD||adress==ANGLE_ADD||adress==MAGNITUDE_ADD||adress==CONF_ADD)
    {
        i2c_master_read_byte(cmd,value_lsb,ACK_VAL);
        //printf("\nlsb: %d", (int) value_lsb);
    }

    //uint8_t value_msb;
    i2c_master_read_byte(cmd, value_msb, NACK_VAL);
    //printf("\nmsb: %d", (int) value_msb);

    
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
}

void write_data(i2c_port_t i2c_num, int adress, uint8_t * value_lsb, uint8_t * value_msb)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd); //start comunicazione
    
    //write address 
    i2c_master_write_byte(cmd, (hall_sensor << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    //write address register
    i2c_master_write_byte(cmd, adress, ACK_CHECK_EN);    

    //write uint8_t value_lsb;
    i2c_master_write_byte(cmd,value_lsb,ACK_VAL);

    //write uint8_t value_msb;
    i2c_master_write_byte(cmd, value_msb, NACK_VAL);

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
}

uint8_t value_lsb,value_msb,zeros;

uint16_t angle, magnitude;

void write_ZPOS(double angle_raw)
{
    angle = (uint16_t) angle_raw * 4096 / 360;
    value_msb =(uint8_t) angle & 0b0000000011111111;  
    value_lsb = (uint8_t) angle>>8 & 0b0000000000001111;
    
    write_data( 0 ,ZPOS_ADD, &value_lsb, &value_msb);
}

void write_MPOS(double angle_raw)
{
    angle = (uint16_t) angle_raw * 4096 / 360;
    value_msb =(uint8_t) angle & 0b0000000011111111;  
    value_lsb = (uint8_t) angle>>8 & 0b0000000000001111;
    
    write_data( 0 ,MPOS_ADD, &value_lsb, &value_msb);
}

void write_MANG(double angle_raw)
{
    angle = (uint16_t) angle_raw * 4096 / 360;
    value_msb =(uint8_t) angle & 0b0000000011111111;  
    value_lsb = (uint8_t) angle>>8 & 0b0000000000001111;
    
    write_data( 0 ,MANG_ADD, &value_lsb, &value_msb);
}

void write_config(uint8_t power_mode, uint8_t isteresis, uint8_t output, uint8_t pwmF, uint8_t slow_filter, uint8_t fast_filter_threshold, uint8_t whatdog)
{
    value_lsb=(uint8_t) pwmF<<6 | output<<4 | isteresis<<2 | power_mode;
    value_msb=(uint8_t) whatdog<<5 | slow_filter<<2 | fast_filter_threshold;
    //to test per evitare che faccia qualche porcheria i puntatori
    //write_data( 0 ,ZPOS_ADD, &value_lsb, &value_msb);
}

double get_raw_angle()
{    
    send_receive_data(0, RAW_ANGLE_ADD, &value_lsb, &value_msb);
    angle=(((uint16_t)value_lsb<<8)|value_msb);
    return ((int) angle & 0b0000111111111111)*360.0/4096.0;
}

double get_angle()
{
    send_receive_data(0, ANGLE_ADD, &value_lsb, &value_msb);
    angle=(((uint16_t)value_lsb<<8)|value_msb);
    return ((int) (angle & 0b0000111111111111))*360.0/4096.0;
}

void get_magnete_status()
{
    //MD(5) ML(4) MH(3)
        /*
        MH AGC minimum gain overflow, magnet too strong
        ML AGC maximum gain overflow, magnet too weak
        MD Magnet detected
        */

    send_receive_data(0, STATUS_ADD,&zeros, &value_msb);
    ((value_msb & 0b00001000) == 0b00001000)?printf("\nstrong"):printf("\ngood");
    ((value_msb & 0b00010000) == 0b00010000)?printf("\nweak"):printf("\ngood");
    ((value_msb & 0b00100000) == 0b00100000)?printf("\nc'è"):printf("\nmanca");
}

int get_AGC()
{
    send_receive_data(0, AGC_ADD,&zeros, &value_msb);
    return (int) value_msb;
}

int get_magnitude()
{
    send_receive_data(0, MAGNITUDE_ADD,&value_lsb, &value_msb);
    magnitude=(((uint16_t)value_lsb<<8)|value_msb);
    return (int) (magnitude & 0b0000111111111111);
    
}

void get_config()
{
    send_receive_data(0, CONF_ADD,&value_lsb, &value_msb);
    
    //PM(1:0) Power Mode 
    switch (value_msb & 0b00000011)
    {
        case NOM: printf("\nNOM"); break;
        case LPM1: printf("\nLPM1"); break;
        case LPM2: printf("\nLPM2"); break;
        case LPM3: printf("\nLPM3"); break;
    }
    //HYST(1:0) 3:2 Hysteresis 
    switch (value_msb>>2 & 0b00000011)
    {
        case OFF: printf("\nOFF"); break;
        case LSB: printf("\nLSB"); break;
        case LSBs1: printf("\nLSBs1"); break;
        case LSBs2: printf("\nLSBs2"); break;
    }

    //OUTS(1:0) 5:4 Output Stage
    switch (value_msb>>4 & 0b00000011)
    {
        case ANAL_FULL: printf("\nANAL_FULL"); break;
        case ANAL_RED: printf("\nANAL_RED"); break;
        case PWM: printf("\nPWM"); break;
    }

    //PWMF(1:0) 7:6 PWM Frequency 
    switch (value_msb>>6 & 0b00000011)
    {
        case PWM_115: printf("\nPWM_115"); break;
        case PWM_230: printf("\nPWM_230"); break;
        case PWM_460: printf("\nPWM_460"); break;
        case PWM_920: printf("\nPWM_920"); break;
    }

    //SF (1:0) 9:8 Slow Filter 
    switch (value_lsb & 0b00000011)
    {
        case SL_16x: printf("\nSL_16x"); break;
        case SL_8x: printf("\nSL_8x"); break;
        case SL_4x: printf("\nSL_4x"); break;
        case SL_2x: printf("\nSL_2x"); break;
    }

    //FTH (2:0) 12:10 Fast Filter Threshold 
    switch (value_lsb>>2 & 0b000000111)
    {
        case FTH_SLOW_ONLY: printf("\nFTH_SLOW_ONLY"); break;
        case FTH_6LSB: printf("\nFTH_6LSB"); break;
        case FTH_7LSB: printf("\nFTH_7LSB"); break;
        case FTH_9LSB: printf("\nFTH_9LSB"); break;
        case FTH_18LSB: printf("\nFTH_18LSB"); break;
        case FTH_21LSB: printf("\nFTH_21LSB"); break;
        case FTH_24LSB: printf("\nFTH_24LSB"); break;
        case FTH_10LSB: printf("\nFTH_10LSB"); break;
    }

    //WD (13) Watchdog

    switch (value_lsb>>5 & 0b00000001)
    {
        case WDOG_OFF: printf("\nWDOG_OFF"); break;
        case WDOG_ON: printf("\nWDOG_ON"); break;
    }

}

double get_ZPOS() //start position
{
    send_receive_data(0, ZPOS_ADD, &value_lsb, &value_msb);
    angle=(((uint16_t)value_lsb<<8)|value_msb);
    return ((int) (angle & 0b0000111111111111))*360.0/4096.0;
}

double get_MPOS() //stop position
{
    send_receive_data(0, MPOS_ADD, &value_lsb, &value_msb);
    angle=(((uint16_t)value_lsb<<8)|value_msb);
    return ((int) (angle & 0b0000111111111111))*360.0/4096.0;
}

double get_MANG() //TOCHECK
{
    send_receive_data(0, MANG_ADD, &value_lsb, &value_msb);
    angle=(((uint16_t)value_lsb<<8)|value_msb);
    return ((int) (angle & 0b0000111111111111));
}

int get_ZMCO()
{
    send_receive_data(0, MANG_ADD, &value_lsb, &value_msb);
    printf("\n\nZMCO: %d \n",((int) value_msb & 0b00000011));
    return 0;

}


//to add at the code
//ESP_ERROR_CHECK(i2c_master_init());
