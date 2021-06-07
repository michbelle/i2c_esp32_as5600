
#ifndef as5600_i2c_esp32_h
#define as5600_i2c_esp32_h



/*
function configuration of the master i2c for the esp32
*/
//static 
esp_err_t i2c_master_init(void);

/*
function to receive data from a start address with i2c
*/
void send_receive_data(i2c_port_t i2c_num, int adress, uint8_t * value_lsb, uint8_t * value_msb);

/*
function to write at some address 16 bit
*/
void write_data(i2c_port_t i2c_num, int adress, uint8_t * value_lsb, uint8_t * value_msb);

void write_ZPOS(double angle_raw);

void write_MPOS(double angle_raw);

void write_MANG(double angle_raw);

void write_config(uint8_t power_mode, uint8_t isteresis, uint8_t output, uint8_t pwmF, uint8_t slow_filter, uint8_t fast_filter_threshold, uint8_t whatdog);

double get_raw_angle();

double get_angle();

void get_magnete_status();

int get_AGC();

int get_magnitude();

void get_config();

double get_ZPOS();

double get_MPOS();

double get_MANG();

int get_ZMCO();


#endif