/*
function configuration of the master i2c for the esp32
*/
extern static esp_err_t i2c_master_init(void)

/*
function to receive data from a start address with i2c
*/
extern void send_receive_data(i2c_port_t i2c_num, int adress, uint8_t * value_lsb, uint8_t * value_msb)

/*
function to write at some address 16 bit
*/
extern void write_data(i2c_port_t i2c_num, int adress, uint8_t * value_lsb, uint8_t * value_msb)

extern void write_ZPOS(double angle_raw)

extern void write_MPOS(double angle_raw)

extern void write_MANG(double angle_raw)

extern void write_config(uint8_t power_mode, uint8_t isteresis, uint8_t output, uint8_t pwmF, uint8_t slow_filter, uint8_t fast_filter_threshold, uint8_t whatdog)

extern double get_raw_angle()

extern double get_angle()

extern void get_magnete_status()

extern int get_AGC()

extern int get_magnitude()

extern void get_config()

extern double get_ZPOS()

extern double get_MPOS()

extern double get_MANG()

extern int get_ZMCO()