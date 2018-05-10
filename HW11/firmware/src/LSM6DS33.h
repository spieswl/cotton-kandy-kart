#ifndef LSM6DS33_H__
#define LSM6DS33_H__

// Modified version of i2c_master_noint.h
// Original supplied courtesy of N. Marchuk, modified by W. Spies, 5/7/2018

#define LSM6DS33_addr 0x6B                  // Address for our particular LSM6DS33 (0b01101011)
                                            // Left shifted by 1 in TX/RX functions

void i2c_master_setup(void);                // set up I2C2 as a master, at 400 kHz

void i2c_master_start(void);                // send a START signal
void i2c_master_restart(void);              // send a RESTART signal
void i2c_master_send(unsigned char byte);   // send a byte (either an address or data)
unsigned char i2c_master_recv(void);        // receive a byte of data
void i2c_master_ack(int val);               // send an ACK (0) or NACK (1)
void i2c_master_stop(void);                 // send a stop

////////////////////////////////////////////////////////////////////////////////

void IMU_init();
void IMU_write(unsigned char reg, unsigned char payload);
void IMU_read(unsigned char reg, unsigned char * data);
void IMU_read_mult(unsigned char reg, unsigned char * data, int N); // Special multiple read function to check N registers
signed short combine_sensor_data(unsigned char lower, unsigned char upper);

#endif