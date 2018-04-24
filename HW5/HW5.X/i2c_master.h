#ifndef I2C_MASTER_H__
#define I2C_MASTER_H__

// Modified version of i2c_master_noint.h
// Original supplied courtesy of N. Marchuk, modified by W. Spies, 4/23/2018

#define MCP23008_addr 0x27                  // Address for our particular MCP23008s (0b00100111)
                                            // Left shifted by 1 in TX/RX functions

void i2c_master_setup(void);                // set up I2C 2 as a master, at 400 kHz

void i2c_master_start(void);                // send a START signal
void i2c_master_restart(void);              // send a RESTART signal
void i2c_master_send(unsigned char byte);   // send a byte (either an address or data)
unsigned char i2c_master_recv(void);        // receive a byte of data
void i2c_master_ack(int val);               // send an ACK (0) or NACK (1)
void i2c_master_stop(void);                 // send a stop

#endif