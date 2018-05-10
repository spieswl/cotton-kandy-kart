#include <xc.h>
#include "LSM6DS33.h"

// Modified version of i2c_master_noint.c
// Original supplied courtesy of N. Marchuk, modified by W. Spies, 5/7/2018

// I2C Master utilities, 400 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol

void i2c_master_setup(void)
{
    I2C2BRG = 53;                           // I2CBRG = [1/(2*Fsck) - PGD]*PBclk - 2
                                            // Fsck = 400kHz, PGD = typ. 104ns, PBclk = 48MHz
    I2C2CONbits.ON = 1;                     // turn on the I2C2 module
}

void i2c_master_start(void)                 // Start a transmission on the I2C bus
{
    I2C2CONbits.SEN = 1;                    // send the start bit
    while(I2C2CONbits.SEN) { ; }            // wait for the start bit to be sent
}

void i2c_master_restart(void)
{
    I2C2CONbits.RSEN = 1;                   // send a restart
    while(I2C2CONbits.RSEN) { ; }           // wait for the restart to clear
}

void i2c_master_send(unsigned char byte)    // send a byte to slave
{
    I2C2TRN = byte;                         // if an address, bit 0 = 0 for write, 1 for read
    while(I2C2STATbits.TRSTAT) { ; }        // wait for the transmission to finish
    if(I2C2STATbits.ACKSTAT)                // if this is high, slave has not acknowledged
    {        
        // ("I2C2 Master: failed to receive ACK\r\n");
    }
}

unsigned char i2c_master_recv(void)         // receive a byte from the slave
{
    I2C2CONbits.RCEN = 1;                   // start receiving data
    while(!I2C2STATbits.RBF) { ; }          // wait to receive the data
    return I2C2RCV;                         // read and return the data
}

void i2c_master_ack(int val)                // sends ACK = 0 (slave should send another byte)
                                            // or NACK = 1 (no more bytes requested from slave)
{                                                                  
    I2C2CONbits.ACKDT = val;                // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;                  // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }          // wait for ACK/NACK to be sent
}

void i2c_master_stop(void)                  // send a STOP:
{
    I2C2CONbits.PEN = 1;                    // comm is complete and master relinquishes bus
    while(I2C2CONbits.PEN) { ; }            // wait for STOP to complete
}

////////////////////////////////////////////////////////////////////////////////

void IMU_init()
{
    i2c_master_setup();
    
    // Configure the IMU registers for our particular purposes
    IMU_write(0x10, 0b10000010);    // 1.66 kHz, +-2g sens, 100 Hz filter
    IMU_write(0x11, 0b10001000);    // 1.66 KhZ, 1000 dps, No full-scale @ 125 dps
    IMU_write(0x12, 0b00000100);    // All defaults, except IF_INC set to '1'
}

void IMU_write(unsigned char reg, unsigned char payload)
{
    unsigned char address = (LSM6DS33_addr << 1);   // Should be 0xD7 for reading, 0xD6 for writing
    
    i2c_master_start();
    i2c_master_send(address | 0);           // WRITE
    i2c_master_send(reg);
    i2c_master_send(payload);
    i2c_master_stop();
}

void IMU_read(unsigned char reg, unsigned char * data)
{
    unsigned char address = (LSM6DS33_addr << 1);   // Should be 0xD7 for reading, 0xD6 for writing
    
    i2c_master_start();
    i2c_master_send(address | 0);           // WRITE
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(address | 1);           // READ
    *data = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
}

void IMU_read_mult(unsigned char reg, unsigned char * data, int N)
{
    unsigned char counter = 0;
    unsigned char address = (LSM6DS33_addr << 1);   // Should be 0xD7 for reading, 0xD6 for writing
    
    i2c_master_start();
    i2c_master_send(address | 0);           // WRITE
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(address | 1);           // READ (Multiple))
    
    for(counter = 0; counter < N; ++counter)
    {
        *(data + counter) = i2c_master_recv();
        
        if(counter < N - 1) { i2c_master_ack(0); }
        else                { i2c_master_ack(1); }
    }
    i2c_master_stop();
}

signed short combine_sensor_data(unsigned char lower, unsigned char upper)
{
    return ((upper << 8) | lower);
}