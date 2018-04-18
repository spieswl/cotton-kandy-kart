#include <proc/p32mx250f128b.h>
#include "spi.h"

void init_SPI1()
{
    // Pins used in this configuration (w/ PIC32MX250F128B)
    // B14 (25) : SPI Clock 1 [SCK1]
    // B7 (16)  : Chip Select 1 [SS1]
    // B8 (17)  : Master In Slave Out (MISO) [SDI1]
    // B13 (24) : Master Out Slave In (MOSI) [SDO1]
    
    TRISBbits.TRISB7 = 0;
    SDI1Rbits.SDI1R = 0b0100;   // RPB8
    RPB13Rbits.RPB13R = 0b0011; // RPB13
    CS = 1;
    
    // Setup SPI channel 1 - Master configuraion
    SPI1CON = 0;                // Disable and reset SPI module
    SPI1BUF;                    // Clear the RX buffer by reading from it
    SPI1BRG = 0x1;              // Set baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;    // Clear the overflow bit
    SPI1CONbits.CKE = 1;        // Data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;      // Master enable
    SPI1CONbits.MODE16 = 0;     // 16-bit Channel, 32-bit Frame mode
    SPI1CONbits.MODE32 = 0;     
    SPI1CONbits.SSEN = 0;       // Control SPI CS function with digital output
    SPI1CONbits.ON = 1;         // Turn on SPI channel 1
}

unsigned char SPI_txrx(unsigned char o)
{
    SPI1BUF = o;
    
    while(!SPI1STATbits.SPIRBF)
    {
        ;                       // Wait until SPI1 buffer is populated
    }
    
    return SPI1BUF;    
}