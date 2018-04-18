#ifndef _SPI_H
#define _SPI_H

#define CS LATBbits.LATB7               // SPI interface Chip Select pin

void init_SPI();

unsigned char SPI_txrx(unsigned char o);

#endif /* _SPI_H */