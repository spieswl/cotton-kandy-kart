#include <xc.h>                         // processor SFR definitions
#include <sys/attribs.h>                // __ISR macro
#include <math.h>
#include "spi.h"                        // SPI interface

// DEVCFG0
#pragma config DEBUG = OFF              // no debugging
#pragma config JTAGEN = OFF             // no jtag
#pragma config ICESEL = ICS_PGx1        // use PGED1 and PGEC1
#pragma config PWP = OFF                // no write protect
#pragma config BWP = OFF                // no boot write protect
#pragma config CP = OFF                 // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL           // use primary oscillator with pll
#pragma config FSOSCEN = OFF            // turn off secondary oscillator
#pragma config IESO = OFF               // no switching clocks
#pragma config POSCMOD = HS             // high speed crystal mode
#pragma config OSCIOFNC = OFF           // disable secondary osc
#pragma config FPBDIV = DIV_1           // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD           // do not enable clock switch
#pragma config WDTPS = PS1              // use slowest wdt
#pragma config WINDIS = OFF             // wdt no window mode
#pragma config FWDTEN = OFF             // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25     // wdt window at 25%

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24         // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2         // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2         // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON              // USB clock on

// DEVCFG3
#pragma config USERID = 1               // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF           // allow multiple reconfigurations
#pragma config IOL1WAY = OFF            // allow multiple reconfigurations
#pragma config FUSBIDIO = ON            // USB pins controlled by USB module
#pragma config FVBUSONIO = ON           // USB BUSON controlled by USB module


int get_sinusoid(int counter)
{
    int voltage;
    double phase_calc;
    
    phase_calc = sin(2.0 * M_PI * counter / 100);
    
    voltage = 512 + (511 * phase_calc);
    
    return voltage;
}

int get_triang(int counter)
{
    int voltage = 0;
    
    if(counter < 100)
    {
        voltage = (int) ((float) counter * (1023.0 / 100.0));
    }
    else if (counter < 200)
    {
        voltage = (int) ((float) (200 - counter) * (1023.0 / 100.0));
    }
    
    return voltage;
}

void set_voltage(unsigned char channel, int voltage)
{
    unsigned char byte1, byte2;
    unsigned char resp;
    
    // Voltage scaling to counts - max. is 1024
    if(voltage > 1024) { voltage = 1024; }
    else if(voltage < 0) { voltage = 0; }
    
    // SPI MOSI packet configuration - channel, buffer, gain select, shutdown control
    byte1 = channel << 7 | 0b01110000;
    byte1 = byte1 | (voltage >> 6);     // Add the upper 4 bits of vCount to the bottom of byte1
    byte2 = voltage << 2;               // Pack the upper 6 bits of vCount to the top of byte2         
    
    CS = 0;
    resp = SPI_txrx(byte1);
    resp = SPI_txrx(byte2);
    CS = 1;
}

int main() {

    __builtin_disable_interrupts();
    
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    init_SPI1();
    
    __builtin_enable_interrupts();
    
    int count = 0;
    int v_sinusoid, v_triang;
    
    while(1)
    {
        _CP0_SET_COUNT(0);
        
        count++;
        if(count >= 200)
        {
            count = 0;
        }
        
        v_sinusoid = get_sinusoid(count);
        v_triang = get_triang(count);
        
        set_voltage(0, v_sinusoid);
        set_voltage(1, v_triang);
        
        while(_CP0_GET_COUNT() < 48000000/2/1000)
        {
            ;
        }
    }
}