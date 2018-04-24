#include <xc.h>                         // processor SFR definitions
#include <sys/attribs.h>                // __ISR macro
#include <math.h>
#include "i2c_master.h"                 // I2C header file

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


unsigned char get_IOExp(unsigned char reg)
{
    unsigned char value;
    unsigned char address = (MCP23008_addr << 1);   // Should be 0x4F for reading, 0x4E for writing
    
    i2c_master_start();
    i2c_master_send(address | 0);       // WRITE
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(address | 1);       // READ
    value = i2c_master_recv();
    i2c_master_ack(1);                  // No more bytes needed from slave
    i2c_master_stop();
    
    return value;
}

void set_IOExp(unsigned char reg, unsigned char payload)
{
    unsigned char address = (MCP23008_addr << 1);   // Should be 0x4F for reading, 0x4E for writing
    
    i2c_master_start();
    i2c_master_send(address | 0);       // WRITE
    i2c_master_send(reg);
    i2c_master_send(payload);
    i2c_master_stop();
}

void init_IOExp()                       // Use after setting up I2C communication
{
    i2c_master_setup();
    set_IOExp(0x00,0b11110000);
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
    
    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;               // Sinking input
    TRISAbits.TRISA4 = 0;               // Sourcing output
    LATAbits.LATA4 = 1;
    
    // Turn off ANSEL B pins we are using for I2C
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    __builtin_enable_interrupts();
    
    // Initialization for MCP23008 I/O expander
    init_IOExp();
    
    unsigned char exPort1 = 0x00;
    
    while(1)
    {
        _CP0_SET_COUNT(0);
        
        // I2C Communication
        exPort1 = get_IOExp(0x09);
        if(exPort1 >> 7)    { set_IOExp(0x0A,0); }  // When GP7 is high, GP0 is low
                                                    // (Button contact is closed, light is OFF)
        else                { set_IOExp(0x0A,1); }  // When GP7 is low, GP0 is high
                                                    // (Button contact is open, light is ON)
        
        // LED Code
        while(_CP0_GET_COUNT() < 4800000)    // Wait some amount of time
        {
            while(!PORTBbits.RB4)
            {
                LATAbits.LATA4 = 0;     // Hold LATA pin 4 to LOW
            }
        }
        
        LATAINV = 0x10;                 // Invert LATA pin 4
    }
}