#include <xc.h>                         // processor SFR definitions
#include <sys/attribs.h>                // __ISR macro
#include <math.h>
#include <stdio.h>
#include "ST7735.h"                     // ST7735 LCD header

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

void LCD_drawChar(unsigned short x, unsigned short y, unsigned char character, unsigned short color1, unsigned short color2)
{
    unsigned char col = 0;
    unsigned char row = 0;
    unsigned char sel = character - 0x20;
    unsigned char pixels = 0;
    
    for(col = 0; col < 5; ++col)
    {
        pixels = ASCII[sel][col];
        
        for(row = 0; row < 8; row++)
        {
            if(pixels >> row & 1)   { LCD_drawPixel(x+col, y+row, color1); }
            else                    { LCD_drawPixel(x+col, y+row, color2); }
        }
    }
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
    
    // Initialize LCD display
    LCD_init();
    
    // Clear LCD display and display BLACK
    LCD_clearScreen(BLACK);
    
    __builtin_enable_interrupts();
    
    while(1)
    {
        _CP0_SET_COUNT(0);

        LCD_drawPixel(10,10,RED);
        LCD_drawPixel(30,10,GREEN);
        LCD_drawPixel(10,30,BLUE);
        
        LCD_drawChar(50,10,'A',WHITE,BLACK);
        LCD_drawChar(55,10,'B',BLUE,BLACK);
        LCD_drawChar(60,10,'C',RED,BLACK);
        LCD_drawChar(65,10,'D',MAGENTA,BLACK);
        
        
                
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