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

signed short combine_sensor_data(unsigned char lower, unsigned char upper)
{
    return ((upper << 8) | lower);
}

int main() {

    unsigned char string_disp[30];
    unsigned char sensor_id[1];
    unsigned char sensor_data[14];
    
    signed short temperature, gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    
    ////////////////////////////////////////////////////////////
    __builtin_disable_interrupts();
    
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    ////////////////////////////////////////////////////////////
    
    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;               // Sinking input
    TRISAbits.TRISA4 = 0;               // Sourcing output
    LATAbits.LATA4 = 1;
    
    // Initialize IMU (and I2C2 interface)
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    IMU_init();
    IMU_write(0x10, 0b10000010);    // 1.66 kHz, +-2g sens, 100 Hz filter
    IMU_write(0x11, 0b10001000);    // 1.66 KhZ, 1000 dps, No full-scale @ 125 dps
    IMU_write(0x12, 0b00000100);    // All defaults, except IF_INC set to '1'
    IMU_read(0x0F, sensor_id);
    
    // Initialize LCD display (and SPI1 interface))
    LCD_init();
    
    // Clear LCD display and display BLACK
    LCD_clearScreen(BLACK);
    
    __builtin_enable_interrupts();
    
    //// Hello World text string
    //sprintf(string_disp, "Hello World!");
    //LCD_drawString(10, 15, string_disp, WHITE, BLACK);
        
    // IMU Sensor address readout
    sprintf(string_disp, "IMU Address : %d   ", sensor_id[0]);
    LCD_drawString(10, 5, string_disp, WHITE, BLACK);
    
    while(1)
    {        
        _CP0_SET_COUNT(0);
        
        // IMU sensor data readout
        IMU_read_mult(0x20, sensor_data, 14);
        
        // NOTE: Flipped the sign of all sensor readings to make it "agree" with my expectations
        //temperature = -1*(combine_sensor_data(sensor_data[0], sensor_data[1]));
        //gyroX = -1*(combine_sensor_data(sensor_data[2], sensor_data[3]));
        //gyroY = -1*(combine_sensor_data(sensor_data[4], sensor_data[5]));
        //gyroZ = -1*(combine_sensor_data(sensor_data[6], sensor_data[7]));
        accelX = -1*(combine_sensor_data(sensor_data[8], sensor_data[9]));
        accelY = -1*(combine_sensor_data(sensor_data[10], sensor_data[11]));
        accelZ = -1*(combine_sensor_data(sensor_data[12], sensor_data[13]));
        
        // Display IMU readings
        //sprintf(string_disp, "Temperature : %d   ", temperature);
        //LCD_drawString(10, 25, string_disp, WHITE, BLACK);
        //sprintf(string_disp, "Gyro (X) : %d   ", gyroX);
        //LCD_drawString(10, 35, string_disp, WHITE, BLACK);
        //sprintf(string_disp, "Gyro (Y) : %d   ", gyroY);
        //LCD_drawString(10, 45, string_disp, WHITE, BLACK);
        //sprintf(string_disp, "Gyro (Z) : %d   ", gyroZ);
        //LCD_drawString(10, 55, string_disp, WHITE, BLACK);
        sprintf(string_disp, "Accel (X):");
        LCD_drawString(10, 25, string_disp, WHITE, BLACK);
        sprintf(string_disp, "%d    ", accelX);
        LCD_drawString(10, 35, string_disp, WHITE, BLACK);
        sprintf(string_disp, "Accel (Y):");
        LCD_drawString(10, 45, string_disp, WHITE, BLACK);
        sprintf(string_disp, "%d    ", accelY);
        LCD_drawString(10, 55, string_disp, WHITE, BLACK);
        //sprintf(string_disp, "Accel (Z) : %d   ", accelZ);
        //LCD_drawString(10, 65, string_disp, WHITE, BLACK);
        
        // IMU accelerometer measurement bars
        LCD_drawImuFeedbackBars(accelX, accelY);        
                
        // FPS Counter
        LCD_drawFPS(10, 145);
        
        // LED Heartbeat Code
        while(_CP0_GET_COUNT() < (48000000/2)/20)   // Inner computation gets loop to 1Hz
        {
            ;
        }
        LATAINV = 0x10;                 // Invert LATA pin 4
    }
}