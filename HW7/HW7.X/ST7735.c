// functions to operate the ST7735 on the PIC32
// adapted from https://github.com/sumotoy/TFT_ST7735
// and https://github.com/adafruit/Adafruit-ST7735-Library

// Modified version of ST7735.C
// Original supplied courtesy of N. Marchuk, modified by W. Spies, 4/30/2018

// pin connections:
// VCC - 3.3V
// GND - GND
// CS - B7
// RESET - 3.3V
// A0 - B15
// SDA - A1
// SCK - B14
// LED - 3.3V

// B8 is turned into SDI1 but is not used or connected to anything

#include <stdio.h>
#include <xc.h>
#include "ST7735.h"

void SPI1_init()
{
    SDI1Rbits.SDI1R = 0b0100;       // B8 is SDI1
    RPA1Rbits.RPA1R = 0b0011;       // A1 is SDO1
    TRISBbits.TRISB7 = 0;           // B7 is SS
    LATBbits.LATB7 = 1;             // SS starts high

    // A0 / DAT pin
    ANSELBbits.ANSB15 = 0;
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;

    SPI1CON = 0;                    // turn off the spi module and reset it
    SPI1BUF;                        // clear the rx buffer by reading from it
    SPI1BRG = 0;                    // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;        // clear the overflow bit
    SPI1CONbits.CKE = 1;            // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;          // master operation
    SPI1CONbits.ON = 1;             // turn on spi1
}

unsigned char spi_io(unsigned char o)
{
    SPI1BUF = o;
    while(!SPI1STATbits.SPIRBF)     // wait to receive the byte
    {
        ;
    }
    return SPI1BUF;
}

void LCD_command(unsigned char com)
{
    LATBbits.LATB15 = 0;            // DAT
    LATBbits.LATB7 = 0;             // CS
    spi_io(com);
    LATBbits.LATB7 = 1;             // CS
}

void LCD_data(unsigned char dat)
{
    LATBbits.LATB15 = 1;            // DAT
    LATBbits.LATB7 = 0;             // CS
    spi_io(dat);
    LATBbits.LATB7 = 1;             // CS
}

void LCD_data16(unsigned short dat)
{
    LATBbits.LATB15 = 1;            // DAT
    LATBbits.LATB7 = 0;             // CS
    spi_io(dat>>8);
    spi_io(dat);
    LATBbits.LATB7 = 1;             // CS
}

void LCD_init()
{
    SPI1_init();
    int time = 0;
    LCD_command(ST7735_SWRESET);    //software reset
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/2) {}

    LCD_command(ST7735_SLPOUT);     //exit sleep
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/200) {}

    LCD_command(ST7735_FRMCTR1);    //Frame Rate Control (In normal mode/Full colors)
    LCD_data(0x01);
    LCD_data(0x2C);
    LCD_data(0x2D);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_FRMCTR2);    //Frame Rate Control (In normal mode/Full colors)
    LCD_data(0x01);
    LCD_data(0x2C);
    LCD_data(0x2D);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_FRMCTR3);    //Frame Rate Control (In normal mode/Full colors)
    LCD_data(0x01);
    LCD_data(0x2C);
    LCD_data(0x2D);
    LCD_data(0x01);
    LCD_data(0x2C);
    LCD_data(0x2D);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_INVCTR);     //display inversion
    LCD_data(0x07);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_PWCTR1);
    LCD_data(0x0A);                 //4.30 - 0x0A
    LCD_data(0x02);                 //0x05
    LCD_data(0x84);                 //added auto mode
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_PWCTR2);
    LCD_data(0xC5);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command( ST7735_PWCTR3);
    LCD_data(0x0A);
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command( ST7735_PWCTR4);
    LCD_data(0x8A);
    LCD_data(0x2A);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command( ST7735_PWCTR5);
    LCD_data(0x8A);
    LCD_data(0xEE);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_VMCTR1);
    LCD_data(0x0E);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_INVOFF);

    LCD_command(ST7735_MADCTL);
    LCD_data(0xC8);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_COLMOD);
    LCD_data(0x05);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_CASET);
    LCD_data(0x00);
    LCD_data(0x00);
    LCD_data(0x00);
    LCD_data(0x7F);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_RASET);
    LCD_data(0x00);
    LCD_data(0x00);
    LCD_data(0x00);
    LCD_data(0x9F);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_GMCTRP1);
    LCD_data(0x02);
    LCD_data(0x1C);
    LCD_data(0x07);
    LCD_data(0x12);
    LCD_data(0x37);
    LCD_data(0x32);
    LCD_data(0x29);
    LCD_data(0x2D);
    LCD_data(0x29);
    LCD_data(0x25);
    LCD_data(0x2B);
    LCD_data(0x39);
    LCD_data(0x00);
    LCD_data(0x01);
    LCD_data(0x03);
    LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_GMCTRN1);
    LCD_data(0x03);
    LCD_data(0x1D);
    LCD_data(0x07);
    LCD_data(0x06);
    LCD_data(0x2E);
    LCD_data(0x2C);
    LCD_data(0x29);
    LCD_data(0x2D);
    LCD_data(0x2E);
    LCD_data(0x2E);
    LCD_data(0x37);
    LCD_data(0x3F);
    LCD_data(0x00);
    LCD_data(0x00);
    LCD_data(0x02);
    LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/1000) {}

    LCD_command(ST7735_NORON);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/100) {}

    LCD_command(ST7735_DISPON);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/10) {}

    LCD_command(ST7735_MADCTL);     // rotation
    LCD_data(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color)
{
    // check boundary
    LCD_setAddr(x,y,x+1,y+1);
    LCD_data16(color);
}

void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1)
{
    LCD_command(ST7735_CASET);      // Column
    LCD_data16(x0);
    LCD_data16(x1);

    LCD_command(ST7735_RASET);      // Page
    LCD_data16(y0);
    LCD_data16(y1);

    LCD_command(ST7735_RAMWR);      // Into RAM
}

void LCD_clearScreen(unsigned short color)
{
    int i;
    LCD_setAddr(0,0,_GRAMWIDTH,_GRAMHEIGH);
    for (i = 0;i < _GRAMSIZE; i++)
    {
        LCD_data16(color);
    }
}

////////////////////////////////////////////////////////////////////////////////

void LCD_drawChar(unsigned short x, unsigned short y, unsigned char character, unsigned short color1, unsigned short color2)
{
    unsigned char col = 0;
    unsigned char row = 0;
    unsigned char sel = character - 0x20;
    unsigned char pixels = 0;
    
    for(col = 0; col < 5; ++col)
    {
        pixels = ASCII[sel][col];
        
        for(row = 0; row < 8; ++row)
        {
            if(x+col < 128)
            {
                if(pixels >> row & 1)   { LCD_drawPixel(x+col, y+row, color1); }
                else                    { LCD_drawPixel(x+col, y+row, color2); }
            }
        }
    }
}

void LCD_drawString(unsigned short x, unsigned short y, unsigned char* message, unsigned short color1, unsigned short color2)
{
    unsigned short count = 0;
    unsigned short offset = 0;
    
    while(message[count])
    {
        LCD_drawChar(x+offset, y, message[count], color1, color2);
        
        count++;
        offset += 5;
    }
}

void LCD_drawImuFeedbackBars(signed short x_accel, signed short y_accel)
{
    unsigned short height = 0;
    signed short length = 0;
    unsigned short deadzone = 384;
    signed short x_range;
    signed short y_range;
    
    unsigned short x_cent = 62;
    unsigned short y_cent = 78;
    
    // Determine what "range" of the respective bars are filled
    // "X"
    if(x_accel > deadzone)
    {
        x_range = (x_accel-deadzone)/320;
        if(x_range > 50)   { x_range = 50; }
    }
    else if(x_accel < -1*deadzone)
    {
        x_range = (x_accel+deadzone)/320;
        if(x_range < -50)   { x_range = -50; }
    }
    else    { x_range = 0; }
    
    // "Y"
    if(y_accel > deadzone)
    {
        y_range = (y_accel-deadzone)/320;
        if(y_range > 50)   { y_range = 50; }
    }
    else if(y_accel < -1*deadzone)
    {
        y_range = (y_accel+deadzone)/320;
        if(y_range < -50)   { y_range = -50; }
    }
    else    { y_range = 0; }
    
    
    // Center of accel axis display (since axes > 1 height, need a deadzone to keep things looking normal)
    for(height = 0; height < 4; ++height)
    {
        LCD_drawPixel(x_cent+height, y_cent,   WHITE);
        LCD_drawPixel(x_cent+height, y_cent+1, WHITE);
        LCD_drawPixel(x_cent+height, y_cent+2, WHITE);
        LCD_drawPixel(x_cent+height, y_cent+3, WHITE);
    }
    
    // "X" Acceleration
    if(x_range > 0)         // Positive case (+ side bar)
    {
        for(length = 0; length < 50; ++length)
        {
            for(height = 0; height < 4; ++height)
            {
                if(length <= x_range)   { LCD_drawPixel(x_cent+length+4, y_cent+height, YELLOW); }
                else                    { LCD_drawPixel(x_cent+length+4, y_cent+height, BLUE); }
                
                // This constantly clears the opposite side measurement bar
                LCD_drawPixel(x_cent+length-50, y_cent+height, BLUE);
            }
        }
    }
    else if(x_range < 0)    // Negative case (- side bar)
    {
        for(length = -50; length < 0; ++length)
        {
            for(height = 0; height < 4; ++height)
            {
                if(length >= x_range)   { LCD_drawPixel(x_cent+length, y_cent+height, YELLOW); }
                else                    { LCD_drawPixel(x_cent+length, y_cent+height, BLUE); }
                
                // This constantly clears the opposite side measurement bar
                LCD_drawPixel(x_cent+length+54, y_cent+height, BLUE);
            }
        }
    }
    else
    {
        for(length = 0; length < 50; ++length)
        {
            LCD_drawPixel(x_cent+length+4, y_cent,      BLUE);
            LCD_drawPixel(x_cent+length+4, y_cent+1,    BLUE);
            LCD_drawPixel(x_cent+length+4, y_cent+2,    BLUE);
            LCD_drawPixel(x_cent+length+4, y_cent+3,    BLUE);
            
            LCD_drawPixel(x_cent-length-1, y_cent,      BLUE);
            LCD_drawPixel(x_cent-length-1, y_cent+1,    BLUE);
            LCD_drawPixel(x_cent-length-1, y_cent+2,    BLUE);
            LCD_drawPixel(x_cent-length-1, y_cent+3,    BLUE);
        }
    }
    
    // "Y" Acceleration
    if(y_range > 0)         // Positive case (+ side bar)
    {
        for(length = 0; length < 50; ++length)
        {
            for(height = 0; height < 4; ++height)
            {
                if(length <= y_range)   { LCD_drawPixel(x_cent+height, y_cent-length, YELLOW); }
                else                    { LCD_drawPixel(x_cent+height, y_cent-length, BLUE); }
                
                // This constantly clears the opposite side measurement bar
                LCD_drawPixel(x_cent+height, y_cent+length+4, BLUE);
            }
        }
    }
    else if(y_range < 0)    // Negative case (- side bar)
    {
        for(length = -50; length < 0; ++length)
        {
            for(height = 0; height < 4; ++height)
            {
                if(length >= y_range)   { LCD_drawPixel(x_cent+height, y_cent-length+3, YELLOW); }
                else                    { LCD_drawPixel(x_cent+height, y_cent-length+3, BLUE); }
                
                // This constantly clears the opposite side measurement bar
                LCD_drawPixel(x_cent+height, y_cent-length-51, BLUE);
            }
        }
    }
    else
    {
        for(length = 0; length < 50; ++length)
        {
            LCD_drawPixel(x_cent,   y_cent-length-1,    BLUE);
            LCD_drawPixel(x_cent+1, y_cent-length-1,    BLUE);
            LCD_drawPixel(x_cent+2, y_cent-length-1,    BLUE);
            LCD_drawPixel(x_cent+3, y_cent-length-1,    BLUE);
            
            LCD_drawPixel(x_cent,   y_cent+length+4,    BLUE);
            LCD_drawPixel(x_cent+1, y_cent+length+4,    BLUE);
            LCD_drawPixel(x_cent+2, y_cent+length+4,    BLUE);
            LCD_drawPixel(x_cent+3, y_cent+length+4,    BLUE);
        }
    }
}

void LCD_drawFPS(unsigned short x, unsigned short y)
{
    unsigned char message[30];
    int elapsed = _CP0_GET_COUNT();
    float fps = (24000000.0 / (float) elapsed);
    
    sprintf(message, "FPS : %.4f", fps);
    LCD_drawString(x, y, message, WHITE, BLACK);
}
