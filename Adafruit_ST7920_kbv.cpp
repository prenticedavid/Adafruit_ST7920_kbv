// much of this was stolen from https://github.com/BornaBiro/ST7920_GFX_Library
// and SPI code from https://github.com/cbm80amiga/ST7920_SPI
//
// it shows how simple to add hardware to GFX
// uses a 1024 byte buffer.  call lcd.display() to show the buffer
// speed and efficiency is not important.  LCDs are SLOW.

#include <Arduino.h>

#include <SPI.h>
#include "Adafruit_ST7920_kbv.h"

#define st7920_swap(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)))

void Adafruit_ST7920_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (x >= width()) || (y < 0) || (y >= height())) return;
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
        case 1:
            st7920_swap(x, y);
            x = WIDTH - x - 1;
            break;
        case 2:
            x = WIDTH  - x - 1;
            y = HEIGHT - y - 1;
            break;
        case 3:
            st7920_swap(x, y);
            y = HEIGHT - y - 1;
            break;
    }
    uint8_t mask = (0x80 >> (x & 7)); //MSB first
    uint8_t x0 = (x >> 3) & 15;       //16 bytes per 128 columns
    uint8_t y0 = (y > 31) ? 16 : 0;   //rows are stored 0,32,1,33,2,34,..
    uint16_t n = (y & 31) * 32 + y0 + x0;
    if (color == 0) {
        buffer[n] &= ~mask;
    } else if (color == 1) {
        buffer[n] |= mask;
    } else if (color == 2) { //.kbv
        buffer[n] ^= mask;
    }
}

Adafruit_ST7920_kbv::Adafruit_ST7920_kbv(int8_t CS) : Adafruit_GFX(ST7920_WIDTH, ST7920_HEIGHT)
{
    cs = CS;
}

void Adafruit_ST7920_kbv::begin(void)
{
    _xor = 0x00;
    SPI.begin();
    sendCmd(0x30); //LCD_BASIC);
    sendCmd(0x30); //LCD_BASIC);
    sendCmd(0x01); //LCD_CLS);
    delay(2);
    sendCmd(0x06); //LCD_ADDRINC);
    sendCmd(0x0C); //LCD_DISPLAYON);
    //setGfxMode(true);
    sendCmd(0x34); //LCD_EXTEND);
    sendCmd(0x36); //LCD_GFXMODE);
}

void Adafruit_ST7920_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    uint8_t c = (color == ST7920_WHITE) ? 0xFF : 0x00;
    uint8_t *p = getBuffer();
    if (x == 0 && y == 0 && w == width() && h == height()) {
        for (int i = 0; i < 1024; i++, p++) {
            uint8_t oldb = *p;
            if (color == ST7920_INVERSE) c = oldb ^ 0xFF;
            if (oldb != c) {
                *p = c;
            }
        }
        return;
    }
    for (int row =  0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            drawPixel(x + col, y + row, color);
        }
    }
}

void Adafruit_ST7920_kbv::clearDisplay(void)
{
    fillRect(0, 0, width(), height(), ST7920_BLACK);
}

void Adafruit_ST7920_kbv::display()
{
    uint8_t *p = buffer;
    int x = 0, y = 0, n = 0;
    for (y = 0; y < 32; y++) {
        sendCmd(0x80 | y);
        sendCmd(0x80 | 0);
        SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3));
        digitalWrite(cs, HIGH);
        SPI.transfer(0xFA); // data
        for (x = 0; x < 32; x++) { //rows: 0,32,1,33,2,34,...
            uint8_t b = *p++ ^ _xor;
            SPI.transfer(b & 0xF0);
            SPI.transfer(b << 4);
        }
        digitalWrite(cs, LOW);
        SPI.endTransaction();
    }
}

void Adafruit_ST7920_kbv::invertDisplay(bool i)
{
#if 1
    _xor = (i) ? 0xFF : 0x00;
    display();
#else
    sendCmd(0x34); //LCD_EXTEND);
    sendCmd(i ? 0x07 : 0x04);  //not sure how this should work. fails anyway.
    sendCmd(0x36); //LCD_GFXMODE);
#endif
}

void Adafruit_ST7920_kbv::sendCmd(byte b)
{
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3));
    digitalWrite(cs, HIGH);
    SPI.transfer(0xF8);
    SPI.transfer(b & 0xF0);
    SPI.transfer(b << 4);
    digitalWrite(cs, LOW);
    SPI.endTransaction();
}
// ----------------------------------------------------------------

void Adafruit_ST7920_kbv::sendData(byte b)
{
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3));
    digitalWrite(cs, HIGH);
    SPI.transfer(0xFA);
    SPI.transfer(b & 0xF0);
    SPI.transfer(b << 4);
    digitalWrite(cs, LOW);
    SPI.endTransaction();
}
