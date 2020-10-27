#ifndef ADAFRUIT_ST7920_kbv_H_
#define ADAFRUIT_ST7920_kbv_H_   100

#include "Adafruit_GFX.h"

#define SPI_SPEED      1000000 //no faster than 1MHz
#define ST7920_HEIGHT  64      //64 pixels tall display
#define ST7920_WIDTH   128     //128 pixels wide display
#define ST7920_BLACK   0
#define ST7920_WHITE   1
#define ST7920_INVERSE 2
#define BLACK                     ST7920_BLACK    ///< Draw 'off' pixels
#define WHITE                     ST7920_WHITE    ///< Draw 'on' pixels
#define INVERSE                   ST7920_INVERSE  ///< Invert pixels

class Adafruit_ST7920_kbv : public Adafruit_GFX {

    public:
        Adafruit_ST7920_kbv(int8_t CS, int8_t LED = -1);  //
        void     begin(void);                       // you only need the constructor
        virtual void     drawPixel(int16_t x, int16_t y, uint16_t color);  // and these three
        virtual void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
#if 0
        virtual void     drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
            fillRect(x, y, 1, h, color);
        }
        virtual void     drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
            fillRect(x, y, w, 1, color);
        }
        virtual void     fillScreen(uint16_t color)                                     {
            fillRect(0, 0, _width, _height, color);
        }
#endif
        void clearDisplay(void);
        //virtual void     setRotation(uint8_t r);
        virtual void     invertDisplay(bool i);
        bool     getPixel(int16_t x, int16_t y);
        uint8_t *getBuffer(void) {
            return buffer;
        }
        void     display(void);
        void     backlight(bool on);
        void     dim(bool off) { backlight(!off); } //like Adafruit_SSD1306

    protected:
        uint8_t _left, _rt, _top, _bot;
        void sendCmd(uint8_t b);
        void sendData(uint8_t b);

    private:
        uint8_t buffer[1024];
        uint8_t _xor;
        int8_t cs, led;
        void st7920Init(void);                       // interface specific
        void st7920CmdPort(uint8_t cmd, uint8_t port);
        void st7920BlitRect(const uint8_t *buf, uint8_t x, uint8_t y, 
                            uint8_t w, uint8_t h, uint8_t isflash);
};
#endif
