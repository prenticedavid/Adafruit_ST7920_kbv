#define USE_PAR
#define USE_DIYMORE
#define INVERT_PIC 0x00
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

void Adafruit_ST7920_kbv::begin(void)
{
    st7920Init();
    sendCmd(0x30); //LCD_BASIC);
    sendCmd(0x30); //LCD_BASIC);
    sendCmd(0x01); //LCD_CLS);
    delay(2);
    sendCmd(0x06); //LCD_ADDRINC);
    sendCmd(0x0C); //LCD_DISPLAYON);
    //setGfxMode(true);
    sendCmd(0x34); //LCD_EXTEND);
    sendCmd(0x36); //LCD_GFXMODE);
    clearDisplay();
    backlight(true);
    setTextColor(WHITE); //because GFX defaults to 0xFFFF
    _xor = 0x00;
    _left = 0, _rt = 127; //ensure that whole buffer is displayed at start
    _top = 0, _bot = 63;
}

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
    uint8_t *ads = &buffer[n];
    uint8_t d = *ads;
    uint8_t old = d;
    switch (color) {
        case ST7920_WHITE:   d |= mask; break;
        case ST7920_BLACK:   d &= ~mask; break;
        case ST7920_INVERSE: d ^= mask; break;
    }
    if (d != old) { //is not too expensive
        *ads = d;
        if (x < _left) _left = x;
        if (x > _rt) _rt = x;
        if (y < _top) _top = y;
        if (y > _bot) _bot = y;
    }
}

Adafruit_ST7920_kbv::Adafruit_ST7920_kbv(int8_t CS, int8_t LED) : Adafruit_GFX(ST7920_WIDTH, ST7920_HEIGHT)
{
    cs = CS;
    led = LED;
}

void Adafruit_ST7920_kbv::fillRect(int16_t x0, int16_t y0, int16_t w, int16_t h, uint16_t color)
{
#if 1
    // constrain the logical arguments
    if (x0 >= width() || y0 >= height()) return;
    if (x0 < 0) w += x0, x0 = 0; //adding -ve vales
    if (y0 < 0) h += y0, y0 = 0;
    if (x0 + w > width()) w = width() - x0;
    if (y0 + h > height()) h = height() - y0;
    //transform to the physical memory layout
    switch (getRotation()) {
        case 1:
            st7920_swap(x0, y0);
            st7920_swap(w, h);
            x0 = WIDTH - x0 - w;
            break;
        case 2:
            x0 = WIDTH  - x0 - w;
            y0 = HEIGHT - y0 - h;
            break;
        case 3:
            st7920_swap(x0, y0);
            st7920_swap(w, h);
            y0 = HEIGHT - y0 - h;
            break;
    }
    //update the physical buffer memory
    for (uint8_t y = y0; y < y0 + h; y += 1) {
        uint8_t rshift = x0 & 7;
        int16_t ww = w;
        uint8_t *p = getBuffer() + ((y & 31) * 32) + x0 / 8;
        if (y > 31) p += 16;
        for (int x = x0; ww > 0; x += 8, p++) {
            uint8_t lshift = (ww < 8) ? 8 - ww : 0;
            uint8_t mask = 0xFF << lshift;
            if (rshift) {
                mask >>= rshift;
                ww -= 8 - rshift;
                rshift = 0;  //one-off
            }
            else ww -= 8;
            uint8_t oldb = *p;
            uint8_t newb = (oldb & ~mask);
            if (color == ST7920_WHITE) newb |= mask;
            if (color == ST7920_INVERSE) newb |= oldb ^ mask;
            if (newb != oldb) {
                *p = newb;

                if (x < _left) _left = x;
                if (x > _rt) _rt = x;
                if (y < _top) _top = y;
                if (y > _bot) _bot = y;

            }
        }
    }
#else
    uint8_t c = (color == ST7920_WHITE) ? 0xFF : 0x00;
    uint8_t *p = getBuffer();
    if (x0 == 0 && y0 == 0 && w == width() && h == height()) {
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
            drawPixel(x0 + col, y0 + row, color);
        }
    }
#endif
}

void Adafruit_ST7920_kbv::clearDisplay(void)
{
    fillRect(0, 0, width(), height(), ST7920_BLACK);
}

void Adafruit_ST7920_kbv::invertDisplay(bool i)
{
    _xor = (i) ? 0xFF : 0x00;
    display();
}

void Adafruit_ST7920_kbv::display(void)
{
    //st7920Blit((const uint8_t*)buffer, 0);  //use SRAM
    //is not too expensive to minimise I2C traffic
    //extra housekeeping in ks0108BlitRect() is one off
    if (_rt < _left) _left = 0, _rt = 127;
    if (_bot < _top) _top = 0, _bot = 63;
    st7920BlitRect((const uint8_t*)buffer, _left, _top, _rt - _left + 1, _bot - _top + 1, 0);  //use SRAM
    _left = 127;
    _rt = 0;
    _top = 63;
    _bot = 0;
}

void Adafruit_ST7920_kbv::backlight(bool on)
{
    if (led > 0) {
        pinMode(led, OUTPUT);
        digitalWrite(led, on);
    }
    display();   //compatibility with Adafruit_KS0108_kbv class
}

#ifndef USE_PAR
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

void Adafruit_ST7920_kbv::st7920Init(void)
{
    SPI.begin();
}

void Adafruit_ST7920_kbv::st7920BlitRect(const uint8_t *buf, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint8_t isflash)
{
    uint8_t *p;
    int x, y;
    for (y = y0; y < y0 + h; y++) {
        uint8_t y1 = y & 31;        //32 rows 
        uint8_t x1 = x0 / 16;       //word boundary
        x = x1 * 2;
        p = buf + (y1 * 32) + x;    //buffer rows: 0,32,1,33,2,34,...
        if (y > 31) {               //rows: 0,1,2,3,... or 32, 33, ...
            p += 16;                //buffer
            x1 += 8;                //for Graphic RAM Address
        }
        sendCmd(0x80 | y1);
        sendCmd(0x80 | x1);
        SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE3));
        digitalWrite(cs, HIGH);
        SPI.transfer(0xFA); // data
        for ( ; x < x0 + w; x += 8) { //rows: 0,32,1,33,2,34,...
            uint8_t b = *p++ ^ _xor;
            SPI.transfer(b & 0xF0);
            SPI.transfer(b << 4);
        }
        digitalWrite(cs, LOW);
        SPI.endTransaction();
    }
}

#elif defined(USE_DIYMORE)
#include <Wire.h>
#define twi_master_init(kHz) { Wire.begin(); Wire.setClock(1000L * kHz); }

void twi_master_trans(uint8_t slave, uint8_t *txbuf, int txn, uint8_t *rxbuf, int rxn)
{
    Wire.beginTransmission(slave);
    while (txn--) Wire.write(*txbuf++);
    Wire.endTransmission();
    if (rxn) {
        Wire.requestFrom(slave, rxn);    // request 6 bytes from slave device #8

        while (Wire.available()) { // slave may send less than requested
            *rxbuf++ = Wire.read(); // receive a byte as character
        }
    }
}

#define LCD_PSB    (1<<4)   //PA4 
#define LCD_RESET  (1<<2)
#define LCD_DATA   (1<<7)   //PA7
#define LCD_READ   (1<<6)
#define LCD_ENABLE (1<<5)
#define LCD_BLK    (1<<1)   //PA1

#define LCD_CMD_MASK   (LCD_PSB|LCD_DATA|LCD_READ|LCD_ENABLE)

//AUTOINC: BANK=?, SEQOP=0. BYTE_TOGGLE: BANK=0, SEQOP=1.  BYTE_MODE: BANK=1, SEQOP=1
//AUTOINC BANK?: OLATB,data; OLATA,E=1; OLATA,E=0.  9 I2C bytes (old data 6 bytes)
//AUTOINC BANK0: GPIOB,data,E=1; OLATA,E=0.   7 I2C bytes. (old data 6 bytes)
//BYTE_TOGGLE: OLATB,data,E=1; OLATA,E=0.   7 I2C bytes. (old data 6 bytes)
//BYTE_MODE:   OLATB,data; OLATA,E=1,E=0. 6 I2C bytes.  (old data: 2 bytes)
#define MCP23017   0x20
#define IODIRA     0x00   //BANK1  BYTE_MODE is most efficient 
#define IODIRB     0x10
#define IOCON      0x05
#define OLATA      0x0A
#define OLATB      0x1A

static uint8_t __x = 255, __y = 255, LCD_CMD_PORT;
static uint8_t LCD_DATA_OUT;
//uint8_t ks0108Xor = 0, ks0108Led = 1;

static void i2cRegData(uint8_t slave, uint8_t reg, uint8_t data)
{
    uint8_t txbuf[2];
    txbuf[0] = reg;
    txbuf[1] = data;
    twi_master_trans(slave, txbuf, 2, NULL, 0);
}

static void i2cRegData2(uint8_t slave, uint8_t reg, uint8_t data1, uint8_t data2)
{
    uint8_t txbuf[3];
    txbuf[0] = reg;
    txbuf[1] = data1;
    txbuf[2] = data2;
    twi_master_trans(slave, txbuf, 3, NULL, 0);
}

void Adafruit_ST7920_kbv::st7920CmdPort(uint8_t cmd, uint8_t port)
{
    port = (LCD_CMD_PORT & ~LCD_CMD_MASK) | (port & LCD_CMD_MASK) | LCD_PSB;
    if (led) port |= LCD_BLK;
    else port &= ~LCD_BLK;    //data and port can only change when E=0
    if (LCD_DATA_OUT != cmd) i2cRegData(MCP23017, OLATB, cmd);
    i2cRegData2(MCP23017, OLATA, port | LCD_ENABLE, port); //tWH>450ns. tWH=2.25us @400kHz
    LCD_DATA_OUT = cmd;
    LCD_CMD_PORT = port;
}

void Adafruit_ST7920_kbv::sendCmd(uint8_t cmd)
{
    st7920CmdPort(cmd, 0);
}

void Adafruit_ST7920_kbv::sendData(uint8_t data)
{
    st7920CmdPort(data, LCD_DATA);
}

void Adafruit_ST7920_kbv::st7920Init(void)
{
    twi_master_init(400);
    i2cRegData(MCP23017, 0x0A, 0xA0); //BANK=1, SEQOP=1 if BANK0 (POR)
    i2cRegData(MCP23017, 0x05, 0xA0); //BANK=1, SEQOP=1 i.e. BYTE_MODE
    i2cRegData(MCP23017, IODIRA, (LCD_CMD_MASK | LCD_RESET | LCD_BLK) ^ 0xFF); //Output
    i2cRegData(MCP23017, IODIRB, 0xFF ^ 0xFF);                              //Output
    i2cRegData(MCP23017, OLATA, LCD_CMD_PORT = LCD_RESET);
    i2cRegData(MCP23017, OLATA, LCD_CMD_PORT = 0);
    i2cRegData(MCP23017, OLATA, LCD_CMD_PORT = LCD_RESET | LCD_PSB);
}

#if 0
void Adafruit_ST7920_kbv::st7920BlitRect(const uint8_t *buf, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint8_t isflash)
{
    uint8_t *p;
    int x, y;
    uint8_t txbuf[32], cnt, port;
    for (y = y0; y < y0 + h; y++) {
        uint8_t y1 = y & 31;        //32 rows 
        uint8_t x1 = x0 / 16;       //word boundary
        x = x1 * 2;
        p = buf + (y1 * 32) + x;    //buffer rows: 0,32,1,33,2,34,...
        if (y > 31) {               //rows: 0,1,2,3,... or 32, 33, ...
            p += 16;                //buffer
            x1 += 8;                //for Graphic RAM Address
        }
        sendCmd(0x80 | y1);
        sendCmd(0x80 | x1);
        for ( ; x < x0 + w; ) { //copy bytes from buffer
            uint8_t c = ((isflash) ? pgm_read_byte(p) : (*p)) ^ _xor;
            if (LCD_DATA_OUT != c) i2cRegData(MCP23017, OLATB, LCD_DATA_OUT = c);
            port = LCD_DATA | LCD_RESET | LCD_PSB;
            if (led) port |= LCD_BLK;
            cnt = 0;
            txbuf[cnt++] = OLATA;
            do {
                txbuf[cnt++] = port | LCD_ENABLE;
                txbuf[cnt++] = port;
                p++;
                x += 8;
                c = ((isflash) ? pgm_read_byte(p) : (*p)) ^ _xor;
            } while (c == LCD_DATA_OUT && (x + 8 < x0 + w) && (cnt < 30));
            twi_master_trans(MCP23017, txbuf, cnt, NULL, 0); //N strobes
        }
    }
}
#else
void Adafruit_ST7920_kbv::st7920BlitRect(const uint8_t *buf, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t isflash)
{
    uint8_t c, col, row, endcol, *p;
    uint8_t txbuf[32], cnt, port, c_xor = _xor ^ INVERT_PIC;
    for (row = y; row < y + h; row++) {
        uint8_t x0 = x / 16;
        if (row > 31) x0 += 8;
        sendCmd(0x80 | (row & 31));
        sendCmd(0x80 | (x0));   //start col
        p = (uint8_t*)buf + (row & 31) * 32 + 2 * x0; //rows: 0,1,2,3,... or 32, 33, ...
        col = (x / 16) * 2;        //starts on word boundary
        endcol = (x + w + 7) / 8;  //stops on byte boundary
        while (col < endcol) {
            c = ((isflash) ? pgm_read_byte(p) : (*p)) ^ c_xor;
            if (LCD_DATA_OUT != c) i2cRegData(MCP23017, OLATB, LCD_DATA_OUT = c);
            port = LCD_DATA | LCD_RESET | LCD_PSB;
            if (led) port |= LCD_BLK;
            cnt = 0;
            txbuf[cnt++] = OLATA;
            do {
                txbuf[cnt++] = port | LCD_ENABLE;
                txbuf[cnt++] = port;
                p++;
                col++;
                c = ((isflash) ? pgm_read_byte(p) : (*p)) ^ c_xor;
            } while ((col < endcol - 1) && (cnt < 30) && c == LCD_DATA_OUT);
            twi_master_trans(MCP23017, txbuf, cnt, NULL, 0); //N strobes
        }
    }
}
#endif

#else  //Parallel
#define LCD_PSB    (1<<3)   // 
#define LCD_RESET  (1<<5)
#define LCD_DATA   (1<<0)   //
#define LCD_READ   (1<<1)
#define LCD_ENABLE (1<<2)
#define LCD_BLK    (0<<6)   //unused.  LED is hard-wired
#define LCD_CMD_MASK   (LCD_PSB|LCD_DATA|LCD_READ|LCD_ENABLE)

#if defined(__AVR_ATmega328P__) || defined(_CHIP_ATMEGA328P_)
#define LCD_CMD_PORT PORTC
#define LCD_CMD_DIR  DDRC
#define BMASK 0x03
#define write_data(x) {\
        PORTB=(PORTB&~BMASK)|((x)&BMASK);\
        PORTD=(PORTD&BMASK)|((x)&~BMASK);}
#define data_output() { DDRB |= BMASK; DDRD |= ~BMASK; }
#endif

void Adafruit_ST7920_kbv::st7920CmdPort(uint8_t c, uint8_t port)
{
    write_data(c);
    _delay_us(1);
    LCD_CMD_PORT = port | LCD_PSB | LCD_RESET | LCD_ENABLE;
    _delay_us(1);
    LCD_CMD_PORT &= ~LCD_ENABLE;
    _delay_us(32);
}

void Adafruit_ST7920_kbv::sendCmd(byte b)
{
    st7920CmdPort(b, 0);
}

void Adafruit_ST7920_kbv::sendData(byte b)
{
    st7920CmdPort(b, LCD_DATA);
}

void Adafruit_ST7920_kbv::st7920Init(void)
{
    LCD_CMD_DIR |= LCD_CMD_MASK | LCD_RESET;
    data_output();
}

void Adafruit_ST7920_kbv::st7920BlitRect(const uint8_t *buf, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint8_t isflash)
{
    uint8_t *p;
    int x, y;
    for (y = y0; y < y0 + h; y++) {
        uint8_t y1 = y & 31;        //32 rows 
        uint8_t x1 = x0 / 16;       //word boundary
        x = x1 * 2;
        p = buf + (y1 * 32) + x;    //buffer rows: 0,32,1,33,2,34,...
        if (y > 31) {               //rows: 0,1,2,3,... or 32, 33, ...
            p += 16;                //buffer
            x1 += 8;                //for Graphic RAM Address
        }
        sendCmd(0x80 | y1);
        sendCmd(0x80 | x1);
        for ( ; x < x0 + w; x += 8) { //copy bytes from buffer
            uint8_t b = *p++ ^ _xor;
            sendData(b);
        }
    }
}
#endif   //USE_PAR, USE_DIYMORE, 
