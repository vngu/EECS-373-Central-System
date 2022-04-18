/*
 * graphicsDisplay.c
 *
 *  Created on: Apr 16, 2022
 *      Author: User
 */
#include "graphicsDisplay.h"

static const uint16_t _width = 320;
static const uint16_t _height = 480;

uint8_t picture[IMAGE_BYTES];
static uint16_t dis_row = 0;
static uint16_t dis_height = 0;
static uint8_t
    initd[] = {
      HX8357_SWRESET, 0x80 + 10/5, // Soft reset, then delay 10 ms
      HX8357D_SETC, 3,
        0xFF, 0x83, 0x57,
      0xFF, 0x80 + 300/5,          // No command, just delay 300 ms
      HX8357_SETRGB, 4,
        0x80, 0x00, 0x06, 0x06,    // 0x80 enables SDO pin (0x00 disables)
      HX8357D_SETCOM, 1,
        0x25,                      // -1.52V
      HX8357_SETOSC, 1,
        0x68,                      // Normal mode 70Hz, Idle mode 55 Hz
      HX8357_SETPANEL, 1,
        0x05,                      // BGR, Gate direction swapped
      HX8357_SETPWR1, 6,
        0x00,                      // Not deep standby
        0x15,                      // BT
        0x1C,                      // VSPR
        0x1C,                      // VSNR
        0x83,                      // AP
        0xAA,                      // FS
      HX8357D_SETSTBA, 6,
        0x50,                      // OPON normal
        0x50,                      // OPON idle
        0x01,                      // STBA
        0x3C,                      // STBA
        0x1E,                      // STBA
        0x08,                      // GEN
      HX8357D_SETCYC, 7,
        0x02,                      // NW 0x02
        0x40,                      // RTN
        0x00,                      // DIV
        0x2A,                      // DUM
        0x2A,                      // DUM
        0x0D,                      // GDON
        0x78,                      // GDOFF
      HX8357D_SETGAMMA, 34,
        0x02, 0x0A, 0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b,
        0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03, 0x02, 0x0A,
        0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b, 0x42, 0x3A,
        0x27, 0x1B, 0x08, 0x09, 0x03, 0x00, 0x01,
      HX8357_COLMOD, 1,
        0x55,                      // 16 bit
      HX8357_MADCTL, 1,
        0xC0,
      HX8357_TEON, 1,
        0x00,                      // TW off
      HX8357_TEARLINE, 2,
        0x00, 0x02,
      HX8357_SLPOUT, 0x80 + 150/5, // Exit Sleep, then delay 150 ms
      HX8357_DISPON, 0x80 +  50/5, // Main screen turn on, delay 50 ms
      0,                           // END OF COMMAND LIST
    };

void graphicsDisplayInit(SPI_HandleTypeDef *hspi) {
	uint8_t cmd, x, numArgs;

	uint8_t* addr = initd;

	while((cmd = *(addr++)) > 0) {
	  if(cmd != 0xFF) {
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_SPI_Transmit(hspi, &cmd, 1, 1000);       // '255' is ignored
	  }
	  x = *(addr++);
	  numArgs = x & 0x7F;
	  if(x & 0x80) {        // If high bit set...
		HAL_Delay(numArgs * 5); // numArgs is actually a delay time (5ms units)
	  } else {              // Otherwise, issue args to command...
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET);
		HAL_SPI_Transmit(hspi,addr,numArgs, 1000);
		addr+=numArgs;
	  }
	}
}

void writeCommand(SPI_HandleTypeDef *hspi, uint8_t cmd){
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &cmd, 1, 1000);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET);
}

void SPI_write16(SPI_HandleTypeDef *hspi, uint16_t num) {
	uint8_t upper = (uint8_t)((num&0xFF00) >> 8);
	uint8_t lower = (uint8_t)(num & 0xFF);
	uint8_t buf[2] = {upper, lower};
	HAL_SPI_Transmit(hspi, buf, 2, 1000);
}

void setAddrWindow(SPI_HandleTypeDef *hspi, uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
  uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
  writeCommand(hspi,HX8357_CASET);
  SPI_write16(hspi,x1);
  SPI_write16(hspi,x2);
  writeCommand(hspi,HX8357_PASET);
  SPI_write16(hspi,y1);
  SPI_write16(hspi,y2);
  writeCommand(hspi,HX8357_RAMWR);
}

void writeInplace(SPI_HandleTypeDef *hspi, uint16_t color) {
  uint8_t lower = color & 0xFF;
  uint8_t upper = (color & 0xFF00) >> 8;
  uint8_t buf[2] = {upper, lower}; // TODO Might be other way around
  HAL_SPI_Transmit(hspi, buf, 2, 1000);
}

void writePixel(SPI_HandleTypeDef *hspi, int16_t x, int16_t y, uint16_t color) {
	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
	setAddrWindow(hspi,x,y,1,1);
	writeInplace(hspi,color);
}

void printPicture(SPI_HandleTypeDef *hspi) {
	for(int i = 0; i < IMAGE_WIDTH; i ++) {
	  for(int j = 0; j < IMAGE_HEIGHT; j++) {
		uint8_t* pix_start = picture + i*IMAGE_HEIGHT*3 + j*3;
		uint16_t color =
				((*(pix_start) >> 3) << 11) |
				((*(pix_start + 1) >> 2) << 5) |
				((*(pix_start + 2) >> 3) << 0);
		writePixel(hspi,i,j,color);
	  }
	}
}

void resetDistributedPP() {
	dis_row = 0;
}

void distributedPrintPicture(SPI_HandleTypeDef *hspi) {
	if(dis_row == IMAGE_WIDTH) return;
	for(int i = 0; i < IMAGE_HEIGHT; i++) {
		uint8_t* pix_start = picture + dis_row*IMAGE_HEIGHT*3 + i*3;
		uint16_t color =
				((*(pix_start) >> 3) << 11) |
				((*(pix_start + 1) >> 2) << 5) |
				((*(pix_start + 2) >> 3) << 0);
		writePixel(hspi, dis_row, i, color);
	}
	dis_row++;
}

