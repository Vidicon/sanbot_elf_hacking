///*
// * SSD1305.c
// *
// *  Created on: Oct 29, 2024
// *      Author: bram
// */
#include "SSD1305_eyes.h"
#include "font.h"

GPIO_PIN resetPin_;
GPIO_PIN dcPin_;

GPIO_PIN toGPIO(GPIO_TypeDef *port, uint16_t pin)
{
	GPIO_PIN gpio_pin;
	gpio_pin.port = port;
	gpio_pin.pin = pin;
	return gpio_pin;
};

void SSD1305_writeCMD(OLED_HandleTypeDef *oled, uint8_t cmd)
{
	HAL_GPIO_WritePin(oled->cs.port, oled->cs.pin, 0);
	HAL_GPIO_WritePin(dcPin_.port, dcPin_.pin, 0);
	HAL_SPI_Transmit(oled->hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(dcPin_.port, dcPin_.pin, 1);
	HAL_GPIO_WritePin(oled->cs.port, oled->cs.pin, 1);
}

void SSD1305_reset()
{
	HAL_GPIO_WritePin(resetPin_.port, resetPin_.pin, 0);
	HAL_Delay(2);
	HAL_GPIO_WritePin(resetPin_.port, resetPin_.pin, 1);
	HAL_Delay(5);
}

void SSD1305_init(OLED_HandleTypeDef *h_left_oled, OLED_HandleTypeDef *h_right_oled)
{

	resetPin_ = h_left_oled->reset;
	dcPin_ = h_left_oled->dc;
	SSD1305_reset();

	static const uint8_t init_cmds[] = {
	  SSD1305_DISPLAYOFF,          // 0xAE
	  SSD1305_SETLOWCOLUMN | 0x2,  // low col = 0
	  SSD1305_SETHIGHCOLUMN | 0x0, // hi col = 0
	  SSD1305_SETSTARTLINE | 0x0,  // line #0
//	  SSD1305_DEACTIVATESCROLL,
	  SSD1305_SETCONTRAST,
	  0xCF,
	  SSD1305_SEGREMAP | 0x01,
	  SSD1305_COMSCANDEC,
	  SSD1305_NORMALDISPLAY, // 0xA6
	  SSD1305_SETMULTIPLEX,
	  0x3F, // 0xA8, 0x3F (1/64)
	  SSD1305_SETCONTRAST, // repeated
	  0xCF,
	  SSD1305_SETDISPLAYOFFSET,
	  0x00, // 0xD3, 0x40
	  SSD1305_SETDISPLAYCLOCKDIV,
	  0x80, // 0xD5, 0xF0
	  SSD1305_SETPRECHARGE,
	  0xF1, // 0xd9, 0xF1
	  SSD1305_SETCOMPINS,
	  0x12, // 0xDA, 0x12
	  SSD1305_SETVCOMLEVEL,
	  0x40,
	  SSD1305_MEMORYMODE,
	  0x02,
	  SSD1305_CHARGEPUMPSETTING,
	  SSD1305_ENABLECHARGEPUMP,
	  SSD1305_DISPLAYALLON_RESUME,
	  SSD1305_NORMALDISPLAY,
	  SSD1305_SETCONTRAST, // repeated
	  0xFF,
	  SSD1305_DISPLAYON,
	  SSD1305_DISPLAYON
	};

	for(int i = 0; i < sizeof(init_cmds); i++)
	{
		SSD1305_writeCMD(h_left_oled, init_cmds[i]);
	}



	int bufferCount = 128;
	int bankcount = 8;
	for(int i = 0; i < 	bankcount; i++)
	{

		SSD1305_writeCMD(h_left_oled, (SSD1305_SETPAGESTART + i));
		SSD1305_writeCMD(h_left_oled, (SSD1305_SETHIGHCOLUMN));
		SSD1305_writeCMD(h_left_oled, (SSD1305_SETLOWCOLUMN));

		HAL_GPIO_WritePin(h_left_oled->cs.port, h_left_oled->cs.pin, 0);
		for(int y = 0; y < 	bufferCount; y++)
		{
			HAL_SPI_Transmit(h_left_oled->hspi, &sara_logo[(y) + (i * 128)], 1, 100);
		}
		HAL_GPIO_WritePin(h_left_oled->cs.port, h_left_oled->cs.pin, 1);
	}


	//################### right #################################
	for(int i = 0; i < sizeof(init_cmds); i++)
	{
		SSD1305_writeCMD(h_right_oled, init_cmds[i]);
	}

	for(int i = 0; i < 	bankcount; i++)
	{

		SSD1305_writeCMD(h_right_oled, (SSD1305_SETPAGESTART + i));
		SSD1305_writeCMD(h_right_oled, (SSD1305_SETHIGHCOLUMN));
		SSD1305_writeCMD(h_right_oled, (SSD1305_SETLOWCOLUMN));

		HAL_GPIO_WritePin(h_right_oled->cs.port, h_right_oled->cs.pin, 0);
		for(int y = 0; y < 	bufferCount; y++)
		{
			HAL_SPI_Transmit(h_right_oled->hspi, &nobleo_logo[(y) + (i * 128)], 1, 100);
		}
		HAL_GPIO_WritePin(h_right_oled->cs.port, h_right_oled->cs.pin, 1);
	}
	//################### right #################################


	/*
	 *
	 *
	 * 	  SSD1305_SETBRIGHTNESS,
	  0x10, // 0x82, 0x10
	 * */




}

void SSD1305_writeDisplay(OLED_HandleTypeDef *holed, const uint8_t * data)
{

	for(int i = 0; i < 	8; i++)
	{

		SSD1305_writeCMD(holed, (SSD1305_SETPAGESTART + i));
		SSD1305_writeCMD(holed, (SSD1305_SETHIGHCOLUMN));
		SSD1305_writeCMD(holed, (SSD1305_SETLOWCOLUMN));

		HAL_GPIO_WritePin(holed->cs.port, holed->cs.pin, 0);
		for(int y = 0; y < 	128; y++)
		{
			HAL_SPI_Transmit(holed->hspi, &data[(y) + (i * 128)], 1, 100);
		}
		HAL_GPIO_WritePin(holed->cs.port, holed->cs.pin, 1);
	}
}

void add_text(const char * text, uint8_t * data)
{
	int i = 0;
//	int offset = 0;
	while(text[i] != 0)
	{
		char charcter = text[i];
		for(int x = 0; x < 6; x++)
		{
			data[(i*6) + x] = font[(6*(charcter-0x20))+x];
		}
		i++;
	}
}

//void SSD1305_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIO_CS, uint16_t GPIO_CS_Pin)
//{
//
//	 static const uint8_t init_128x32[] = {
//	      // Init sequence for 128x32 OLED module
//	      SSD1305_DISPLAYOFF,          // 0xAE
//	      SSD1305_SETLOWCOLUMN | 0x0,  // low col = 0
//	      SSD1305_SETHIGHCOLUMN | 0x0, // hi col = 0
//	      SSD1305_SETSTARTLINE | 0x0,  // line #0
//	      0x2E,                        // ??
//	      SSD1305_SETCONTRAST,
//	      0x32, // 0x81, 0x32
//	      SSD1305_SETBRIGHTNESS,
//	      0x10, // 0x82, 0x10
//	      SSD1305_SEGREMAP | 0x01,
//	      SSD1305_NORMALDISPLAY, // 0xA6
//	      SSD1305_SETMULTIPLEX,
//	      0x3F, // 0xA8, 0x3F (1/64)
//	      SSD1305_MASTERCONFIG,
//	      0x8E, // external vcc supply
//	      SSD1305_COMSCANDEC,
//	      SSD1305_SETDISPLAYOFFSET,
//	      0x40, // 0xD3, 0x40
//	      SSD1305_SETDISPLAYCLOCKDIV,
//	      0xF0, // 0xD5, 0xF0
//	      SSD1305_SETAREACOLOR,
//	      0x05,
//	      SSD1305_SETPRECHARGE,
//	      0xF1, // 0xd9, 0xF1
//	      SSD1305_SETCOMPINS,
//	      0x12, // 0xDA, 0x12
//	      SSD1305_SETLUT,
//	      0x3F,
//	      0x3F,
//	      0x3F,
//	      0x3F};8
//
//
//}
