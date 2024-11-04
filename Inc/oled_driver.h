/**
  ******************************************************************************
  * @file    oled_driver.h
  * @author  Albert
  * @version V1.0.0
  * @date    30-Jan-2018
  * @brief   Header for oled_driver.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OLED_DRIVER_H
#define __OLED_DRIVER_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include <stdarg.h>

/* Defined constants ---------------------------------------------------------*/

#define SSD1305_SETLOWCOLUMN 0x00
#define SSD1305_SETHIGHCOLUMN 0x10

#define SSD1305_MEMORYMODE 0x20
#define SSD1305_SETCOLADDR 0x21
#define SSD1305_SETPAGEADDR 0x22

#define SSD1305_SETSTARTLINE 0x40

#define SSD1305_SETCONTRAST 0x81
#define SSD1305_SETBRIGHTNESS 0x82

#define SSD1305_SEGREMAP 0xA0
#define SSD1305_DISPLAYALLON_RESUME 0xA4
#define SSD1305_DISPLAYALLON 0xA5
#define SSD1305_NORMALDISPLAY 0xA6
#define SSD1305_INVERTDISPLAY 0xA7
#define SSD1305_SETMULTIPLEX 0xA8
#define SSD1305_DISPLAYDIM 0xAC
#define SSD1305_MASTERCONFIG 0xAD
#define SSD1305_DISPLAYOFF 0xAE
#define SSD1305_DISPLAYON 0xAF

#define SSD1305_SETPAGESTART 0xB0

#define SSD1305_COMSCANINC 0xC0
#define SSD1305_COMSCANDEC 0xC8

#define SSD1305_SETDISPLAYOFFSET 0xD3
#define SSD1305_SETDISPLAYCLOCKDIV 0xD5
#define SSD1305_SETAREACOLOR 0xD8
#define SSD1305_SETPRECHARGE 0xD9
#define SSD1305_SETCOMPINS 0xDA
#define SSD1305_SETVCOMLEVEL 0xDB


/* Exported definitions ------------------------------------------------------*/
#define OLED_I2C_ADDRESS		0x78

// Display Setting
#define OLED_OFF						0
#define OLED_ON             1

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern u8 bOledInvertor;

/* Exported functions --------------------------------------------------------*/
void OLED_Driver_Init(void);
void OLED_Clear_Screen(void);
void OLED_Full_Screen(void);
void OLED_Screen_Display(u8 bIsDisplayOn);
void OLED_Write_Battery(uint8_t u8Char);

void vPrintText(uint8_t u8Line,char *str,...);
void vDrawBattery(uint8_t u8Char,uint8_t HalfMode);
void vSetCursor(uint8_t u8Value);

/* Exported macro ------------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* __OLED_DRIVER_H */
