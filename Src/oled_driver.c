/**
  ******************************************************************************
  * @file    oled_driver.c
  * @author  Albert
  * @version V1.0.0
  * @date    30-Jan-2018
  * @brief   OLED driver sub-routine
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "oled_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t m_u8cursor = 0;
u8 bOledInvertor = 0;


static const uint8_t _BATTERY_1610[24*8]=
{
    0x00,0xFC,0x02,0xFA,0xFA,0xFA,0xFA,0xFA,
    0x00,0x01,0x02,0x02,0x02,0x02,0x02,0x02,    
    0xFA,0xFA,0xFA,0xFA,0x02,0xFE,0xF8,0x70,
    0x02,0x02,0x02,0x02,0x02,0x03,0x00,0x00,
    
    0x00,0xFC,0x02,0xFA,0xFA,0xFA,0xFA,0xFA,
    0x00,0x01,0x02,0x02,0x02,0x02,0x02,0x02,    
    0xFA,0x02,0x02,0x02,0x02,0xFE,0xF8,0x70,
    0x02,0x02,0x02,0x02,0x02,0x03,0x00,0x00,
    
    0x00,0xFC,0x02,0xFA,0xFA,0xFA,0x02,0x02,
    0x00,0x01,0x02,0x02,0x02,0x02,0x02,0x02,    
    0x02,0x02,0x02,0x02,0x02,0xFE,0xF8,0x70,
    0x02,0x02,0x02,0x02,0x02,0x03,0x00,0x00,    

    0x00,0xFC,0x02,0x02,0x02,0x02,0x02,0x02,
    0x00,0x01,0x02,0x02,0x02,0x02,0x02,0x02,    
    0x02,0x02,0x02,0x02,0x02,0xFE,0xF8,0x70,
    0x02,0x02,0x02,0x02,0x02,0x03,0x00,0x00,    

    0x00,0xFC,0x22,0x22,0x72,0xFA,0xAA,0x8A,
    0x00,0x01,0x02,0x02,0x02,0x02,0x02,0x02,
    0xFA,0x72,0x52,0x52,0x02,0xFE,0xF8,0x70,
    0x02,0x02,0x02,0x02,0x02,0x03,0x00,0x00,

    // Plugin
    0x00,0xE0,0xE0,0x30,0x30,0x10,0xFC,0xCC,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,    
    0xB4,0xB4,0xB4,0xFC,0xFC,0x48,0x48,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  
};

// 4k memory, 8 * 16
static const uint8_t m_ASCII_Table[256 * 16] = 
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//0
	0xF0,0x08,0x28,0x08,0x08,0x28,0x08,0xF0,0x0F,0x10,0x11,0x13,0x13,0x11,0x10,0x0F,//1
	0xF0,0xF8,0xD8,0xF8,0xF8,0xD8,0xF8,0xF0,0x0F,0x1F,0x1E,0x1C,0x1C,0x1E,0x1F,0x0F,//2
	0xC0,0xE0,0xE0,0xC0,0xE0,0xE0,0xC0,0x00,0x03,0x07,0x0F,0x1F,0x0F,0x07,0x03,0x00,//3
	0x00,0x80,0xC0,0xE0,0xC0,0x80,0x00,0x00,0x01,0x03,0x07,0x0F,0x07,0x03,0x01,0x00,//4
	0x80,0x80,0xE0,0x70,0x70,0xE0,0x80,0x80,0x03,0x03,0x13,0x1C,0x1C,0x13,0x03,0x03,//5
	0x80,0xC0,0xE0,0xF0,0xF0,0xE0,0xC0,0x80,0x01,0x03,0x13,0x1F,0x1F,0x13,0x03,0x01,//6
	0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x07,0x03,0x00,0x00,//7
	0xFF,0xFF,0xFF,0x7F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0xF8,0xF8,0xFC,0xFF,0xFF,//8
	0x00,0x80,0xC0,0x40,0x40,0xC0,0x80,0x00,0x00,0x07,0x0C,0x08,0x08,0x0C,0x07,0x00,//9
	0xFF,0x7F,0x3F,0xBF,0xBF,0x3F,0x7F,0xFF,0xFF,0xF8,0xF3,0xF7,0xF7,0xF3,0xF8,0xFF,//10
	0x00,0x80,0xC0,0xE8,0xB8,0x18,0x78,0x00,0x0F,0x1F,0x10,0x10,0x1F,0x0F,0x00,0x00,//11
	0x00,0xF0,0xF8,0x08,0x08,0xF8,0xF0,0x00,0x00,0x04,0x05,0x1F,0x1F,0x05,0x04,0x00,//12
	0x00,0x00,0xF8,0xF8,0x28,0x28,0x38,0x38,0x18,0x1C,0x1F,0x0F,0x00,0x00,0x00,0x00,//13
	0x00,0xF8,0xF8,0x28,0x28,0x28,0xF8,0xF8,0x38,0x3F,0x1F,0x00,0x00,0x1C,0x1F,0x0F,//14
	0x40,0x40,0x80,0xF0,0xF0,0x80,0x40,0x40,0x05,0x05,0x03,0x1E,0x1E,0x03,0x05,0x05,//15
	0xFC,0xF8,0xF0,0xE0,0xC0,0x80,0x80,0x00,0x1F,0x0F,0x07,0x03,0x01,0x00,0x00,0x00,//16
	0x80,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0x00,0x00,0x00,0x01,0x03,0x07,0x0F,0x1F,0x00,//17
	0x00,0x20,0x30,0xF8,0xF8,0x30,0x20,0x00,0x00,0x02,0x06,0x0F,0x0F,0x06,0x02,0x00,//18
	0x00,0xF8,0xF8,0x00,0x00,0xF8,0xF8,0x00,0x00,0x1B,0x1B,0x00,0x00,0x1B,0x1B,0x00,//19
	0x70,0xF8,0x88,0xF8,0xF8,0x08,0xF8,0xF8,0x00,0x00,0x00,0x1F,0x1F,0x00,0x1F,0x1F,//20
	0x88,0xDC,0x74,0x24,0x64,0xCC,0x88,0x00,0x11,0x33,0x26,0x24,0x2E,0x3B,0x11,0x00,//21
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x00,//22
	0x00,0x20,0x30,0xF8,0xF8,0x30,0x20,0x00,0x00,0x12,0x16,0x1F,0x1F,0x16,0x12,0x00,//23
	0x00,0x20,0x30,0xF8,0xF8,0x30,0x20,0x00,0x00,0x00,0x00,0x1F,0x1F,0x00,0x00,0x00,//24
	0x00,0x00,0x00,0xF8,0xF8,0x00,0x00,0x00,0x00,0x04,0x0C,0x1F,0x1F,0x0C,0x04,0x00,//25
	0x00,0x00,0x00,0x40,0xC0,0x80,0x00,0x00,0x01,0x01,0x01,0x05,0x07,0x03,0x01,0x00,//26
	0x00,0x80,0xC0,0x40,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x05,0x01,0x01,0x01,0x00,//27
	0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x07,0x04,0x04,0x04,0x04,0x04,0x00,//28
	0x00,0x80,0xC0,0x00,0xC0,0x80,0x00,0x00,0x01,0x03,0x07,0x01,0x07,0x03,0x01,0x00,//29
	0x00,0x00,0xC0,0xE0,0xC0,0x00,0x00,0x00,0x0C,0x0F,0x0F,0x0F,0x0F,0x0F,0x0C,0x00,//30
	0x60,0xE0,0xE0,0xE0,0xE0,0xE0,0x60,0x00,0x00,0x01,0x07,0x0F,0x07,0x01,0x00,0x00,//31
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//32
	0x00,0x00,0x70,0xF8,0xF8,0x70,0x00,0x00,0x00,0x00,0x00,0x1B,0x1B,0x00,0x00,0x00,//33
	0x00,0x1C,0x3C,0x00,0x00,0x3C,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//34
	0x40,0xF0,0xF0,0x40,0xF0,0xF0,0x40,0x00,0x04,0x1F,0x1F,0x04,0x1F,0x1F,0x04,0x00,//35
	0x70,0xF8,0x88,0x8E,0x8E,0x98,0x30,0x00,0x0C,0x18,0x10,0x70,0x70,0x1F,0x0F,0x00,//36
	0x60,0x60,0x00,0x00,0x80,0xC0,0x60,0x00,0x18,0x0C,0x06,0x03,0x01,0x18,0x18,0x00,//37
	0x00,0xB0,0xF8,0xC8,0x78,0xB0,0x80,0x00,0x0F,0x1F,0x10,0x11,0x0F,0x1F,0x10,0x00,//38
	0x00,0x20,0x3C,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//39
	0x00,0x00,0xE0,0xF0,0x18,0x08,0x00,0x00,0x00,0x00,0x07,0x0F,0x18,0x10,0x00,0x00,//40
	0x00,0x00,0x08,0x18,0xF0,0xE0,0x00,0x00,0x00,0x00,0x10,0x18,0x0F,0x07,0x00,0x00,//41	
	0x00,0x40,0xC0,0x80,0x80,0xC0,0x40,0x00,0x01,0x05,0x07,0x03,0x03,0x07,0x05,0x01,//42	
	0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x01,0x01,0x07,0x07,0x01,0x01,0x00,//43
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x3C,0x1C,0x00,0x00,0x00,//44
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,//45
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,//46
	0x00,0x00,0x00,0x00,0x80,0xC0,0x60,0x00,0x18,0x0C,0x06,0x03,0x01,0x00,0x00,0x00,//47
	0xE0,0xF0,0x18,0x88,0x18,0xF0,0xE0,0x00,0x07,0x0F,0x18,0x11,0x18,0x0F,0x07,0x00,//48
	0x00,0x20,0x30,0xF8,0xF8,0x00,0x00,0x00,0x00,0x10,0x10,0x1F,0x1F,0x10,0x10,0x00,//49
	0x10,0x18,0x08,0x88,0xC8,0x78,0x30,0x00,0x1C,0x1E,0x13,0x11,0x10,0x18,0x18,0x00,//50
	0x10,0x18,0x88,0x88,0x88,0xF8,0x70,0x00,0x08,0x18,0x10,0x10,0x10,0x1F,0x0F,0x00,//51
	0x80,0xC0,0x60,0x30,0xF8,0xF8,0x00,0x00,0x01,0x01,0x01,0x11,0x1F,0x1F,0x11,0x00,//52
	0xF8,0xF8,0x88,0x88,0x88,0x88,0x08,0x00,0x08,0x18,0x10,0x10,0x10,0x1F,0x0F,0x00,//53
	0xE0,0xF0,0x98,0x88,0x88,0x80,0x00,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//54
	0x18,0x18,0x08,0x08,0x88,0xF8,0x78,0x00,0x00,0x00,0x1E,0x1F,0x01,0x00,0x00,0x00,//55
	0x70,0xF8,0x88,0x88,0x88,0xF8,0x70,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//56
	0x70,0xF8,0x88,0x88,0x88,0xF8,0xF0,0x00,0x00,0x10,0x10,0x10,0x18,0x0F,0x07,0x00,//57
	0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00,0x00,0x00,//58
	0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00,0x10,0x1C,0x0C,0x00,0x00,0x00,//59
	0x00,0x00,0x80,0xC0,0x60,0x30,0x10,0x00,0x00,0x01,0x03,0x06,0x0C,0x18,0x10,0x00,//60
	0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,//61
	0x00,0x10,0x30,0x60,0xC0,0x80,0x00,0x00,0x00,0x10,0x18,0x0C,0x06,0x03,0x01,0x00,//62
	0x30,0x38,0x08,0x88,0xC8,0x78,0x30,0x00,0x00,0x00,0x00,0x1B,0x1B,0x00,0x00,0x00,//63
	0xE0,0xF0,0x10,0x90,0x90,0xF0,0xE0,0x00,0x0F,0x1F,0x10,0x17,0x17,0x17,0x03,0x00,//64
	0xC0,0xE0,0x30,0x18,0x30,0xE0,0xC0,0x00,0x1F,0x1F,0x01,0x01,0x01,0x1F,0x1F,0x00,//65	0xC0,0xE0,0x30,0x18,0x30,0xE0,0xC0,0x00,0x1F,0x1F,0x01,0x01,0x01,0x1F,0x1F,0x00,
	0x08,0xF8,0xF8,0x88,0x88,0xF8,0x70,0x00,0x10,0x1F,0x1F,0x10,0x10,0x1F,0x0F,0x00,//66 	0x08,0xF8,0xF8,0x88,0x88,0xF8,0x70,0x00,0x10,0x1F,0x1F,0x10,0x10,0x1F,0x0F,0x00,
	0xE0,0xF0,0x18,0x08,0x08,0x18,0x30,0x00,0x07,0x0F,0x18,0x10,0x10,0x18,0x0C,0x00,//67
	0x08,0xF8,0xF8,0x08,0x18,0xF0,0xE0,0x00,0x10,0x1F,0x1F,0x10,0x18,0x0F,0x07,0x00,//68
	0x08,0xF8,0xF8,0x88,0xC8,0x18,0x38,0x00,0x10,0x1F,0x1F,0x10,0x11,0x18,0x1C,0x00,//69
	0x08,0xF8,0xF8,0x88,0xC8,0x18,0x38,0x00,0x10,0x1F,0x1F,0x10,0x01,0x00,0x00,0x00,//70
	0xE0,0xF0,0x18,0x08,0x08,0x18,0x30,0x00,0x07,0x0F,0x18,0x11,0x11,0x0F,0x1F,0x00,//71
	0xF8,0xF8,0x80,0x80,0x80,0xF8,0xF8,0x00,0x1F,0x1F,0x00,0x00,0x00,0x1F,0x1F,0x00,//72
	0x00,0x00,0x08,0xF8,0xF8,0x08,0x00,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//73
	0x00,0x00,0x00,0x08,0xF8,0xF8,0x08,0x00,0x0E,0x1E,0x10,0x10,0x1F,0x0F,0x00,0x00,//74
	0x08,0xF8,0xF8,0x80,0xC0,0x78,0x38,0x00,0x10,0x1F,0x1F,0x01,0x03,0x1E,0x1C,0x00,//75
	0x08,0xF8,0xF8,0x08,0x00,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x10,0x18,0x1C,0x00,//76
	0xF8,0xF8,0x70,0xE0,0x70,0xF8,0xF8,0x00,0x1F,0x1F,0x00,0x00,0x00,0x1F,0x1F,0x00,//77
	0xF8,0xF8,0x70,0xE0,0xC0,0xF8,0xF8,0x00,0x1F,0x1F,0x00,0x00,0x01,0x1F,0x1F,0x00,//78
	0xF0,0xF8,0x08,0x08,0x08,0xF8,0xF0,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//79
	0x08,0xF8,0xF8,0x88,0x88,0xF8,0x70,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,0x00,0x00,//80
	0xF0,0xF8,0x08,0x08,0x08,0xF8,0xF0,0x00,0x0F,0x1F,0x10,0x1C,0x78,0x7F,0x4F,0x00,//81
	0x08,0xF8,0xF8,0x88,0x88,0xF8,0x70,0x00,0x10,0x1F,0x1F,0x00,0x01,0x1F,0x1E,0x00,//82
	0x30,0x78,0xC8,0x88,0x88,0x38,0x30,0x00,0x0C,0x1C,0x10,0x10,0x11,0x1F,0x0E,0x00,//83
	0x00,0x38,0x18,0xF8,0xF8,0x18,0x38,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//84
	0xF8,0xF8,0x00,0x00,0x00,0xF8,0xF8,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//85
	0xF8,0xF8,0x00,0x00,0x00,0xF8,0xF8,0x00,0x03,0x07,0x0C,0x18,0x0C,0x07,0x03,0x00,//86
	0xF8,0xF8,0x00,0x80,0x00,0xF8,0xF8,0x00,0x0F,0x1F,0x1C,0x07,0x1C,0x1F,0x0F,0x00,//87
	0x18,0x78,0xE0,0xC0,0xE0,0x78,0x18,0x00,0x18,0x1E,0x07,0x03,0x07,0x1E,0x18,0x00,//88
	0x00,0x78,0xF8,0x80,0x80,0xF8,0x78,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//89
	0x38,0x18,0x08,0x88,0xC8,0x78,0x38,0x00,0x1C,0x1E,0x13,0x11,0x10,0x18,0x1C,0x00,//90
	0x00,0x00,0xF8,0xF8,0x08,0x08,0x00,0x00,0x00,0x00,0x1F,0x1F,0x10,0x10,0x00,0x00,//91
	0x70,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x0E,0x1C,0x00,//92
	0x00,0x00,0x08,0x08,0xF8,0xF8,0x00,0x00,0x00,0x00,0x10,0x10,0x1F,0x1F,0x00,0x00,//93
	0x10,0x18,0x0C,0x06,0x0C,0x18,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//94
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,//95
	0x00,0x00,0x06,0x0E,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//96
	0x00,0x40,0x40,0x40,0xC0,0x80,0x00,0x00,0x0E,0x1F,0x11,0x11,0x0F,0x1F,0x10,0x00,//97
	0x08,0xF8,0xF8,0x40,0xC0,0x80,0x00,0x00,0x00,0x1F,0x1F,0x10,0x10,0x1F,0x0F,0x00,//98
	0x80,0xC0,0x40,0x40,0x40,0xC0,0x80,0x00,0x0F,0x1F,0x10,0x10,0x10,0x18,0x08,0x00,//99
	0x00,0x80,0xC0,0x48,0xF8,0xF8,0x00,0x00,0x0F,0x1F,0x10,0x10,0x0F,0x1F,0x10,0x00,//100
	0x80,0xC0,0x40,0x40,0x40,0xC0,0x80,0x00,0x0F,0x1F,0x11,0x11,0x11,0x19,0x09,0x00,//101
	0x80,0xF0,0xF8,0x88,0x18,0x30,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,0x00,0x00,//102
	0x80,0xC0,0x40,0x40,0x80,0xC0,0x40,0x00,0x4F,0xDF,0x90,0x90,0xFF,0x7F,0x00,0x00,//103
	0x08,0xF8,0xF8,0x80,0x40,0xC0,0x80,0x00,0x10,0x1F,0x1F,0x00,0x00,0x1F,0x1F,0x00,//104
	0x00,0x00,0x40,0xD8,0xD8,0x00,0x00,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//105
	0x00,0x00,0x00,0x00,0x40,0xD8,0xD8,0x00,0x00,0x60,0xE0,0x80,0x80,0xFF,0x7F,0x00,//106
	0x08,0xF8,0xF8,0x00,0x80,0xC0,0x40,0x00,0x10,0x1F,0x1F,0x03,0x07,0x1C,0x18,0x00,//107
	0x00,0x00,0x08,0xF8,0xF8,0x00,0x00,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//108
	0xC0,0xC0,0xC0,0x80,0xC0,0xC0,0x80,0x00,0x1F,0x1F,0x00,0x0F,0x00,0x1F,0x1F,0x00,//109
	0x40,0xC0,0x80,0x40,0x40,0xC0,0x80,0x00,0x00,0x1F,0x1F,0x00,0x00,0x1F,0x1F,0x00,//110
	0x80,0xC0,0x40,0x40,0x40,0xC0,0x80,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//111
	0x40,0xC0,0x80,0x40,0x40,0xC0,0x80,0x00,0x80,0xFF,0xFF,0x90,0x10,0x1F,0x0F,0x00,//112
	0x80,0xC0,0x40,0x40,0x80,0xC0,0x40,0x00,0x0F,0x1F,0x10,0x90,0xFF,0xFF,0x80,0x00,//113
	0x40,0xC0,0x80,0xC0,0x40,0xC0,0x80,0x00,0x10,0x1F,0x1F,0x10,0x00,0x01,0x01,0x00,//114
	0x80,0xC0,0x40,0x40,0x40,0xC0,0x80,0x00,0x08,0x19,0x13,0x12,0x16,0x1C,0x08,0x00,//115
	0x40,0x40,0xF0,0xF8,0x40,0x40,0x00,0x00,0x00,0x00,0x0F,0x1F,0x10,0x18,0x08,0x00,//116
	0xC0,0xC0,0x00,0x00,0xC0,0xC0,0x00,0x00,0x0F,0x1F,0x10,0x10,0x0F,0x1F,0x10,0x00,//117
	0x00,0xC0,0xC0,0x00,0x00,0xC0,0xC0,0x00,0x00,0x07,0x0F,0x18,0x18,0x0F,0x07,0x00,//118
	0xC0,0xC0,0x00,0x00,0x00,0xC0,0xC0,0x00,0x0F,0x1F,0x18,0x0F,0x18,0x1F,0x0F,0x00,//119
	0x40,0xC0,0x80,0x00,0x80,0xC0,0x40,0x00,0x10,0x18,0x0F,0x07,0x0F,0x18,0x10,0x00,//120
	0xC0,0xC0,0x00,0x00,0x00,0xC0,0xC0,0x00,0x8F,0x9F,0x90,0x90,0xD0,0x7F,0x3F,0x00,//121
	0xC0,0xC0,0x40,0x40,0xC0,0xC0,0x40,0x00,0x18,0x1C,0x16,0x13,0x11,0x18,0x18,0x00,//122
	0x00,0x80,0x80,0xF0,0x78,0x08,0x08,0x00,0x00,0x00,0x00,0x0F,0x1F,0x10,0x10,0x00,//123
	0x00,0x00,0x00,0x78,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x00,0x00,0x00,//124
	0x00,0x08,0x08,0x78,0xF0,0x80,0x80,0x00,0x00,0x10,0x10,0x1F,0x0F,0x00,0x00,0x00,//125
	0x10,0x18,0x08,0x18,0x10,0x18,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//126
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//127
	0x40,0xF0,0xF8,0x48,0x48,0x48,0x08,0x00,0x02,0x0F,0x1F,0x12,0x12,0x10,0x10,0x00,//128
	0x80,0xB0,0x30,0x00,0xB0,0xB0,0x00,0x00,0x0F,0x1F,0x10,0x10,0x0F,0x1F,0x10,0x00,//129
	0x00,0x80,0x80,0xA0,0x90,0x80,0x00,0x00,0x0F,0x1F,0x12,0x12,0x12,0x1B,0x0B,0x00,//130
	0x00,0x00,0xA0,0x90,0xA0,0x00,0x00,0x00,0x0E,0x1B,0x11,0x10,0x0F,0x1F,0x10,0x00,//131
	0x00,0x30,0xB0,0x80,0xB0,0x30,0x00,0x00,0x0E,0x1B,0x11,0x10,0x0F,0x1F,0x10,0x00,//132
	0x00,0x00,0x90,0xA0,0x80,0x00,0x00,0x00,0x0E,0x1B,0x11,0x10,0x0F,0x1F,0x10,0x00,//133
	0x00,0x00,0x90,0xA8,0x90,0x00,0x00,0x00,0x0E,0x1B,0x11,0x10,0x0F,0x1F,0x10,0x00,//134
	0x00,0xE0,0x30,0x10,0x10,0x30,0x00,0x00,0x00,0x03,0x16,0x14,0x0C,0x06,0x00,0x00,//135
	0x00,0x80,0xA0,0x90,0xA0,0x80,0x00,0x00,0x0F,0x1F,0x12,0x12,0x12,0x1B,0x0B,0x00,//136
	0x00,0xB0,0xB0,0x80,0xB0,0xB0,0x00,0x00,0x0F,0x1F,0x12,0x12,0x12,0x1B,0x0B,0x00,//137
	0x00,0x80,0x90,0xA0,0x80,0x80,0x00,0x00,0x0F,0x1F,0x12,0x12,0x12,0x1B,0x0B,0x00,//138
	0x00,0x30,0xB0,0x80,0xB0,0x30,0x00,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//139
	0x00,0x00,0xA0,0x90,0xA0,0x00,0x00,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//140
	0x00,0x00,0x80,0x90,0xA0,0x00,0x00,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//141
	0x18,0x98,0xC0,0x60,0xC0,0x98,0x18,0x00,0x1F,0x1F,0x04,0x04,0x04,0x1F,0x1F,0x00,//142
	0x00,0x80,0xD0,0x68,0xD0,0x80,0x00,0x00,0x1F,0x1F,0x04,0x04,0x04,0x1F,0x1F,0x00,//143
	0xC0,0xC0,0x50,0x48,0x40,0xC0,0xC0,0x00,0x1F,0x1F,0x12,0x12,0x12,0x18,0x18,0x00,//144
	0x80,0x40,0x40,0x80,0x80,0x40,0xC0,0x00,0x0C,0x12,0x12,0x0F,0x0F,0x12,0x13,0x00,//145
	0xF0,0x88,0x88,0xF0,0xF8,0x88,0x88,0x00,0x1F,0x00,0x00,0x1F,0x1F,0x10,0x10,0x00,//146
	0x00,0x80,0xA0,0x90,0xA0,0x80,0x00,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//147
	0x00,0x98,0x98,0x80,0x98,0x98,0x00,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//148
	0x00,0x80,0x90,0xA0,0x80,0x80,0x00,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//149
	0x80,0x80,0x20,0x10,0xA0,0x80,0x00,0x00,0x0F,0x1F,0x10,0x10,0x0F,0x1F,0x10,0x00,//150
	0x80,0x80,0x10,0x20,0x80,0x80,0x00,0x00,0x0F,0x1F,0x10,0x10,0x0F,0x1F,0x10,0x00,//151
	0x00,0x80,0xB0,0x30,0x00,0xB0,0xB0,0x00,0x00,0x03,0x17,0x14,0x14,0x1F,0x0F,0x00,//152
	0x80,0xD8,0x58,0x40,0x58,0xD8,0x80,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//153
	0xC0,0xD8,0x18,0x00,0x18,0xD8,0xC0,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//154
	0x80,0xC0,0x40,0xE0,0x40,0xC0,0x80,0x00,0x07,0x0C,0x08,0x1F,0x08,0x0C,0x04,0x00,//155
	0x80,0xF0,0xF8,0x88,0x18,0x30,0x00,0x00,0x18,0x0F,0x0F,0x18,0x18,0x18,0x10,0x00,//156
	0x00,0x88,0x90,0xE0,0xE0,0x90,0x88,0x00,0x00,0x04,0x04,0x1F,0x1F,0x04,0x04,0x00,//157
	0xF8,0xF8,0x48,0x48,0x48,0x48,0x30,0x00,0x1F,0x1F,0x00,0x02,0x1F,0x12,0x00,0x00,//158
	0x00,0x80,0xF0,0xF8,0x88,0x88,0x30,0x00,0x0C,0x18,0x1F,0x0F,0x00,0x00,0x00,0x00,//159
	0x00,0x00,0xA0,0x90,0x80,0x00,0x00,0x00,0x0E,0x1B,0x11,0x10,0x0F,0x1F,0x10,0x00,//160
	0x00,0x00,0x80,0xA0,0x90,0x00,0x00,0x00,0x00,0x00,0x10,0x1F,0x1F,0x10,0x00,0x00,//161
	0x00,0x80,0xA0,0x90,0x80,0x80,0x00,0x00,0x0F,0x1F,0x10,0x10,0x10,0x1F,0x0F,0x00,//162
	0x80,0x80,0x20,0x10,0x80,0x80,0x00,0x00,0x0F,0x1F,0x10,0x10,0x0F,0x1F,0x10,0x00,//163
	0x90,0x88,0x08,0x90,0xA0,0xA0,0x10,0x00,0x00,0x1F,0x1F,0x00,0x00,0x1F,0x1F,0x00,//164
	0x90,0x88,0x88,0x10,0x20,0xA0,0x90,0x00,0x1F,0x1F,0x03,0x07,0x0E,0x1F,0x1F,0x00,//165
	0x80,0xC0,0x60,0x20,0xE0,0xC0,0x00,0x00,0x13,0x16,0x14,0x14,0x13,0x17,0x14,0x00,//166
	0xE0,0xF0,0x10,0x10,0x10,0xF0,0xE0,0x00,0x13,0x17,0x14,0x14,0x14,0x17,0x13,0x00,//167
	0x00,0x00,0x80,0xE8,0x68,0x00,0x00,0x00,0x0E,0x1B,0x11,0x10,0x10,0x18,0x0C,0x00,//168
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x07,0x07,0x01,0x01,0x01,0x01,0x01,0x00,//169
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x01,0x01,0x01,0x01,0x01,0x07,0x07,0x00,//170
	0x78,0xF8,0x40,0xA0,0x90,0x88,0x00,0x00,0x01,0x00,0x11,0x18,0x14,0x12,0x11,0x00,//171
	0x78,0xF8,0x40,0x20,0x90,0x88,0x00,0x00,0x01,0x0C,0x0A,0x09,0x1F,0x1F,0x08,0x00,//172
	0x00,0x00,0x00,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x00,0x00,0x00,//173
	0x00,0x80,0xC0,0x60,0x80,0xC0,0x60,0x00,0x01,0x03,0x06,0x0D,0x03,0x06,0x0C,0x00,//174
	0x60,0xC0,0x80,0x60,0xC0,0x80,0x00,0x00,0x0C,0x06,0x03,0x0D,0x06,0x03,0x01,0x00,//175
	0x90,0x00,0x48,0x00,0x90,0x00,0x48,0x00,0x04,0x00,0x12,0x00,0x04,0x00,0x12,0x00,//176
	0x20,0x90,0x48,0x20,0x90,0x48,0x20,0x00,0x09,0x04,0x12,0x09,0x04,0x12,0x09,0x00,//177
	0xA8,0x50,0xA8,0x50,0xA8,0x50,0xA8,0x00,0x0A,0x15,0x0A,0x15,0x0A,0x15,0x0A,0x00,//178
	0x00,0x00,0x00,0xF8,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x00,0x00,0x00,//179
	0x80,0x80,0x80,0xF8,0xF8,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x1F,0x00,0x00,0x00,//180
	0x60,0x60,0x60,0xF8,0xF8,0x00,0x00,0x00,0x06,0x06,0x06,0x1F,0x1F,0x00,0x00,0x00,//181
	0x80,0x80,0xF8,0xF8,0x00,0xF8,0xF8,0x00,0x01,0x01,0x1F,0x1F,0x00,0x1F,0x1F,0x00,//182
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x01,0x01,0x1F,0x1F,0x01,0x1F,0x1F,0x00,//183
	0xC0,0xC0,0xC0,0xC0,0x00,0x00,0x00,0x00,0x06,0x06,0x1F,0x1F,0x00,0x00,0x00,0x00,//184
	0x60,0x60,0x78,0x78,0x00,0xF8,0xF8,0x00,0x06,0x06,0x1E,0x1E,0x00,0x1F,0x1F,0x00,//185
	0x00,0xF8,0xF8,0x00,0xF8,0xF8,0x00,0x00,0x00,0x1F,0x1F,0x00,0x1F,0x1F,0x00,0x00,//186
	0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0x06,0x1E,0x1E,0x00,0x1F,0x1F,0x00,0x00,//187
	0x60,0x78,0x78,0x00,0xF8,0xF8,0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,//188
	0x80,0xF8,0xF8,0x80,0xF8,0xF8,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,//189
	0x60,0x60,0xF8,0xF8,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,//190
	0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x01,0x01,0x1F,0x1F,0x00,0x00,0x00,0x00,//191
	0x00,0x00,0x00,0xF8,0xF8,0x80,0x80,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x00,//192
	0x00,0x80,0x80,0xF8,0xF8,0x80,0x80,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x00,//193
	0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x01,0x01,0x1F,0x1F,0x01,0x01,0x00,//194
	0x00,0x00,0xF8,0xF8,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x1F,0x01,0x01,0x01,0x00,//195
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,//196
	0x80,0x80,0xF8,0xF8,0x80,0x80,0x00,0x00,0x01,0x01,0x1F,0x1F,0x01,0x01,0x00,0x00,//197
	0x00,0x00,0xF8,0xF8,0x60,0x60,0x60,0x00,0x00,0x00,0x1F,0x1F,0x06,0x06,0x06,0x00,//198
	0x00,0xF8,0xF8,0x00,0xF8,0xF8,0x80,0x00,0x00,0x1F,0x1F,0x00,0x1F,0x1F,0x01,0x00,//199
	0x00,0xF8,0xF8,0x00,0xF8,0xF8,0xC0,0x00,0x00,0x07,0x07,0x06,0x06,0x06,0x06,0x00,//200
	0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0x1F,0x1F,0x00,0x1E,0x1E,0x06,0x00,//201
	0x60,0x78,0x78,0x00,0x78,0x78,0x60,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,//202
	0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x06,0x1E,0x1E,0x00,0x1E,0x1E,0x06,0x00,//203
	0x00,0xF8,0xF8,0x00,0x78,0x78,0x60,0x00,0x00,0x1F,0x1F,0x00,0x1E,0x1E,0x06,0x00,//204
	0x00,0x60,0x60,0x60,0x60,0x60,0x60,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x06,0x00,//205
	0x60,0x78,0x78,0x00,0x78,0x78,0x60,0x00,0x06,0x1E,0x1E,0x00,0x1E,0x1E,0x06,0x00,//206
	0x00,0x60,0x60,0x78,0x78,0x60,0x60,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x06,0x00,//207
	0x60,0x78,0x78,0x60,0x78,0x78,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//208
	0x00,0x60,0x60,0x60,0x60,0x60,0x60,0x00,0x00,0x06,0x06,0x1E,0x1E,0x06,0x06,0x00,//209
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x1E,0x1E,0x06,0x1E,0x1E,0x06,0x00,//210
	0x00,0xF8,0xF8,0xC0,0xF8,0xF8,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//211
	0x00,0x00,0x00,0xF8,0xF8,0x60,0x60,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x00,//212
	0x40,0xF0,0xF8,0x48,0x48,0x48,0x08,0x00,0x02,0x0F,0x1F,0x12,0x12,0x10,0x10,0x00,//213
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x03,0x1F,0x1F,0x03,0x00,//214
	0x80,0xF8,0xF8,0x80,0xF8,0xF8,0x80,0x00,0x01,0x1F,0x1F,0x01,0x1F,0x1F,0x01,0x00,//215
	0x60,0x60,0xF8,0xF8,0x60,0x60,0x00,0x00,0x06,0x06,0x1F,0x1F,0x06,0x06,0x00,0x00,//216
	0x80,0x80,0xF8,0xF8,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,//217
	0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x1F,0x1F,0x01,0x01,0x00,//218
	0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00,//219
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00,//220
	0xF8,0xF8,0xF8,0xF8,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x00,0x00,0x00,0x00,//221
	0x00,0x00,0x00,0xF8,0xF8,0xF8,0xF8,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x00,//222
	0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//223
	0xE0,0x30,0x10,0x30,0xE0,0xC0,0x70,0x00,0x07,0x0C,0x08,0x0C,0x07,0x03,0x0E,0x00,//224
	0x00,0xE0,0xF0,0x48,0x48,0xF8,0xB0,0x00,0x10,0x1F,0x0F,0x0C,0x04,0x07,0x03,0x00,//225
	0x18,0xF8,0xF8,0x18,0x18,0x18,0x78,0x00,0x18,0x1F,0x1F,0x18,0x00,0x00,0x00,0x00,//226
	0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x10,0x0F,0x07,0x00,0x0F,0x1F,0x18,0x00,//227
	0x30,0x70,0xD0,0x90,0x10,0x10,0x30,0x00,0x18,0x1C,0x16,0x13,0x11,0x10,0x18,0x00,//228
	0x00,0x00,0x80,0xC0,0xC0,0x40,0x40,0x00,0x0E,0x1B,0x11,0x13,0x1E,0x0C,0x00,0x00,//229
	0x00,0xE0,0xE0,0x00,0x00,0xE0,0xE0,0x00,0x18,0x1F,0x07,0x06,0x06,0x07,0x0F,0x00,//230
	0x60,0x60,0xE0,0xE0,0x60,0x60,0x60,0x00,0x00,0x00,0x07,0x0F,0x18,0x18,0x08,0x00,//231
	0xC0,0x60,0x20,0xF8,0x20,0x60,0xC0,0x00,0x03,0x06,0x04,0x1F,0x04,0x06,0x03,0x00,//232
	0xC0,0xF0,0x98,0x88,0x98,0xF0,0xC0,0x00,0x00,0x0F,0x19,0x11,0x19,0x0F,0x00,0x00,//233
	0xE0,0x30,0x10,0x10,0x10,0x30,0xE0,0x00,0x09,0x0A,0x0C,0x00,0x0C,0x0A,0x09,0x00,//234
	0x38,0x68,0xC8,0x88,0x18,0x30,0x20,0x00,0x0E,0x1B,0x11,0x11,0x1B,0x0E,0x00,0x00,//235
	0x80,0x40,0x40,0x80,0x40,0x40,0x80,0x00,0x00,0x01,0x01,0x00,0x01,0x01,0x00,0x00,//236
	0xE0,0x90,0x00,0xF0,0x38,0x80,0xE0,0x00,0x00,0x11,0x1D,0x07,0x01,0x01,0x00,0x00,//237
	0x00,0x80,0xC0,0x60,0x20,0x20,0x00,0x00,0x07,0x0F,0x1A,0x12,0x12,0x12,0x00,0x00,//238
	0xE0,0xE0,0x18,0x18,0x18,0xE0,0xE0,0x00,0x1F,0x1F,0x00,0x00,0x00,0x1F,0x1F,0x00,//239
	0xB0,0xB0,0xB0,0xB0,0xB0,0xB0,0xB0,0x00,0x0D,0x0D,0x0D,0x0D,0x0D,0x0D,0x0D,0x00,//240
	0xC0,0xC0,0xF0,0xF0,0xC0,0xC0,0x00,0x00,0x18,0x18,0x1B,0x1B,0x18,0x18,0x00,0x00,//241
	0x00,0x08,0x18,0xB0,0xE0,0x40,0x00,0x00,0x00,0x1A,0x1B,0x19,0x18,0x18,0x00,0x00,//242
	0x00,0x40,0xE0,0xB0,0x18,0x08,0x00,0x00,0x00,0x18,0x18,0x19,0x1B,0x1A,0x00,0x00,//243
	0x00,0xE0,0xF0,0x18,0x70,0x60,0x00,0x00,0x00,0x1F,0x1F,0x00,0x00,0x00,0x00,0x00,//244
	0x00,0x00,0x00,0x00,0xF8,0xF8,0x00,0x00,0x00,0x06,0x0E,0x18,0x0F,0x07,0x00,0x00,//245
	0x80,0x80,0xB0,0xB0,0x80,0x80,0x00,0x00,0x01,0x01,0x0D,0x0D,0x01,0x01,0x00,0x00,//246
	0xC0,0x60,0x30,0x60,0xC0,0x60,0x30,0x00,0x0C,0x06,0x03,0x06,0x0C,0x06,0x03,0x00,//247
	0x70,0xF8,0x88,0x88,0x88,0xF8,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//248
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x1C,0x00,0x00,0x00,0x00,//249
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,//250
	0x00,0x00,0xF8,0xF8,0x18,0x18,0x18,0x00,0x08,0x10,0x1F,0x0F,0x00,0x00,0x00,0x00,//251
	0x70,0xE0,0x30,0x18,0x18,0xF0,0xE0,0x00,0x00,0x00,0x10,0x10,0x18,0x0F,0x07,0x00,//252
	0x90,0xC8,0xA8,0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//253
	0x00,0x00,0x00,0x00,0xC0,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F,0x00,//254
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 //255

};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/******************************************************************************/
/**
  * @brief  
  * @param  None
  * @retval None
  */
void OLED_Driver_Init(void)
{
    OLED_WriteCmd(SSD1305_DISPLAYOFF);

    OLED_WriteCmd(SSD1305_SEGREMAP | 0x01);    //segment remap

    OLED_WriteCmd(SSD1305_SETCOMPINS);    //common pads hardware: alternative
    OLED_WriteCmd(0x12);

    OLED_WriteCmd(SSD1305_COMSCANDEC);    //common output scan direction:com63~com0

    OLED_WriteCmd(SSD1305_SETMULTIPLEX);    //multiplex ration mode:63
    OLED_WriteCmd(0x3F); 		// 1/64

    OLED_WriteCmd(SSD1305_SETDISPLAYCLOCKDIV);    //display divide ratio/osc. freq. mode
    OLED_WriteCmd(0xA0);    //Osc. Freq:320kHz,DivideRation:1, default value=0x70

    OLED_WriteCmd(SSD1305_SETCONTRAST);    //contrast control
    OLED_WriteCmd(0x70);    // mode:64

    OLED_WriteCmd(SSD1305_SETPRECHARGE);	  //set pre-charge period
    OLED_WriteCmd(0x22);	  //set period 1:1;period 2:15

    OLED_WriteCmd(SSD1305_MEMORYMODE);    //Set Memory Addressing Mode
    OLED_WriteCmd(0x02);    //page addressing mode

    OLED_WriteCmd(SSD1305_SETVCOMLEVEL);    //VCOM deselect level mode
    OLED_WriteCmd(0x3C);	  //set Vvcomh=0.83*Vcc

		OLED_WriteCmd(SSD1305_MASTERCONFIG);    //master configuration
    OLED_WriteCmd(0x8E);    //external VCC supply

		OLED_WriteCmd(SSD1305_DISPLAYALLON_RESUME);    //out follows RAM content
    OLED_WriteCmd(SSD1305_NORMALDISPLAY);    //set normal display
}


/******************************************************************************/
/**
  * @brief  
  * @param  None
  * @retval None
  */
void OLED_Clear_Screen(void)
{
    u8 i, j;

		OLED_WriteCmd(SSD1305_SETSTARTLINE);

		for(i = 0; i < 8; i++)		// LED height is 64
    {
        OLED_WriteCmd(SSD1305_SETPAGESTART + i);
        OLED_WriteCmd(SSD1305_SETLOWCOLUMN);
				OLED_WriteCmd(SSD1305_SETHIGHCOLUMN);

				for(j = 0; j < 128; j++)	// LED width is 128
            OLED_WriteData(0x00);
    }
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void OLED_Full_Screen(void)
{
    u8 i, j;

		OLED_WriteCmd(SSD1305_SETSTARTLINE);

		for(i = 0; i < 8; i++)		// LED height is 64
    {
        OLED_WriteCmd(SSD1305_SETPAGESTART + i);
        OLED_WriteCmd(SSD1305_SETLOWCOLUMN);
				OLED_WriteCmd(SSD1305_SETHIGHCOLUMN);

				for(j = 0; j < 128; j++)	// LED width is 128
            OLED_WriteData(0xFF);
    }
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void OLED_Screen_Display(u8 bIsDisplayOn)
{
		if(bIsDisplayOn == OLED_ON)
				OLED_WriteCmd(SSD1305_DISPLAYON);
		else
				OLED_WriteCmd(SSD1305_DISPLAYOFF);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void OLED_Write_Battery(uint8_t u8Char)
{
	// switch to page 0
  OLED_WriteCmd(0xb0);

  // set cursor to current cursor, see document Table 9-3
  OLED_WriteCmd(m_u8cursor & 0x0f);
  OLED_WriteCmd((m_u8cursor >> 4) | 0x10);

  // draw character
  vDrawBattery(u8Char, 0);

  // switch to page 1
  OLED_WriteCmd(0xb1);

  // set cursor to current cursor, see document Table 9-3
  OLED_WriteCmd(m_u8cursor & 0x0f);
  OLED_WriteCmd((m_u8cursor >> 4) | 0x10);

  // draw character 
  vDrawBattery(u8Char, 1);

  m_u8cursor = (m_u8cursor + 8) % 128;
}


/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval None
  */
void vSetCursor(uint8_t u8Value)
{
		if(u8Value > 128) return;

		m_u8cursor = u8Value;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
// draw character from m_ASCII_Table, Just draw half.
// HalfMode = 0 is up half , HalfMode = 1 is down halp
static void vDrawCharacter_8_16(uint8_t u8Char,uint8_t HalfMode)
{
	uint8_t  u8Inx = 0;
	uint16_t u16TableInx = u8Char * 16;
	uint32_t au32SourceData = 0;

	if(HalfMode > 1) return; 

	for(u8Inx = 0; u8Inx < 8; u8Inx ++)
	{
		if(HalfMode == 0)
		{
			au32SourceData =  m_ASCII_Table[u16TableInx + u8Inx];
		}
		else if (HalfMode == 1)
		{
			au32SourceData =  m_ASCII_Table[u16TableInx + 8 + u8Inx];
		}

    if (bOledInvertor)
        au32SourceData = ~au32SourceData;

    OLED_WriteData(au32SourceData);
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void vWriteLCDData_8_16(uint8_t u8Char, uint8_t u8Offset)
{
	// switch to page 0
  OLED_WriteCmd(0xb0 | (u8Offset * 2));

  // set cursor to current cursor, see document Table 9-3
  OLED_WriteCmd(m_u8cursor & 0x0f);
  OLED_WriteCmd((m_u8cursor >> 4) | 0x10);

  // draw character
  vDrawCharacter_8_16(u8Char, 0);

  // switch to page 1
  OLED_WriteCmd(0xb1 | (u8Offset * 2));

  // set cursor to current cursor, see document Table 9-3
  OLED_WriteCmd(m_u8cursor & 0x0f);
  OLED_WriteCmd((m_u8cursor >> 4) | 0x10);

  // draw character 
  vDrawCharacter_8_16(u8Char, 1);

  m_u8cursor = (m_u8cursor + 8) % 128;
}


/**
  * @brief  
  * @param  
  * @retval None
  */
// Max Length of string is 16
void vPrintText(uint8_t u8Line,char *str,...)
{
	uint8_t u8Len;
	char word[48] = {0};

	va_list argptr;
	va_start(argptr, str);
	vsprintf(word, str, argptr);
	va_end(argptr);

	// Set cursor to start	
	vSetCursor(0);

	// Show character
	for(u8Len = 0; u8Len < 16; u8Len += 1)
	{
		if(word[u8Len] == '\0' || word[u8Len] == 0) 
			break;

		if(u8Line == 1)
		{
			vWriteLCDData_8_16(word[u8Len], 0);			
		}	
		else if(u8Line == 2)
		{
			vWriteLCDData_8_16(word[u8Len], 1);					
		}
		else if(u8Line == 3)
		{
			vWriteLCDData_8_16(word[u8Len], 2);					
		}
		else if(u8Line == 4)
		{
			vWriteLCDData_8_16(word[u8Len], 3);
		}     
	}

	// Show Left of space to ' '
	for(; u8Len < 16; u8Len += 1)
	{
		word[u8Len] = ' ';

		if(u8Line == 1)
		{
			vWriteLCDData_8_16(word[u8Len], 0);			
		}
		else if(u8Line == 2)
		{
			vWriteLCDData_8_16(word[u8Len], 1);					
		}
		else if(u8Line == 3)
		{
			vWriteLCDData_8_16(word[u8Len], 2);					
		}
		else if(u8Line == 4)
		{
			vWriteLCDData_8_16(word[u8Len], 3);					
		}       
	}
}

void vDrawBattery(uint8_t u8Char,uint8_t HalfMode)
{
	uint8_t  u8Inx = 0;
	uint16_t u16TableInx = u8Char * 16;
	uint8_t au32SourceData = 0;

	if(HalfMode > 1) return;

	for(u8Inx = 0; u8Inx < 8; u8Inx ++)
	{
		if(HalfMode == 0)
		{
      au32SourceData = _BATTERY_1610[u16TableInx + u8Inx];
		}
		else if(HalfMode == 1)
		{
      au32SourceData = _BATTERY_1610[u16TableInx + 8 + u8Inx];
		}

    if(bOledInvertor)
      au32SourceData = ~au32SourceData;    

    OLED_WriteData(au32SourceData);
	}
}



