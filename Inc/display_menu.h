/**
  ******************************************************************************
  * @file    display_menu.h
  * @author  Albert
  * @version V1.0.0
  * @date    23-Jan-2018
  * @brief   Header for display_menu.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DISPLAY_MENU_H
#define __DISPLAY_MENU_H

#ifdef __cplusplus
 extern "C" {
#endif


// Compiler options
#define NEW_AUR700_CODES_MENU

	 
/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"
#include "oled_driver.h"


/* Defined constants ---------------------------------------------------------*/
#define MENU_MAX_DATA_SIZE					16

#define OLED_MESSAGE_TIME_MS				2000
#define OLED_WAITNEXT_TIME_MS				1000
#define OLED_SUBMENU_TIME_MS				15000
#define LEDS_WARNING_TIME_MS				2000
#define LEDS_SUCCESS_TIME_MS				1500
#define VIBR_SUCCESS_TIME_MS				50
#define BUZZ_SUCCESS_TIME_MS				1500
#define BUZZ_WARNING_TIME_MS				900

#define MAX_BARCODE_NUM							10
#define MAX_LOCATION_NUM						7

#define TASK_MAX_LIST_NUM						8			// including the last task "B2E_TASK_NONE"


/* Exported definitions ------------------------------------------------------*/
// Menu Event Set
typedef enum
{
		EVENT_MENU_NONE = 0x00,

		EVENT_MENU_UPDATE = 0x01,
		EVENT_MENU_CHECK = 0x02,

		EVENT_KEY_LEFT = 0x11,
		EVENT_KEY_RIGHT = 0x12,
		EVENT_KEY_TRIG = 0x13,

		EVENT_LPTASK_DONE = 0x21,
		EVENT_LPTASK_ERROR = 0x22,
		EVENT_TIMEOUT_BACK = 0x23,

		EVENT_CHARGER_CHANGE = 0x31,

} Menu_EvtSet;

// Main Menu Set
typedef enum
{
		MAIN_READ_LOGSTATUS = 0x00,
		MAIN_START_LOGGING = 0x01,
		MAIN_STOP_LOGGING = 0x02,
		MAIN_WRITE_LOCATION = 0x03,
		MAIN_READ_DEVICEINFO = 0x04,
		MAIN_BARCODE_LOGGING = 0x05,

		MAIN_DEBUG_TEST = 0x0F,		// only for debugging and testing. Maximum is 15 based on the size of bMainIdx

} Main_MenuSet;

// Sub Menu Set
typedef enum
{
		SUB_PAGE_MAIN = 0x01,		// 0x00 is for page's calculation while cycle switch
		SUB_PAGE_1 = 0x02,
		SUB_PAGE_2 = 0x03,
		SUB_PAGE_3 = 0x04,
		SUB_PAGE_4 = 0x05,
		SUB_PAGE_5 = 0x06,
		SUB_PAGE_6 = 0x07,
		SUB_PAGE_7 = 0x08,
		SUB_PAGE_8 = 0x09,
		SUB_PAGE_9 = 0x0A,
		SUB_PAGE_10 = 0x0B,
		SUB_PAGE_11 = 0x0C,
		SUB_PAGE_12 = 0x0D,
		SUB_PAGE_13 = 0x0E,
		SUB_PAGE_14 = 0x0F,		// Maximum is 15 based on the size of bSubIdx

} Sub_MenuSet;


/* Exported types ------------------------------------------------------------*/
typedef struct
{
		u8 bSubIdx:4;	                                          // Sub menu index
		u8 bMainIdx:4;                                          // Main menu index
    u8 bEvent; 	                                            // Event to change menu display
		u8 bData[MENU_MAX_DATA_SIZE];														// Data to display message
} MenuFormat_t;

typedef struct
{
		u8 bTaskIdx;
		u8 bTaskBuf[TASK_MAX_LIST_NUM];
} TaskList_t;


/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern MenuFormat_t display_menu;
extern TaskList_t task_list;


/* Exported functions --------------------------------------------------------*/
extern void MenuUpdate(u8 bMenuItem);
extern void MenuDisplay(void);
extern void CheckBatStatus(void);



#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_MENU_H */
