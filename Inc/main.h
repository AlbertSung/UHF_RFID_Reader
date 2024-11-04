/*************************************************************************************
 *
 *
 *************************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

// Compiler options
#define ORG_SYS_CLK

//#define NEW_AUR700_CODES_LPRUNMODE
//#define NEW_AUR700_CODES_STOPMODE
#define NEW_AUR700_CODES_LPSLPMODE

#define NEW_AUR700_CODES_ONOFF

#define NEW_AUR700_CODES_IDLE

#define NEW_AUR700_CODES_ADC

#define NEW_AUR700_CODES_BLE

//#define NEW_AUR700_CODES_KEYS


/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "gen2_config.h"
#include "queue.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h" 
#include "usbd_cdc_interface.h"

#include "cmd_protocol_gnet.h"
#include "display_menu.h"
#include "barcode_reader.h"
#include "sl900a_logger.h"

/* Defined constants ---------------------------------------------------------*/
// Command Source Set
typedef enum
{
		CMD_SOURCE_NONE = 0,

		CMD_SOURCE_UART = 1,
		CMD_SOURCE_USB = 2,
		CMD_SOURCE_KEY = 3,

} Cmd_SourceSet;

// Command Source Set
typedef enum
{
		LOOP_DISABLE = 0,

		LOOP_EN_FAIL = 1,
		LOOP_EN_DONE = 2,
		LOOP_MAIN_CHECK = 3,
		LOOP_SUB_CHECK = 4,

} Loop_StatusSet;

// Idle Event Set
typedef enum
{
		IDLE_NORMAL_RUN = 0,

		IDLE_OLED_OFF = 1,

} Idle_EvtSet;


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define NVIC_VECTTAB_FLASH      				(0x8000000)
#define VECTTAB_OFFSET           				0x4000		

#define APP_START_ADDRESS								(0x8004000)

#define ADC_REC_TIME_MS									5000
#define IDLE_PWOFF_TIME_MS							(6 * 60000)


/* User can use this section to tailor SPIx instance used and associated
   resources */
/* Definition for SPIx clock resources */		// modified for AUR700
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_13
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF5_SPI2
#define SPIx_MISO_PIN                    GPIO_PIN_14
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MISO_AF                     GPIO_AF5_SPI2
#define SPIx_MOSI_PIN                    GPIO_PIN_15
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_MOSI_AF                     GPIO_AF5_SPI2

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);

/* Exported variables ------------------------------------------------------- */
extern uint8_t bRxGetBuf1, bRxGetBuf2;
extern uint8_t bRxPutBuf1, bRxPutBuf2;
extern u8 bLoopTask, bLoopMenu, bLoopLEDS, bLoopVIBR, bLoopBUZZ;
extern u32 ulLoop_Tick, ulLoop_Timeout, ulADC_Tick;
extern u32 ulMenu_Tick, ulMenu_Timeout, ulCheck_Tick, ulCheck_Timeout, ulEvent_Tick, ulLEDS_Timeout, ulVIBR_Timeout, ulBUZZ_Timeout;
extern u8 bReadBank;
extern u16 uReadWdAddr, uReadWdLen;
extern u8 bSelectBank, bWriteBank, bSelMask[], bWrData[];
extern u16 uSelWdAddr, uWriteWdAddr, uSelWdLen, uWriteWdLen;
extern u8 g_CmdSource;
extern u16 uTblLogNum;
extern u8 bIdleBreak, bIdleStatus;
extern u8 bBLEConnected;

extern gen2QueryParams_t g_tQueryParams;
extern u8 bQueryTarget;
extern u32 ulTagPassword;

extern u32 ulIdle_Tick, ulIdle_Timeout;


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
