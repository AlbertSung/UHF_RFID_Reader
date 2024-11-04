/**
  ******************************************************************************
  * @file    barcode_reader.h
  * @author  Albert
  * @version V1.0.0
  * @date    24-Jan-2018
  * @brief   Header for barcode_reader.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BARCODE_READER_H
#define __BARCODE_READER_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"

/* Defined constants ---------------------------------------------------------*/
#define BARCODE_RESP_ACK 0x06
#define BARCODE_RESP_ENQ 0x05
#define BARCODE_RESP_NAK 0x15

/* Exported definitions ------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define BARC_WAIT_RESPONSE_TIME		1000
#define BARC_TRIGGER_TIMEOUT			20000

/* Exported variables --------------------------------------------------------*/
extern u8 bBarcLastStr[];
extern u8 g_bBarcTempLen;

/* Exported functions --------------------------------------------------------*/
s8 InitBarcReader(void);
void WakeupBarcoder(void);
void TriggerBarcoder(void);
void UntriggerBarcoder(void);
s8 ReadBarcode(void);

/* Exported macro ------------------------------------------------------------*/
#define BARCODE_MENU_PREFIX(CMD_BUFFER, START_IDX)						\
													 (CMD_BUFFER)[(START_IDX)  ] = 0x16;\
													 (CMD_BUFFER)[(START_IDX)+1] = 'M';	\
													 (CMD_BUFFER)[(START_IDX)+2] = 0x0D;\
													 (START_IDX) =(START_IDX)+3;

#define BARCODE_MENU_COMMA(CMD_BUFFER, START_IDX) 					\
													(CMD_BUFFER)[(START_IDX)  ] = ',';\
													(START_IDX) =(START_IDX)+1;

#define BARCODE_MENU_SEMICOLON(CMD_BUFFER, START_IDX) 					\
													    (CMD_BUFFER)[(START_IDX)  ] = ';';\
													    (START_IDX) =(START_IDX)+1;

#define BARCODE_STORAGE_VOLATILE(CMD_BUFFER, START_IDX) 					\
																(CMD_BUFFER)[(START_IDX)  ] = '!';\
																(START_IDX) =(START_IDX)+1;

#define BARCODE_STORAGE_NONVOLATILE(CMD_BUFFER, START_IDX) 					 \
																   (CMD_BUFFER)[(START_IDX)  ] = '.';\
																   (START_IDX) =(START_IDX)+1;

#define BARCODE_TAG_232(CMD_BUFFER, START_IDX) 					 \
											 (CMD_BUFFER)[(START_IDX)  ] = '2';\
											 (CMD_BUFFER)[(START_IDX)+1] = '3';\
											 (CMD_BUFFER)[(START_IDX)+2] = '2';\
											 (START_IDX) =(START_IDX)+3;

#define BARCODE_TAG_PWR(CMD_BUFFER, START_IDX) 					 \
											 (CMD_BUFFER)[(START_IDX)  ] = 'P';\
											 (CMD_BUFFER)[(START_IDX)+1] = 'W';\
											 (CMD_BUFFER)[(START_IDX)+2] = 'R';\
											 (START_IDX) =(START_IDX)+3;

#define BARCODE_PWRSAVE_OFF(CMD_BUFFER, START_IDX) 					 \
													 (CMD_BUFFER)[(START_IDX)  ] = 'M';\
													 (CMD_BUFFER)[(START_IDX)+1] = 'O';\
													 (CMD_BUFFER)[(START_IDX)+2] = 'D';\
													 (CMD_BUFFER)[(START_IDX)+3] = '0';\
													 (START_IDX) =(START_IDX)+4;

#define BARCODE_PWRSAVE_SLEEP(CMD_BUFFER, START_IDX) 					 \
													   (CMD_BUFFER)[(START_IDX)  ] = 'M';\
													   (CMD_BUFFER)[(START_IDX)+1] = 'O';\
													   (CMD_BUFFER)[(START_IDX)+2] = 'D';\
													   (CMD_BUFFER)[(START_IDX)+3] = '1';\
													   (START_IDX) =(START_IDX)+4;

#define BARCODE_PWRSAVE_HIBERNATE(CMD_BUFFER, START_IDX) 					 \
																 (CMD_BUFFER)[(START_IDX)  ] = 'M';\
																 (CMD_BUFFER)[(START_IDX)+1] = 'O';\
																 (CMD_BUFFER)[(START_IDX)+2] = 'D';\
																 (CMD_BUFFER)[(START_IDX)+3] = '2';\
																 (START_IDX) =(START_IDX)+4;

#define BARCODE_PWRSAVE_TIMEOUT(CMD_BUFFER, START_IDX) 					 \
													     (CMD_BUFFER)[(START_IDX)  ] = 'L';\
															 (CMD_BUFFER)[(START_IDX)+1] = 'P';\
													     (CMD_BUFFER)[(START_IDX)+2] = 'T';\
													     (START_IDX) =(START_IDX)+3;

#define BARCODE_TAG_TRG(CMD_BUFFER, START_IDX) 					 \
											 (CMD_BUFFER)[(START_IDX)  ] = 'T';\
											 (CMD_BUFFER)[(START_IDX)+1] = 'R';\
											 (CMD_BUFFER)[(START_IDX)+2] = 'G';\
											 (START_IDX) =(START_IDX)+3;

#define BARCODE_TRIGGER_TIMEOUT(CMD_BUFFER, START_IDX) 					 \
													     (CMD_BUFFER)[(START_IDX)  ] = 'S';\
															 (CMD_BUFFER)[(START_IDX)+1] = 'T';\
													     (CMD_BUFFER)[(START_IDX)+2] = 'O';\
													     (START_IDX) =(START_IDX)+3;

#define BARCODE_TRIGGER_ACTIVATE(CMD_BUFFER, START_IDX)	 					 \
																(CMD_BUFFER)[(START_IDX)  ] = 0x16;\
																(CMD_BUFFER)[(START_IDX)+1] = 'T'; \
																(CMD_BUFFER)[(START_IDX)+2] = 0x0D;\
																(START_IDX) =(START_IDX)+3;

#define BARCODE_TRIGGER_DEACTIVATE(CMD_BUFFER, START_IDX)	 					 \
																	(CMD_BUFFER)[(START_IDX)  ] = 0x16;\
																	(CMD_BUFFER)[(START_IDX)+1] = 'U'; \
																	(CMD_BUFFER)[(START_IDX)+2] = 0x0D;\
																	(START_IDX) =(START_IDX)+3;

#define BARCODE_CHAR_ASCII(CMD_BUFFER, START_IDX, ASCII_CHAR) 					\
													(CMD_BUFFER)[(START_IDX)  ] = (ASCII_CHAR); 	\
													(START_IDX) =(START_IDX)+1;



#ifdef __cplusplus
}
#endif

#endif /* __BARCODE_READER_H */
