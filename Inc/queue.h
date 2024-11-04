/**
  ******************************************************************************
  * @file    queue.h
  * @author  Albert
  * @version V1.0.0
  * @date    6-July-2017
  * @brief   Header for queue.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QUEUE_H
#define __QUEUE_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"

/* Defined constants ---------------------------------------------------------*/
#define UART_QUEUE_SIZE			64

/* Exported types ------------------------------------------------------------*/
typedef struct
{
		u8 bBuffer[UART_QUEUE_SIZE];
		u8 bPutInx;
		u8 bGetInx;
		u8 bLength;
} queueBuf_t;




/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern queueBuf_t m_bUartRxQueue1, m_bUartRxQueue2;
extern queueBuf_t m_bUsbRxQueue;

/* Exported functions --------------------------------------------------------*/
void QueueInit(queueBuf_t* ptQueue);
u8 QueuePut(queueBuf_t* ptQueue, u8 bData);
u8 QueueGet(queueBuf_t* ptQueue, u8* bData);



#ifdef __cplusplus
}
#endif

#endif /* __QUEUE_H */
