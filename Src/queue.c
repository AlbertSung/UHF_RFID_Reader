/**
  ******************************************************************************
  * @file    queue.c
  * @author  Albert
  * @version V1.0.0
  * @date    6-July-2017
  * @brief   queue buffer sub-routine
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "queue.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
queueBuf_t m_bUartRxQueue1, m_bUartRxQueue2;
queueBuf_t m_bUsbRxQueue;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/******************************************************************************/

/**
  * @brief  
  * @param  
  * @retval None
  */
void QueueInit(queueBuf_t* ptQueue)
{
		memset(ptQueue->bBuffer, 0x0, UART_QUEUE_SIZE);
		ptQueue->bPutInx = 0;
		ptQueue->bGetInx = 0;
		ptQueue->bLength = 0;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void QueueSkip(queueBuf_t* ptQueue)
{
		ptQueue->bGetInx = ptQueue->bPutInx;
		ptQueue->bLength = 0;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u8 QueuePut(queueBuf_t* ptQueue, u8 bData)
{
		if(ptQueue->bLength >= UART_QUEUE_SIZE)
				return FALSE;

		ptQueue->bBuffer[ptQueue->bPutInx] = bData;

		if(++(ptQueue->bPutInx) == UART_QUEUE_SIZE)
				ptQueue->bPutInx = 0;

		ptQueue->bLength++;

		return TRUE;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u8 QueueGet(queueBuf_t* ptQueue, u8* bData)
{
		if(ptQueue->bLength == 0)
				return FALSE;

		*bData = ptQueue->bBuffer[ptQueue->bGetInx];

		if(++(ptQueue->bGetInx) == UART_QUEUE_SIZE)
				ptQueue->bGetInx = 0;

		ptQueue->bLength--;

		return TRUE;
}


