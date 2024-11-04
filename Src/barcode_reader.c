/**
  ******************************************************************************
  * @file    barcode_reader.c
  * @author  Albert
  * @version V1.0.0
  * @date    24-Jan-2018
  * @brief   Barcode reader sub-routine
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "barcode_reader.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8 bBarcTempStr[20], bBarcLastStr[20];		// TL10 only can save 18 bytes, and need one more byte for "\0" to be a string ending
u8 g_bBarcTempLen = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 InitBarcReader(void)
{
		s8 cResult = ERR_BARC_OK;
		u8 sCmd_Str[30], bStr_Len;

		if(cResult == ERR_BARC_OK)
		{
				bStr_Len = 0;
				BARCODE_MENU_PREFIX(sCmd_Str, bStr_Len);
				BARCODE_TAG_PWR(sCmd_Str, bStr_Len);
				BARCODE_PWRSAVE_HIBERNATE(sCmd_Str, bStr_Len);
				BARCODE_STORAGE_NONVOLATILE(sCmd_Str, bStr_Len);
				while(HAL_BUSY == HAL_UART_Transmit_IT(&Uart1Handle, sCmd_Str, bStr_Len));

				ulLoop_Tick = HAL_GetTick();
				while(!IsTimeout_ms(ulLoop_Tick, BARC_WAIT_RESPONSE_TIME))
				{
						if(QueueGet(&m_bUartRxQueue1, &bRxGetBuf1) == TRUE)
						{
								if(bRxGetBuf1 == '.')
								{
										break;
								}
						}
				}
		}

		if(cResult == ERR_BARC_OK)
		{
				bStr_Len = 0;
				BARCODE_MENU_PREFIX(sCmd_Str, bStr_Len);
				BARCODE_TAG_PWR(sCmd_Str, bStr_Len);
				BARCODE_PWRSAVE_TIMEOUT(sCmd_Str, bStr_Len);
				BARCODE_CHAR_ASCII(sCmd_Str, bStr_Len, '3');		// timeout is set to 30 seconds
				BARCODE_CHAR_ASCII(sCmd_Str, bStr_Len, '0');
				BARCODE_STORAGE_NONVOLATILE(sCmd_Str, bStr_Len);
				while(HAL_BUSY == HAL_UART_Transmit_IT(&Uart1Handle, sCmd_Str, bStr_Len));
				
				ulLoop_Tick = HAL_GetTick();
				while(!IsTimeout_ms(ulLoop_Tick, BARC_WAIT_RESPONSE_TIME))
				{
						if(QueueGet(&m_bUartRxQueue1, &bRxGetBuf1) == TRUE)
						{
								if(bRxGetBuf1 == '.')
								{
										break;
								}
						}
				}
		}

#ifdef SET_TIMEOUT_TRIGGER
		if(cResult == ERR_BARC_OK)
		{
				bStr_Len = 0;
				BARCODE_MENU_PREFIX(sCmd_Str, bStr_Len);
				BARCODE_TAG_TRG(sCmd_Str, bStr_Len);
				BARCODE_TRIGGER_TIMEOUT(sCmd_Str, bStr_Len);
				BARCODE_CHAR_ASCII(sCmd_Str, bStr_Len, '2');
				BARCODE_CHAR_ASCII(sCmd_Str, bStr_Len, '0');
				BARCODE_CHAR_ASCII(sCmd_Str, bStr_Len, '0');
				BARCODE_CHAR_ASCII(sCmd_Str, bStr_Len, '0');
				BARCODE_CHAR_ASCII(sCmd_Str, bStr_Len, '0');
				BARCODE_STORAGE_NONVOLATILE(sCmd_Str, bStr_Len);
				while(HAL_BUSY == HAL_UART_Transmit_IT(&Uart1Handle, sCmd_Str, bStr_Len));
				
				ulLoop_Tick = HAL_GetTick();
				while(!IsTimeout_ms(ulLoop_Tick, BARC_WAIT_RESPONSE_TIME))
				{
						if(QueueGet(&m_bUartRxQueue1, &bRxGetBuf1) == TRUE)
						{
								if(bRxGetBuf1 == '.')
								{
										break;
								}
						}
				}
		}
#endif

		return cResult;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void WakeupBarcoder(void)
{
		// Because Sleep/Hibernate mode cannot be woken up by SW command or without WAKE pin
		HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_SET);					// BPWR off
		delay_ms(10);
		HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_RESET);				// BPWR on
		delay_ms(10);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void TriggerBarcoder(void)
{
		u8 sData_Str[30], bStr_Len;

		bStr_Len = 0;
		BARCODE_TRIGGER_ACTIVATE(sData_Str, bStr_Len);
		while(HAL_BUSY == HAL_UART_Transmit_IT(&Uart1Handle, sData_Str, bStr_Len));
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void UntriggerBarcoder(void)
{
		u8 sData_Str[30], bStr_Len;

		bStr_Len = 0;
		BARCODE_TRIGGER_DEACTIVATE(sData_Str, bStr_Len);
		while(HAL_BUSY == HAL_UART_Transmit_IT(&Uart1Handle, sData_Str, bStr_Len));
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
BOOL IsBarcValid(void)
{
		u8 i, bBarcNextStr[20];

		if(g_bBarcTempLen != tSl900a.bBarcLen)
				return FALSE;

		for(i = 0; i < tSl900a.bBarcLen; i++)
		{
				if((bBarcLastStr[i] < 0x30) || (bBarcLastStr[i] > 0x39))
						return FALSE;
		}

		memcpy(bBarcNextStr, bBarcLastStr, g_bBarcTempLen + 1);		// including "0" to be a string
		bBarcNextStr[g_bBarcTempLen - 1] += 0x01;

		for(i = 0; i < (g_bBarcTempLen - 1); i++)
		{
				if(bBarcNextStr[(g_bBarcTempLen - 1) - i] > 0x39)
				{
						bBarcNextStr[(g_bBarcTempLen - 1) - i] -= 0x0A;
						bBarcNextStr[(g_bBarcTempLen - 1) - i - 1] += 0x01;
				}
				else
						break;
		}

		if(memcmp(bBarcNextStr, bBarcTempStr, g_bBarcTempLen) == 0)
				return TRUE;
		else
				return FALSE;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 ReadBarcode(void)
{
		s8 cResult = ERR_BARC_READ;

		if(QueueGet(&m_bUartRxQueue1, &bRxGetBuf1) == TRUE)
		{
				if(bRxGetBuf1 == 0x0A)
				{
						bBarcTempStr[g_bBarcTempLen] = 0;			// for complete string

						if(tSl900a.bBarcNum == 0)
						{
								tSl900a.bBarcNum = 1;
								tSl900a.bBarcLen = g_bBarcTempLen;
								memcpy(tSl900a.bBarcStr, bBarcTempStr, g_bBarcTempLen );
								memcpy(bBarcLastStr, bBarcTempStr, g_bBarcTempLen + 1);		// including "0" to display the string

								cResult = ERR_BARC_OK;
						}
						else
						{
								if(IsBarcValid() == TRUE)
								{
										tSl900a.bBarcNum++;
										memcpy(bBarcLastStr, bBarcTempStr, g_bBarcTempLen + 1);		// including "0" to display the string

										cResult = ERR_BARC_OK;
								}
								else
										cResult = ERR_AUR700_INVALID_BARCSTR;
						}
				}
				else if(bRxGetBuf1 != 0x0D)
				{
						if(g_bBarcTempLen < 18)								// TL10 only can save 18 bytes of barcode
								bBarcTempStr[g_bBarcTempLen++] = bRxGetBuf1;
				}
		}

		return cResult;
}





