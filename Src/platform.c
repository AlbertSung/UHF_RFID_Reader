/**
  ******************************************************************************
  * @file    platform.c
  * @author  Albert
  * @version V1.0.0
  * @date    03-June-2017
  * @brief   platform specific routine
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "gen2.h"
#include "gen2_config.h"
#include "queue.h"
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define _offset_(x,y,max) ((x>=y)? x-y : max+x-y)  //max-y+x

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef Spi2Handle;
UART_HandleTypeDef Uart1Handle, Uart2Handle;
I2C_HandleTypeDef I2c2Handle;
USBD_HandleTypeDef UsbdHandle;
ADC_HandleTypeDef Adc1Handle;
RTC_HandleTypeDef RtcHandle;
TIM_HandleTypeDef Tim3Handle;


const s8 RFPA0133_RF_POWER_TABLE[] = {5, 16, 23, 27};

u16 uBatADC[MAX_ADC_BUF_LEN] = {0};
u8 bAdcNum = 0, bAdcInx = 0;
u8 bLastPercentValue = 255;		// to let the next percent value can be updated


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval None
  */
void delay_us(volatile u32 time)
{    
   volatile u16 i = 0;  
   while(time--)
   {
      i = 10;  
      while(--i){;}    
   }
}

/**
  * @brief  
  * @param  
  * @retval 
  */
bool is_timeout_ms(uint32_t lastcount, uint32_t top)
{
		if(_offset_(HAL_GetTick(), lastcount, 0xFFFFFF) >= top)//nCount )		// Simple 24-bit timer.
				return true;
  	return false;
}


/**
  * @brief  
  * @param  
  * @retval None
  */
void UHF_EN(GPIO_PinState ioState)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, ioState);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void UHF_NCS(GPIO_PinState ioState)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, ioState);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void UHF_PAG(u8 bIsPowerOn)
{
    if(bIsPowerOn)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,(((g_tRegisters.s2.bPowerGain & PA_GAIN16_SEL) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET));  // GB16
       	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,(((g_tRegisters.s2.bPowerGain & PA_GAIN08_SEL) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET));  // GB8
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	// GB16
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	// GB8       
    }
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void OLED_RST(GPIO_PinState ioState)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, ioState);
}


/**
  * @brief  
  * @param  
  * @retval 
  */
s8 SPITxRx(const u8* pbTxData, u8* pbRxData, u16 uLength)
{
		if((pbTxData != NULL) && (pbRxData == NULL))
		{
#ifdef _SERIAL_DEBUG_
        if(g_bDbgShowLog)
            putRxTxEvent("spiTx", pbTxData, uLength);
#endif

				return HAL_SPI_Transmit(&Spi2Handle, (uint8_t*) pbTxData, uLength, 50000);
		}
		else if ((pbTxData == NULL) && (pbRxData != NULL))
		{
#ifdef _SERIAL_DEBUG_
        if(g_bDbgShowLog)
            putRxTxEvent("spiRx", pbRxData, uLength);
#endif

		    return HAL_SPI_Receive(&Spi2Handle, (uint8_t*) pbRxData, uLength, 50000);
		}
		else if((pbTxData != NULL) && (pbRxData != NULL))
		{
				return HAL_SPI_TransmitReceive(&Spi2Handle, (uint8_t*) pbTxData, (uint8_t*) pbRxData, uLength, 50000);
		}

    return HAL_ERROR;				
}

/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1)
		{
				QueuePut(&m_bUartRxQueue1, bRxPutBuf1);

				HAL_UART_Receive_IT(huart, &bRxPutBuf1, 1);
		}
		else if(huart->Instance == USART2)
		{
				QueuePut(&m_bUartRxQueue2, bRxPutBuf2);

				HAL_UART_Receive_IT(huart, &bRxPutBuf2, 1);
		}
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		bIdleBreak = TRUE;

		if (GPIO_Pin == AS3993_IRQ_PIN)
		{
				AS3993_IRQ_Callback();
		}

		else if (GPIO_Pin == USB_CHARGE_PIN)
		{
				display_menu.bEvent = EVENT_CHARGER_CHANGE;
		}
		// to prevent key operation while there is Loop Task running or device is at idle mode
		else if((bLoopTask == B2E_TASK_NONE) && (bIdleStatus == IDLE_NORMAL_RUN))
		{
				if (GPIO_Pin == LEFT_KEY_PIN)
				{
						display_menu.bEvent = EVENT_KEY_LEFT;
				}

				else if (GPIO_Pin == RIGHT_KEY_PIN)
				{
						display_menu.bEvent = EVENT_KEY_RIGHT;
				}

				else if (GPIO_Pin == TRIG_KEY_PIN)
				{
						display_menu.bEvent = EVENT_KEY_TRIG;
				}
		}
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
		// consider to use the flag to indicate the data from EEPROM already can be used

}

#ifdef BUZZER_BY_PWM
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(bLoopBUZZ == LOOP_EN_DONE)
				HAL_GPIO_TogglePin(GPIOA, BUZZ_OUT_PIN);
		else if(bLoopBUZZ == LOOP_EN_FAIL)
		{
				if(!IsTimeout_ms(ulEvent_Tick, 200))
						HAL_GPIO_TogglePin(GPIOA, BUZZ_OUT_PIN);
				else if(!IsTimeout_ms(ulEvent_Tick, 300))
						HAL_GPIO_WritePin(GPIOA, BUZZ_OUT_PIN, GPIO_PIN_RESET);
				else if(!IsTimeout_ms(ulEvent_Tick, 500))
						HAL_GPIO_TogglePin(GPIOA, BUZZ_OUT_PIN);
				else if(!IsTimeout_ms(ulEvent_Tick, 600))
						HAL_GPIO_WritePin(GPIOA, BUZZ_OUT_PIN, GPIO_PIN_RESET);
				else if(!IsTimeout_ms(ulEvent_Tick, 800))
						HAL_GPIO_TogglePin(GPIOA, BUZZ_OUT_PIN);
				else
						HAL_GPIO_WritePin(GPIOA, BUZZ_OUT_PIN, GPIO_PIN_RESET);
		}
}
#endif


/******************************************************************************/
/**
  * @brief  
  * @param  None
  * @retval None
  */
void LP_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* Initialize Right key for EXTI */
		GPIO_InitStruct.Pin = RIGHT_KEY_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

		/* Initialize Charger pin for EXTI */
    GPIO_InitStruct.Pin = USB_CHARGE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Enable and set EXTI line 0 Interrupt to the lowest priority */
		HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  		// De-assert SPI NSS

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); 		// AS3993 Disabled
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); 		// RF PA G16 Off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); 		// PA PA G8 Off

    GPIO_InitStruct.Pin = AS3993_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);							// AS3993 IRQ

	/* Enable and set EXTI line 1 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    GPIO_InitStruct.Pin = SYS_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, SYS_EN_PIN, GPIO_PIN_RESET);   	    // Disable SYS_EN, VCC5

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);			// OLED_RST

    GPIO_InitStruct.Pin = BUZZ_OUT_PIN | BARC_PWR_PIN | VIBR_OUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, BUZZ_OUT_PIN, GPIO_PIN_RESET); 		// Buzzer off
    HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_RESET); 		// Vibrator off
    HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_SET);   		// BPWR off

    GPIO_InitStruct.Pin = LED2_RED_PIN | LED1_GRN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);			// Red LED off
	HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 			// Green LED off

    GPIO_InitStruct.Pin = LEFT_KEY_PIN | RIGHT_KEY_PIN | TRIG_KEY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);							// Left, Right, Trigger keys

	/* Enable and set EXTI lines 9 to 5 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* Enable and set EXTI lines 15 to 10 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    GPIO_InitStruct.Pin = USB_CHARGE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);							// Charger

	/* Enable and set EXTI line 0 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief  SPI2 init function
  * @param  None
  * @retval None
  */
void MX_SPI2_Init(void)
{
    Spi2Handle.Instance = SPI2;
    Spi2Handle.Init.Mode = SPI_MODE_MASTER;
    Spi2Handle.Init.Direction = SPI_DIRECTION_2LINES;
    Spi2Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    Spi2Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    Spi2Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    Spi2Handle.Init.NSS = SPI_NSS_SOFT;
    Spi2Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    Spi2Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    Spi2Handle.Init.TIMode = SPI_TIMODE_DISABLED;
    Spi2Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;

		HAL_SPI_Init(&Spi2Handle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{
    Uart1Handle.Instance = USART1;
    Uart1Handle.Init.BaudRate = 9600;//115200;			// used for Barcode
    Uart1Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart1Handle.Init.StopBits = UART_STOPBITS_1;
    Uart1Handle.Init.Parity = UART_PARITY_NONE;
    Uart1Handle.Init.Mode = UART_MODE_TX_RX;
    Uart1Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    Uart1Handle.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&Uart1Handle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{
    Uart2Handle.Instance = USART2;
    Uart2Handle.Init.BaudRate = 38400;//9600;			// used for BLE
    Uart2Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart2Handle.Init.StopBits = UART_STOPBITS_1;
    Uart2Handle.Init.Parity = UART_PARITY_NONE;
    Uart2Handle.Init.Mode = UART_MODE_TX_RX;
    Uart2Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    Uart2Handle.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&Uart2Handle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_I2C2_Init(void)
{
		I2c2Handle.Instance             = I2C2;
		I2c2Handle.Init.ClockSpeed      = 400000;
		I2c2Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		I2c2Handle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
		I2c2Handle.Init.OwnAddress1     = 0;
		I2c2Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
		I2c2Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		I2c2Handle.Init.OwnAddress2     = 0;
		I2c2Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		I2c2Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

		HAL_I2C_Init(&I2c2Handle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_ADC_Init(void)
{
		ADC_ChannelConfTypeDef   sConfig;

		/* Configuration of AdcHandle init structure: ADC parameters and regular group */
		Adc1Handle.Instance = ADC1;

		HAL_ADC_DeInit(&Adc1Handle);

		Adc1Handle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;//ADC_CLOCK_ASYNC_DIV4;
		Adc1Handle.Init.Resolution            = ADC_RESOLUTION_12B;
		Adc1Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
		Adc1Handle.Init.ScanConvMode          = ADC_SCAN_DISABLE;               /* Sequencer enabled (ADC conversion on several channels, successively, following settings below) */
		Adc1Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
		Adc1Handle.Init.LowPowerAutoWait      = ADC_AUTOWAIT_DISABLE;
		Adc1Handle.Init.LowPowerAutoPowerOff  = ADC_AUTOPOWEROFF_DISABLE;
		Adc1Handle.Init.ChannelsBank          = ADC_CHANNELS_BANK_A;
		Adc1Handle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
		Adc1Handle.Init.NbrOfConversion       = 1;                             /* Sequencer of regular group will convert the 3 first ranks: rank1, rank2, rank3 */
		Adc1Handle.Init.DiscontinuousConvMode = DISABLE;                        /* Sequencer of regular group will convert the sequence in several sub-divided sequences */
		Adc1Handle.Init.NbrOfDiscConversion   = 1;                             /* Sequencer of regular group will convert ranks one by one, at each conversion trig */
		Adc1Handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Trig of conversion start done manually by software, without external event */
		Adc1Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
		Adc1Handle.Init.DMAContinuousRequests = DISABLE;                        /* ADC-DMA continuous requests to match with DMA configured in circular mode */

		HAL_ADC_Init(&Adc1Handle);

		/* Configuration of channel on ADCx regular group on sequencer rank 1 */
		/* Note: Considering IT occurring after each ADC conversion (IT by DMA end  */
		/*       of transfer), select sampling time and ADC clock with sufficient   */
		/*       duration to not create an overhead situation in IRQHandler.        */
		/* Note: Set long sampling time due to internal channels (VrefInt,          */
		/*       temperature sensor) constraints.                                   */
		/*       For example, sampling time of temperature sensor must be higher    */
		/*       than 4us. Refer to device datasheet for min/typ/max values.        */
		sConfig.Channel      = ADC_CHANNEL_5;
		sConfig.Rank         = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_96CYCLES;

		HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_ADC_DeInit(void)
{
		HAL_ADC_Stop(&Adc1Handle);
		HAL_ADC_DeInit(&Adc1Handle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_RTC_Init(void)
{
		RTC_DateTypeDef  sdatestructure;
		RTC_TimeTypeDef  stimestructure;

		/*##-1- Configure the RTC peripheral #######################################*/
		RtcHandle.Instance = RTC;

		/* Configure RTC prescaler and RTC data registers */
		/* RTC configured as follows:
				- Hour Format    = Format 24
				- Asynch Prediv  = Value according to source clock
				- Synch Prediv   = Value according to source clock
				- OutPut         = Output Disable
				- OutPutPolarity = High Polarity
				- OutPutType     = Open Drain */
		RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
		RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
		RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
		RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
		RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
		RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

		HAL_RTC_Init(&RtcHandle);

		if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == RESET)		// don't reset the date/time after wake up from Stop/Sleep mode
		{
				/*##-1- Configure the Date #################################################*/
				sdatestructure.Year = g_tRegisters.s2.tRtc.year;
				sdatestructure.Month = g_tRegisters.s2.tRtc.month;
				sdatestructure.Date = g_tRegisters.s2.tRtc.date;
				sdatestructure.WeekDay = g_tRegisters.s2.tRtc.week;

				HAL_RTC_SetDate(&RtcHandle, &sdatestructure, RTC_FORMAT_BIN);

				/*##-2- Configure the Time #################################################*/
				stimestructure.Hours = g_tRegisters.s2.tRtc.hour;
				stimestructure.Minutes = g_tRegisters.s2.tRtc.minute;
				stimestructure.Seconds = g_tRegisters.s2.tRtc.second;
				stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
				stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
				stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;//RTC_STOREOPERATION_SET;

				HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN);
		}
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_RTC_DeInit(void)
{
		HAL_RTC_DeInit(&RtcHandle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* Init Device Library */
  USBD_Init(&UsbdHandle, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&UsbdHandle, USBD_CDC_CLASS);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&UsbdHandle, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&UsbdHandle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_USB_DEVICE_DeInit(void)
{
    USBD_Stop(&UsbdHandle);

    USBD_DeInit(&UsbdHandle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_TIM_Init(void)
{
		uint32_t uwPrescalerValue = 0;

		/*##-1- Configure the TIM peripheral #######################################*/
		/* -----------------------------------------------------------------------
			In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1),
			since APB1 prescaler is equal to 1.
				TIM3CLK = PCLK1
				PCLK1 = HCLK
				=> TIM3CLK = HCLK = SystemCoreClock
			To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
			Prescaler = (TIM3CLK / TIM3 counter clock) - 1
			Prescaler = (SystemCoreClock /10 KHz) - 1

			Note:
				SystemCoreClock variable holds HCLK frequency and is defined in system_stm32l1xx.c file.
				Each time the core clock (HCLK) changes, user had to update SystemCoreClock
				variable value. Otherwise, any configuration based on this variable will be incorrect.
				This variable is updated in three ways:
					1) by calling CMSIS function SystemCoreClockUpdate()
					2) by calling HAL API function HAL_RCC_GetSysClockFreq()
					3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
		----------------------------------------------------------------------- */

		/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
		uwPrescalerValue = (uint32_t)(SystemCoreClock / 200000) - 1;

		/* Set TIMx instance */
		Tim3Handle.Instance = TIM3;

		/* Initialize TIMx peripheral as follows:
				+ Period = 10000 - 1
				+ Prescaler = (SystemCoreClock/10000) - 1
				+ ClockDivision = 0
				+ Counter direction = Up
		*/
		Tim3Handle.Init.Period            = 28 - 1;
		Tim3Handle.Init.Prescaler         = uwPrescalerValue;
		Tim3Handle.Init.ClockDivision     = 0;
		Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;

		HAL_TIM_Base_Init(&Tim3Handle);

		/*##-2- Start the TIM Base generation in interrupt mode ####################*/
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void MX_TIM_DeInit(void)
{
		HAL_TIM_Base_DeInit(&Tim3Handle);
}


/******************************************************************************/

/**
  * @brief  
  * @param  None
  * @retval None
  */
void DeviceInit(void)
{
		u32 ulWait_Tick;

		MX_GPIO_Init();

		MX_SPI2_Init();

		MX_USART1_UART_Init();
		MX_USART2_UART_Init();

		MX_I2C2_Init();

		MX_USB_DEVICE_Init();

		MX_ADC_Init();

#ifdef BUZZER_BY_PWM
		MX_TIM_Init();
#endif

		MX_RTC_Init();

		OLED_RST(GPIO_PIN_RESET);		// Disable OLED
    HAL_Delay(5);		// at least 3us before power on VOLED
    OLED_RST(GPIO_PIN_SET);			// Enable OLED

		HAL_GPIO_WritePin(GPIOA, SYS_EN_PIN, GPIO_PIN_SET);      // Enable VCC5/VOLED

		HAL_Delay(5);		// after VOLED becomes stable
		OLED_Driver_Init();
    OLED_Clear_Screen();

		HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_RESET);			// Green LED On to indicate the booting status
		ulWait_Tick = HAL_GetTick();																// to control the OLED display time before normal operation

		vPrintText(1, "AUR700");
    vPrintText(2, "Starting up...");
		OLED_Screen_Display(OLED_ON);

		if(g_tRegisters.s2.ucVibrEnable)
		{
				HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_SET);
				HAL_Delay(VIBR_BOOT_TIME_MS);
				HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_RESET);
		}

		gen2Initialize();

		while(!IsTimeout_ms(ulWait_Tick, 2000));
		HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void DeviceDeInit(void)
{
    uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
		uhfSetAntennaPower(UHF_OFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); 		// AS3993 Disabled

		OLED_Screen_Display(OLED_OFF);		// before power off VOLED

		HAL_GPIO_WritePin(GPIOA, SYS_EN_PIN, GPIO_PIN_RESET);      // Disable VCC5/VOLED

#ifdef BUZZER_BY_PWM
		MX_TIM_DeInit();
#endif

		MX_ADC_DeInit();

		MX_USB_DEVICE_DeInit();

		HAL_I2C_DeInit(&I2c2Handle);

		HAL_UART_DeInit(&Uart1Handle);
		HAL_UART_DeInit(&Uart2Handle);

		HAL_SPI_DeInit(&Spi2Handle);

		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All);
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All);

		__HAL_RCC_GPIOA_CLK_DISABLE();		// possibly this is already done when SystemPower_Config_StopEntry()/SystemPower_Config_LpRunEntry() is called
    __HAL_RCC_GPIOB_CLK_DISABLE();		// possibly this is already done when SystemPower_Config_StopEntry()/SystemPower_Config_LpRunEntry() is called
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void DefaultInit(void)
{
		u8 i;

		// Initialize all queues
		QueueInit(&m_bUartRxQueue1);
		QueueInit(&m_bUartRxQueue2);
		QueueInit(&m_bUsbRxQueue);

		// Initialize menu parameters
		display_menu.bMainIdx = SUB_PAGE_MAIN;
		display_menu.bSubIdx = SUB_PAGE_MAIN;
		display_menu.bEvent = EVENT_MENU_NONE;

		// Initialize table parameters
		for(i = 0; i < TBL_CNT_MAX; i++)
		{
				tbl_info[i].uTotalRecCnt = 0;
				tbl_info[i].uCurrRecNum = 0;
				tbl_info[i].bOpened = TBL_STATE_OFF;
		}
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void ScreenInit(void)
{
		// Initialize menu display
		MenuUpdate(g_tRegisters.s2.tMenu.ucItem[display_menu.bMainIdx - 1]);
		bLoopMenu = LOOP_MAIN_CHECK;
		ulCheck_Timeout = ADC_REC_TIME_MS;
		ulCheck_Tick = HAL_GetTick();
		CheckBatStatus();
}

/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval 
  */
s8 um900SetRFTxPower(s8 cPower)
{
    u8 bGain;

		if(cPower > 0)      // cPower can't > 0, never run!!
    {
        for(bGain = (sizeof(RFPA0133_RF_POWER_TABLE)-2); bGain > 0; bGain--)
        {
            if(cPower > RFPA0133_RF_POWER_TABLE[bGain])
            {
                bGain++;
                break;
            }
        }
        g_tRegisters.s2.bPowerGain = bGain;

        cPower -= RFPA0133_RF_POWER_TABLE[bGain];

        if(cPower < 0)
            cPower = as3993SetRFTxPower(cPower);
    }
    else
    {
        bGain = (PA_GAIN08_SEL | PA_GAIN16_SEL);		
        cPower = as3993SetRFTxPower(cPower);
    }

#ifdef ALB_DEBUG
		printf("%09lu Set RF TX power with %+d \r\n", getSysMilliseconds(), cPower);
#endif
		
    return cPower;
}


/******************************************************************************/
void uart1WriteByte(u8 bData)
{
	u8 pData[1];
	pData[0]=bData;

  if(HAL_UART_Transmit(&Uart1Handle, (uint8_t*)pData, 1, 1000)!= HAL_OK)	
  {
  }

	HAL_Delay(20);	
}

u16 uart1WriteBytes(const u8* pbBuffer, u16 uBufferLength)
{
  u16 uBytesToWrite=0;

	while(uBytesToWrite<uBufferLength)
  {
     uart1WriteByte(pbBuffer[uBytesToWrite]);
     uBytesToWrite++;
  }

	return uBytesToWrite;
}


void uart2WriteByte(u8 bData)
{
	u8 pData[1];
	pData[0]=bData;

  if(HAL_UART_Transmit(&Uart2Handle, (uint8_t*)pData, 1, 1000)!= HAL_OK)	
  {
  }

	HAL_Delay(20);	
}

u16 uart2WriteBytes(const u8* pbBuffer, u16 uBufferLength)
{
	u16 uBytesToWrite=0;

	while(uBytesToWrite<uBufferLength)
  {
			uart2WriteByte(pbBuffer[uBytesToWrite]);
      uBytesToWrite++;
  }

  return uBytesToWrite;
}


// fputc() for printf()
int fputc(int c, FILE* file)
{
    uart1WriteByte((u8)c);
    return ERR_NONE;
}


/******************************************************************************/
/*
 * sl900a start time
 * 31..26 Year[5:0], offset of year: 2000
 * 25..22 Month[3:0]
 * 21..17 Day[4:0]
 * 16..12 Hour[4:0]
 * 11..06 Minute[5:0]
 * 05..00 Second[5:0]
 */
/**
  * @brief  
  * @param  
  * @retval 
  */
void RTC_StartTimeSet(u8* pstartTime)
{
    RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;

		// You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock
    /* Get the RTC current Time */
    RTC_TimeGet(&stimestructureget);
    /* Get the RTC current Date */
    RTC_DateGet(&sdatestructureget);

    pstartTime[0] = ((stimestructureget.Minutes & 0x03) << 6) | (stimestructureget.Seconds);
    pstartTime[1] = ((stimestructureget.Minutes & 0x3C) >> 2) | ((stimestructureget.Hours & 0x0F) << 4);
    pstartTime[2] = ((stimestructureget.Hours & 0x10) >> 4) | (sdatestructureget.Date << 1) | ((sdatestructureget.Month & 0x03) << 6);
    pstartTime[3] = ((sdatestructureget.Month & 0x0C) >> 2) | (sdatestructureget.Year << 2);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void RTC_TimeDateShow(uint8_t* pshowtime, uint8_t* pshowdate)
{
    RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;

		// You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock
    /* Get the RTC current Time */
    RTC_TimeGet(&stimestructureget);
    /* Get the RTC current Date */
    RTC_DateGet(&sdatestructureget);

		/* Display time Format : hh:mm:ss */
    sprintf((char*)pshowtime, "%02d:%02d:%02d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
    /* Display date Format : YY/MM/DD */
    sprintf((char*)pshowdate, "%04d/%02d/%02d", (sdatestructureget.Year+2000), sdatestructureget.Month, sdatestructureget.Date);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
// You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock
void RTC_TimeGet(RTC_TimeTypeDef* ptimestructureget)
{
    /* Get the RTC current Time */
    HAL_RTC_GetTime(&RtcHandle, ptimestructureget, RTC_FORMAT_BIN);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void RTC_DateGet(RTC_DateTypeDef* pdatestructureget)
{
		/* Get the RTC current Date */
		HAL_RTC_GetDate(&RtcHandle, pdatestructureget, RTC_FORMAT_BIN);
}


/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval 
  */
void OLED_WriteCmd(uint8_t bTxData)
{
		uint8_t bTxBuf[2];

		bTxBuf[0] = 0x00;		// Control byte: Co = 0, D/C = 0
		bTxBuf[1] = bTxData;

		while(HAL_I2C_Master_Transmit(&I2c2Handle, (uint16_t)OLED_I2C_ADDRESS, (uint8_t*)bTxBuf, 2, 50) == HAL_BUSY);		// about 0.025ms/byte at 400KHz but I2C_TIMEOUT_BUSY_FLAG is 25ms
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void OLED_WriteData(uint8_t bTxData)
{
		uint8_t bTxBuf[2];

		bTxBuf[0] = 0x40;		// Control byte: Co = 0, D/C = 1
		bTxBuf[1] = bTxData;

		while(HAL_I2C_Master_Transmit(&I2c2Handle, (uint16_t)OLED_I2C_ADDRESS, (uint8_t*)bTxBuf, 2, 50) == HAL_BUSY);		// about 0.025ms/byte at 400KHz but I2C_TIMEOUT_BUSY_FLAG is 25ms
}


/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval 
  */
s8 EEPROM_ReadData(uint8_t* pbBuffer, uint16_t uMemoryAddress, uint16_t* puLength)
{
		s8 cResult;
		u16 uRemainBytes = *puLength;

#ifdef EEPROM_READ_DMA
		cResult = HAL_I2C_Mem_Read_DMA(&I2c2Handle, (uint16_t)EEPROM_ADDRESS, uMemoryAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t *)pbBuffer, uRemainBytes);

#else
		cResult = HAL_I2C_Mem_Read(&I2c2Handle, (uint16_t)EEPROM_ADDRESS, uMemoryAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t *)pbBuffer, uRemainBytes, 5000);

#endif

    /* Reading process Error */
		if(cResult != HAL_OK)
				return cResult;

		/* Wait for the end of the transfer */
		while (HAL_I2C_GetState(&I2c2Handle) != HAL_I2C_STATE_READY);

		return cResult;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 EEPROM_WriteData(uint8_t* pbBuffer, uint16_t uMemoryAddress, uint16_t* puLength)
{
		s8 cResult;
		u16 uWriteBytes, uRemainBytes = *puLength, uBufOffset = 0;

		/* Since page size is 128 bytes, the write procedure will be done in a loop */
		while(uRemainBytes > 0)
		{
				if(uRemainBytes >= EEPROM_PAGESIZE)
						uWriteBytes = EEPROM_PAGESIZE;
				else
						uWriteBytes = uRemainBytes;

				/* Check boundary of page address */
        if(((uMemoryAddress % EEPROM_PAGESIZE) + uWriteBytes) > EEPROM_PAGESIZE)
            uWriteBytes = (EEPROM_PAGESIZE - (uMemoryAddress % EEPROM_PAGESIZE));

				/* Write EEPROM_PAGESIZE */
#ifdef EEPROM_WRITE_DMA
				cResult = HAL_I2C_Mem_Write_DMA(&I2c2Handle , (uint16_t)EEPROM_ADDRESS, uMemoryAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t*)(pbBuffer + uBufOffset), uWriteBytes);

#else
				cResult = HAL_I2C_Mem_Write(&I2c2Handle , (uint16_t)EEPROM_ADDRESS, uMemoryAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t*)(pbBuffer + uBufOffset), uWriteBytes, 5000);

#endif
				/* Writing process Error */
				if(cResult != HAL_OK)
						return cResult;

				/* Wait for the end of the transfer */
				/*  Before starting a new communication transfer, you need to check the current   
					state of the peripheral; if it is busy you need to wait for the end of current
					transfer before starting a new one.
					For simplicity reasons, this example is just waiting till the end of the 
					transfer, but application may perform other tasks while transfer operation
					is ongoing. */
				while (HAL_I2C_GetState(&I2c2Handle) != HAL_I2C_STATE_READY);

				/* Check if the EEPROM is ready for a new operation */
				while (HAL_I2C_IsDeviceReady(&I2c2Handle, EEPROM_ADDRESS, EEPROM_MAX_TRIALS, I2C_XFER_TIMEOUT_MAX) == HAL_TIMEOUT);

				/* Wait for the end of the transfer */
				while (HAL_I2C_GetState(&I2c2Handle) != HAL_I2C_STATE_READY);

				/* Update Remaining bytes and Memory Address values */
				uRemainBytes -= uWriteBytes;
				uMemoryAddress += uWriteBytes;
				uBufOffset += uWriteBytes;
		}

		return cResult;
}


/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval 
  */
s8 FLASH_EEPROM_ReadBytes(uint8_t* pbBuffer, uint32_t ulOffsetAddress, uint16_t* puLength)
{
		s8 cResult = HAL_OK;
		u16 uRemainBytes = *puLength, uBufOffset = 0;
		u32 ulByteAddress = FLASH_EEPROM_BASE + ulOffsetAddress;

		while(uRemainBytes > 0)
		{
				*(pbBuffer + uBufOffset) = *(__IO uint8_t *)ulByteAddress;

				uRemainBytes--;
				ulByteAddress++;
				uBufOffset++;
		}

		return cResult;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 FLASH_EEPROM_WriteBytes(uint8_t* pbBuffer, uint32_t ulOffsetAddress, uint16_t* puLength)
{
		s8 cResult = HAL_OK;
		u16 uRemainBytes = *puLength, uBufOffset = 0;
		u32 ulByteAddress = FLASH_EEPROM_BASE + ulOffsetAddress;

		HAL_FLASHEx_DATAEEPROM_Unlock();

		while(uRemainBytes > 0)
		{
				cResult = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_FASTBYTE, ulByteAddress, *(pbBuffer + uBufOffset));

				if(cResult != HAL_OK)
						break;

				uRemainBytes--;
				ulByteAddress++;
				uBufOffset++;
		}

		HAL_FLASHEx_DATAEEPROM_Lock();

		return cResult;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u8 FLASH_EEPROM_IsEmpty(void)
{
		u8 bBuf[3];
		u16 uLen = 3;

		FLASH_EEPROM_ReadBytes(bBuf, 0, &uLen);

		if((bBuf[0] | bBuf[1] | bBuf[2]) == 0)
				return TRUE;
		else
				return FALSE;
}


/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval 
  */
void ADC_DataCollect(void)
{
		HAL_ADC_Start(&Adc1Handle);

		if(HAL_ADC_PollForConversion(&Adc1Handle, 1) == HAL_OK)
		{
				// Put results into queue
				uBatADC[bAdcInx++] = HAL_ADC_GetValue(&Adc1Handle);
				bAdcNum++;

				if(bAdcInx >= MAX_ADC_BUF_LEN)
						bAdcInx = 0;
				if(bAdcNum >= MAX_ADC_BUF_LEN)
						bAdcNum = MAX_ADC_BUF_LEN;
		}	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void ADC_DataReCollect(void)
{
		// Reset the queue
		bAdcInx = 0;
		bAdcNum = 0;

		ADC_DataCollect();
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u16 ADC_AverageGet(void)
{
		u8 i;
		u32 ulAdcAvg = 0;

		for(i = 0; i < bAdcNum; i++)
		{
				ulAdcAvg += uBatADC[i];
		}
		ulAdcAvg /= bAdcNum;

		return (u16)ulAdcAvg;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u8 ADC_PercentGet(void)
{
		u8 i, bNowPercentValue;
		u16 uAdcAvg;
		u16 BatTable[11] = {3500, 3540, 3590, 3620, 3640, 3670, 3700, 3760, 3820, 3900, 4000};

		// 
		if(HAL_GPIO_ReadPin(GPIOA, USB_CHARGE_PIN) == GPIO_PIN_RESET)
		{
						bNowPercentValue = 255;			// indicate the charging status
		}
		else
		{
				uAdcAvg = ADC_AverageGet();

				if(uAdcAvg < BatTable[0])
				{
						bNowPercentValue = 5;		// why not 0%
				}
				else if(uAdcAvg >= BatTable[10])
				{
						bNowPercentValue = 100;
				}
				else
				{
						for(i = 0; i < 10 ; i++)
						{
								if((uAdcAvg >= BatTable[i]) && (uAdcAvg < BatTable[i + 1]))
								{
										if(i == 0)
												bNowPercentValue = 5;
										else
												bNowPercentValue = i * 10;

										break;
								}
						}
				}
		}

		if((bNowPercentValue < bLastPercentValue) || (bNowPercentValue == 255))
				bLastPercentValue = bNowPercentValue;

		return bLastPercentValue;
}


