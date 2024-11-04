/*************************************************************************************
 *
 *
 *************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t bRxPutBuf1, bRxPutBuf2, bRxGetBuf1, bRxGetBuf2;

u8 bLoopTask = B2E_TASK_NONE, bLoopMenu = LOOP_DISABLE, bLoopLEDS = LOOP_DISABLE, bLoopVIBR = LOOP_DISABLE, bLoopBUZZ = LOOP_DISABLE;
u32 ulLoop_Tick, ulLoop_Timeout, ulADC_Tick, ulOnOff_Tick, ulEnBLE_Tick;
u32 ulMenu_Tick, ulMenu_Timeout, ulCheck_Tick, ulCheck_Timeout, ulEvent_Tick, ulLEDS_Timeout, ulVIBR_Timeout, ulBUZZ_Timeout;
u32 ulIdle_Tick, ulIdle_Timeout;

u8 bReadBank;
u16 uReadWdAddr, uReadWdLen;
u8 bSelectBank, bWriteBank;
u8 bSelMask[CMD_MAX_DATA_SIZE], bWrData[CMD_MAX_DATA_SIZE];
u16 uSelWdAddr, uWriteWdAddr, uSelWdLen, uWriteWdLen;
u16 uTblLogNum;

u8 g_CmdSource = CMD_SOURCE_NONE;
u8 bCountOnOff = FALSE, bCountEnBLE = FALSE;
u8 bIdleBreak = FALSE, bIdleStatus = IDLE_NORMAL_RUN;
u8 bBLEConnected = BLE_STATE_DIS;

u8 bQueryTarget;
u32 ulTagPassword;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config_StopEntry(void);
static void SystemClock_Config_StopExit(void);
void LowPower_StopMode_Enter(void);
void LowPower_LpSleepMode_Enter(void);
void LowPower_LpRunMode_Enter(void);
void LowPower_LpRunMode_Exit(void);



/* Private functions ---------------------------------------------------------*/

void NVIC_SetVectorTable(unsigned long NVIC_VectTab, unsigned long Offset)
        {

        SCB->VTOR = NVIC_VectTab | (Offset & (unsigned int)0x1FFFFF80);
        }

#define NVIC_VectTab_FLASH      (0x8000000)
#define VectTab_Offset           0x4000

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	s8 cResult;
	u16 uLen;

	HAL_RCC_DeInit();

    HAL_Init();

	// 
#if defined(NEW_AUR700_CODES_LPRUNMODE)
	LowPower_LpRunMode_Enter();
	LowPower_LpRunMode_Exit();
#endif

	// Enter sleep or Wake up
#if defined(NEW_AUR700_CODES_STOPMODE)
	LowPower_StopMode_Enter();
#elif defined(NEW_AUR700_CODES_LPSLPMODE)
	LowPower_LpSleepMode_Enter();
#else
	SystemClock_Config_StopExit();		// for USB and delay_us clock
#endif

	/* Configure the system clock to 32 MHz */
  SystemClock_Config();

	if(FLASH_EEPROM_IsEmpty() == TRUE)
			g_tRegisters = DEFALUT_UHF_REGISTERS;
	else
	{
			uLen = sizeof(g_tRegisters.s2);
			FLASH_EEPROM_ReadBytes(g_tRegisters.bRegisters, 0, &uLen);
	}

	DeviceInit();				            // this must be placed after "DEFALUT_UHF_REGISTERS" and "EEPROM_ReadBytes()"
	__HAL_RCC_CLEAR_RESET_FLAGS();			// this must be placed after "DeviceInit()"

	DefaultInit();

	// Assign the buffer for UART receiver
	if(HAL_UART_Receive_IT(&Uart1Handle, &bRxPutBuf1, 1) != HAL_OK)
			delay_us(10);
	if(HAL_UART_Receive_IT(&Uart2Handle, &bRxPutBuf2, 1) != HAL_OK)
			delay_us(10);
	if(USBD_CDC_ReceivePacket(&UsbdHandle) != USBD_OK)
			delay_us(10);

	ADC_DataReCollect();
	ulADC_Tick = HAL_GetTick();
	ulIdle_Timeout = g_tRegisters.s2.ulEnterIdleTime;
	ulIdle_Tick = HAL_GetTick();

	// prevent wrong display if USB is just plugged during device initialization after power-on
	ScreenInit();				// this must be placed after "ADC_DataReCollect()"

	while(1)
	{
#ifdef NEW_AUR700_CODES_ONOFF

			if(bCountOnOff == FALSE)
			{
					if(HAL_GPIO_ReadPin(GPIOA, RIGHT_KEY_PIN) == GPIO_PIN_RESET)
					{
							bCountOnOff = TRUE;
							ulOnOff_Tick = HAL_GetTick();
					}
			}
			else
			{
					if(HAL_GPIO_ReadPin(GPIOA, RIGHT_KEY_PIN) == GPIO_PIN_RESET)
					{
							if(IsTimeout_ms(ulOnOff_Tick, 2000))
							{
									if(g_tRegisters.s2.ucVibrEnable)
									{
											HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_SET);
											HAL_Delay(100);
											HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_RESET);
									}

									DeviceDeInit();

									NVIC_SystemReset();

									bCountOnOff = FALSE;

									HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_RESET);

									SystemClock_Config();

									g_tRegisters = DEFALUT_UHF_REGISTERS;

									QueueInit(&m_bUartRxQueue1);
									QueueInit(&m_bUartRxQueue2);
									QueueInit(&m_bUsbRxQueue);

									DeviceInit();
									DefaultInit();

									ADC_DataReCollect();
									ulADC_Tick = HAL_GetTick();

									if(HAL_UART_Receive_IT(&Uart1Handle, &bRxPutBuf1, 1) != HAL_OK)
											delay_us(10);
									if(USBD_CDC_ReceivePacket(&UsbdHandle) != USBD_OK)
											delay_us(10);

									HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET);
							}
					}
					else
					{
							bCountOnOff = FALSE;
					}
			}
#endif

#ifdef NEW_AUR700_CODES_IDLE

			switch(bIdleStatus)
			{
			case IDLE_NORMAL_RUN:

					if(bIdleBreak == TRUE)
					{
							ulIdle_Tick = HAL_GetTick();

							bIdleBreak = FALSE;
					}
					else
					{
							if(bBLEConnected == BLE_STATE_DIS)
							{
									if(IsTimeout_ms(ulIdle_Tick, ulIdle_Timeout))
									{
											OLED_Screen_Display(OLED_OFF);

											bIdleStatus = IDLE_OLED_OFF;
											ulIdle_Tick = HAL_GetTick();
									}
							}
					}
					break;
			case IDLE_OLED_OFF:

					if(bIdleBreak == TRUE)
					{
							OLED_Screen_Display(OLED_ON);

							bIdleStatus = IDLE_NORMAL_RUN;
							ulIdle_Tick = HAL_GetTick();

							bIdleBreak = FALSE;
					}
					else
					{
							if(bBLEConnected == BLE_STATE_DIS)
							{
									if(HAL_GPIO_ReadPin(GPIOA, USB_CHARGE_PIN) != GPIO_PIN_RESET)
									{
											if(IsTimeout_ms(ulIdle_Tick, IDLE_PWOFF_TIME_MS))
											{
													DeviceDeInit();

													NVIC_SystemReset();
											}
									}
							}
					}
					break;
			default:

					bIdleStatus = IDLE_NORMAL_RUN;
			}
#endif

#ifdef NEW_AUR700_CODES_ADC

			// Collect ADC results for battery capacity calculation
			if(IsTimeout_ms(ulADC_Tick, ADC_REC_TIME_MS))
			{
					ADC_DataCollect();

					ulADC_Tick = HAL_GetTick();
			}
#endif

#ifdef NEW_AUR700_CODES_MENU

			if(bLoopMenu != LOOP_DISABLE)
			{
					if((bLoopMenu == LOOP_EN_FAIL) || (bLoopMenu == LOOP_EN_DONE))
					{
							if(IsTimeout_ms(ulMenu_Tick, ulMenu_Timeout))
							{
									display_menu.bEvent = EVENT_TIMEOUT_BACK;
									bLoopMenu = LOOP_DISABLE;
							}
					}
					else if(bLoopMenu == LOOP_MAIN_CHECK)
					{
							if(IsTimeout_ms(ulCheck_Tick, ulCheck_Timeout))
							{
									ulCheck_Tick = HAL_GetTick();
									display_menu.bEvent = EVENT_MENU_CHECK;
							}
					}
					else if(bLoopMenu == LOOP_SUB_CHECK)
					{
							if(IsTimeout_ms(ulMenu_Tick, ulMenu_Timeout))
							{
									display_menu.bEvent = EVENT_TIMEOUT_BACK;
									bLoopMenu = LOOP_DISABLE;
							}
							else
							{
									if(IsTimeout_ms(ulCheck_Tick, ulCheck_Timeout))
									{
											ulCheck_Tick = HAL_GetTick();
											display_menu.bEvent = EVENT_MENU_CHECK;
									}
							}
					}
			}

			if(bLoopLEDS != LOOP_DISABLE)
			{
					if(IsTimeout_ms(ulEvent_Tick, ulLEDS_Timeout))
					{
							HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN | LED2_RED_PIN, GPIO_PIN_SET);
							bLoopLEDS = LOOP_DISABLE;
					}
			}
			if(bLoopVIBR != LOOP_DISABLE)
			{
					if(IsTimeout_ms(ulEvent_Tick, ulVIBR_Timeout))
					{
							HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_RESET);
							bLoopVIBR = LOOP_DISABLE;
					}
			}
#ifdef BUZZER_BY_PWM
			if(bLoopBUZZ != LOOP_DISABLE)
			{
					if(IsTimeout_ms(ulEvent_Tick, ulBUZZ_Timeout))
					{
							HAL_TIM_Base_Stop_IT(&Tim3Handle);
							bLoopBUZZ = LOOP_DISABLE;
					}
			}
#endif
#endif

#ifdef NEW_AUR700_CODES_BLE

			if(bCountEnBLE == FALSE)
			{
					if(HAL_GPIO_ReadPin(GPIOA, LEFT_KEY_PIN) == GPIO_PIN_RESET)
					{
							bCountEnBLE = TRUE;
							ulEnBLE_Tick = HAL_GetTick();
					}
			}
			else
			{
					if(HAL_GPIO_ReadPin(GPIOA, LEFT_KEY_PIN) == GPIO_PIN_RESET)
					{
							if(IsTimeout_ms(ulEnBLE_Tick, 2000))
							{
									// Send command to Nordic
									SendDevNameToNordic();
									HAL_Delay(100);
									SendDevIDToNordic();

									// Reset Idle counter
									bIdleBreak = TRUE;

									bCountEnBLE = FALSE;
							}
					}
					else
					{
							bCountEnBLE = FALSE;
					}
			}
#endif

			// to prevent conflict of more than one commands from different source or with different timeout
			if(bLoopTask == B2E_TASK_NONE)
			{
					if(QueueGet(&m_bUartRxQueue2, &bRxGetBuf2) == TRUE)
					{
							g_CmdSource = CMD_SOURCE_UART;
							CommandParser(bRxGetBuf2);
					}
					else if(QueueGet(&m_bUsbRxQueue, &bRxGetBuf2) == TRUE)
					{
							g_CmdSource = CMD_SOURCE_USB;
							CommandParser(bRxGetBuf2);
					}
#ifdef NEW_AUR700_CODES_MENU
					else if(display_menu.bEvent != EVENT_MENU_NONE)
					{
							g_CmdSource = CMD_SOURCE_KEY;
							MenuDisplay();
					}
#endif
			}

#ifdef NEW_AUR700_CODES_KEYS

			uint8_t sTest_Str[10], bTest_Len;
			u8 bShowTime[8], bShowDate[10];
			u16 i;

			if(display_menu.bEvent != EVENT_MENU_NONE)
			{
					switch(display_menu.bEvent)
					{
					case EVENT_KEY_LEFT:
							// enable barcode reader
							HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_RESET);
							TriggerBarcoder();
							break;
					case EVENT_KEY_RIGHT:
							// enable barcode reader
							HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_RESET);
							UntriggerBarcoder();
							break;
					case EVENT_KEY_TRIG:
							break;
					case EVENT_CHARGER_CHANGE:

							// LED2
							HAL_GPIO_TogglePin(GPIOA, LED2_RED_PIN);
							break;
					}

					display_menu.bEvent = EVENT_MENU_NONE;
			}

#endif

			if(bLoopTask != B2E_TASK_NONE)
			{
					if(IsTimeout_ms(ulLoop_Tick, ulLoop_Timeout))
					{
							if(g_CmdSource != CMD_SOURCE_KEY)
							{
									TimeOutHandler(bLoopTask);
							}
							else
							{
									display_menu.bEvent = EVENT_LPTASK_ERROR;
									if(display_menu.bData[0] == (u8)ERR_GEN2_OK)		// when no any error but timeout just happened
											display_menu.bData[0] = (u8)ERR_TIMEOUT;
							}

							bLoopTask = B2E_TASK_NONE;

							g_bEndInventoryRound = FALSE;
							g_bEndInventoryNum = 0;

							// for preamble error
							m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
							uhfSetSensitivity(m_cRxSensitivity);

							uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
					}

					switch(bLoopTask)
					{
#ifdef B2E_FUNCTION_SUPPORT
					case B2E_TASK_SEARCH:
							cResult = B2ESearchAllTag();

							if(g_bEndInventoryRound == TRUE)
							{
									g_bEndInventoryRound = FALSE;
									g_bEndInventoryNum = 0;

									// reverse the target for next search
									if(bQueryTarget & GEN2_QRY_TARGET_BOTH)
									{
											bQueryTarget ^= GEN2_QRY_TARGET_MASK;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);
									}
							}
							break;
					case B2E_TASK_READALL:
							cResult = B2EReadTagBank(B2E_READ_ALL, bReadBank, uReadWdAddr, uReadWdLen);

							if(g_bEndInventoryRound == TRUE)
							{
									g_bEndInventoryRound = FALSE;
									g_bEndInventoryNum = 0;

									// reverse the target for next search
									if(bQueryTarget & GEN2_QRY_TARGET_BOTH)
									{
											bQueryTarget ^= GEN2_QRY_TARGET_MASK;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);
									}
							}
							break;
					case B2E_TASK_WRITETAG:
							cResult = B2ESelBankWrBank(bSelectBank, uSelWdAddr, uSelWdLen, bSelMask, bWriteBank, uWriteWdAddr, uWriteWdLen, bWrData);

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;
					case B2E_TASK_READTAG:
							cResult = B2ESelBankRdBank(bSelectBank, uSelWdAddr, uSelWdLen, bSelMask, bReadBank, uReadWdAddr, uReadWdLen);

							if(g_bEndInventoryRound == TRUE)
							{
									g_bEndInventoryRound = FALSE;
									g_bEndInventoryNum = 0;
							}
							break;
					case B2E_TASK_LOCKTAG:
							cResult = B2ESelBankLockTag(bSelectBank, uSelWdAddr, uSelWdLen, bSelMask);

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;
					case B2E_TASK_KILLTAG:
							cResult = B2ESelBankKillTag(bSelectBank, uSelWdAddr, uSelWdLen, bSelMask);

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;
					case B2E_TASK_UNTRACETAG:
							cResult = B2ESelBankUntraceTag(bSelectBank, uSelWdAddr, uSelWdLen, bSelMask);

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;

// for Gen2 radio test v
					case B2E_TEST_CONTWAVE:
							if(uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON)
									uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);

							ulLoop_Tick = HAL_GetTick();		// for endless loop
							break;
					case B2E_TEST_MODULATION:
							if(uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON)
									uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);
							cResult = B2ESearchAllTag();

							ulLoop_Tick = HAL_GetTick();		// for endless loop
							break;
					case B2E_TEST_HOPPING:
							if(uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON)
									checkHoppingFrequencies();
							cResult = B2ESearchAllTag();

							ulLoop_Tick = HAL_GetTick();		// for endless loop
							break;
// for Gen2 radio test ^
#endif		// B2E_FUNCTION_SUPPORT

					//--------------------------- Used by USB/BLE command ---------------------------//

					case CMD_TASK_QUERYTAGRSSI:
							cResult = AURQueryTagRSSI();

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;

					case CMD_TASK_RDPRESSBTN:
							cResult = AURReadPressBtn();

							if(cResult == ERR_NONE)
							{
									bLoopTask = B2E_TASK_NONE;
							}
							break;

					case CMD_TASK_READEPCTID:
							cResult = AURReadEPCTID();

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;

					case CMD_TASK_WRITEEPC:
							cResult = AURWriteEPC(uWriteWdLen, bWrData);

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;

					case CMD_TASK_STARTLOGGER:
							cResult = AURStartLog();

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;

					case CMD_TASK_STOPLOGGER:
							cResult = AURStopLog();

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;

					case CMD_TASK_READBARCODE:
							cResult = AURReadBarcode();

							if(cResult == ERR_BARC_OK)
							{
									bLoopTask = B2E_TASK_NONE;

									HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_SET);					// BPWR off
							}
							break;

					case CMD_TEST_CONTWAVE:
							if(uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON)
									uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);

							ulLoop_Tick = HAL_GetTick();		// for endless loop
							break;

					case CMD_TEST_MODULATION:
							if(uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON)
									uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);

							cResult = gen2Inventory();

							ulLoop_Tick = HAL_GetTick();		// for endless loop
							break;

					case CMD_TEST_HOPPING:
							if(uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON)
									checkHoppingFrequencies();

							cResult = gen2Inventory();

							ulLoop_Tick = HAL_GetTick();		// for endless loop
							break;

					//--------------------------- Used by Key event ---------------------------//

					case KEY_TASK_RDLOGGERSTATE:
							cResult = ReadLoggerState();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;
					case KEY_TASK_STARTLOGGER:
							cResult = StartLoggingAction();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;
					case KEY_TASK_STOPLOGGER:
							cResult = StopLogging();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;
					case KEY_TASK_RDLOGGERDATA:
							cResult = ReadLoggerData(&uTblLogNum);
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							else if((cResult == ERR_AUR700_IS_FULL) || (cResult == ERR_AUR700_IS_EMPTY) || (cResult == ERR_AUR700_ERASE_FAIL))
							{
									display_menu.bEvent = EVENT_LPTASK_ERROR;

									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;
					case KEY_TASK_RDLOCATIONID:
							cResult = ReadLocationID();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							else if(cResult == ERR_AUR700_IS_FULL)
							{
									display_menu.bEvent = EVENT_LPTASK_ERROR;

									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;
					case KEY_TASK_WRLOCATIONID:
							cResult = WriteLocationID();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;
					case KEY_TASK_READBARCODE:
							cResult = ReadBarcode();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_BARC_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;
									}
							}
							else if(cResult == ERR_AUR700_INVALID_BARCSTR)
							{
									display_menu.bEvent = EVENT_LPTASK_ERROR;

									bLoopTask = B2E_TASK_NONE;
							}
							break;
					case KEY_TASK_WRITEBARCODE:
							cResult = WriteBarcode();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;

					//--------------------------- Used by Internal event ---------------------------//

					case KEY_TASK_STLOGGERCHECK:
							cResult = StartLoggingCheck();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							else if((cResult == ERR_TAG_NO_POWER) || (cResult == ERR_AUR700_LOG_ACTIVATED) || (cResult == ERR_AUR700_NOT_EMPTY))
							{
									display_menu.bEvent = EVENT_LPTASK_ERROR;

									bLoopTask = B2E_TASK_NONE;

									// for preamble error
									m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
									uhfSetSensitivity(m_cRxSensitivity);

									uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
							}
							break;
					case KEY_TASK_SETPASSWORD:
							cResult = OpenPasswordAll(SL900_SETPW);
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;
					case KEY_TASK_OPENAREA:
							cResult = OpenPasswordAll(SL900_OPEN);
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;
					case KEY_TASK_CLRPASSWORD:
							cResult = OpenPasswordAll(SL900_CLRPW);
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;
					case KEY_TASK_INVENTORY:
							m_bIsAccessTag = 1;
							g_tQueryParams.bTarget = bQueryTarget & GEN2_QRY_TARGET_MASK;//GEN2_QRY_TARGET_A;

							cResult = gen2Inventory();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;

//--------------------------- Used by Internal event ---------------------------//

					case KEY_TASK_DEBUGTEST:
							cResult = FullReadLoggerState();
							display_menu.bData[0] = (u8)cResult;

							if(cResult == ERR_GEN2_OK)
							{
									bLoopTask = task_list.bTaskBuf[++task_list.bTaskIdx];

									if(bLoopTask == B2E_TASK_NONE)
									{
											display_menu.bEvent = EVENT_LPTASK_DONE;

											// for preamble error
											m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
											uhfSetSensitivity(m_cRxSensitivity);

											uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
									}
							}
							break;
					case B2E_TASK_NONE:				// when timeout just happened
							break;

					default:
							cResult = ERR_CMD_INCORRECT_CODE;

							bLoopTask = B2E_TASK_NONE;		// to stop the loop from running

							m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
							uhfSetSensitivity(m_cRxSensitivity);

							uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
					}
			}
	}

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 4000000
  *            HSI Frequency(Hz)              = 16000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = 24 (if HSE) or 6 (if HSI)
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
#ifdef ORG_SYS_CLK
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  /* Set Voltage scale1 as MCU will run at 32MHz */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;//RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL24;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}
#else
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    __PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                                       |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL24;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

    __SYSCFG_CLK_ENABLE();

}
#endif

/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
  *            + Regulator in LP mode
  *            + VREFINT OFF, with fast wakeup enabled
  *            + HSI as SysClk after Wake Up
  *            + No IWDG
  *            + Automatic Wakeup using RTC clocked by LSI (after ~4s)
  * @param  None
  * @retval None
  */
static void SystemPower_Config_StopEntry(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Enable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Disable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
}

/**
  * @brief  Configures system clock after wake-up from STOP: enable HSI, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
static void SystemClock_Config_StopExit(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
	RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = (MSI)
  *            MSI Range                      = 2
  *            SYSCLK(Hz)                     = 32000
  *            HCLK(Hz)                       = 32000
  *            AHB Prescaler                  = 2
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Main regulator output voltage  = Scale2 mode
  * @param  None
  * @retval None
  */
void SystemClock_Config_LpRunEntry(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);//(PWR_REGULATOR_VOLTAGE_SCALE3);

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;//RCC_MSIRANGE_0;
  RCC_OscInitStruct.MSICalibrationValue = 0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;//RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  /* Set MSI range to 0 */
  __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);

}

/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow : 
  *            + System Running at MSI (~32KHz)
  *            + Flash 0 wait state  
  *            + Voltage Range 2
  *            + Code running from Internal FLASH
  *            + Wakeup using Key Button PC.13
  * @param  None
  * @retval None
  */
static void SystemPower_Config_LpRunEntry(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();

}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void LowPower_StopMode_Enter(void)
{
#ifdef NEW_AUR700_CODES_STOPMODE

			/* Configure the system Power */
			SystemPower_Config_StopEntry();

			SystemClock_Config_LpRunEntry();		// to solve the power-reset issue after programming

			LP_GPIO_Init();

			while(1)
			{
					HAL_SuspendTick();				// this shouldn't wake up while stop mode

					/* Enter Stop Mode */
					HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

					/* Configures system clock after wake-up from STOP: enable HSI, PLL and select
						PLL as system clock source (HSI and PLL are disabled automatically in STOP mode) */
					SystemClock_Config_StopExit();

					HAL_ResumeTick();

					ulOnOff_Tick = HAL_GetTick();
					while(!IsTimeout_ms(ulOnOff_Tick, 100));		// to skip the inactive status when bounce happened

					if(HAL_GPIO_ReadPin(GPIOA, RIGHT_KEY_PIN) == GPIO_PIN_RESET)
					{
							while(!IsTimeout_ms(ulOnOff_Tick, 2000))
							{
									if(HAL_GPIO_ReadPin(GPIOA, RIGHT_KEY_PIN) != GPIO_PIN_RESET)
											break;
							}
							if(IsTimeout_ms(ulOnOff_Tick, 2000))
									break;		// enter the normal mode
					}
					else if(HAL_GPIO_ReadPin(GPIOA, USB_CHARGE_PIN) == GPIO_PIN_RESET)
							break;				// enter the normal mode
			}
#endif
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void LowPower_LpSleepMode_Enter(void)
{
#ifdef NEW_AUR700_CODES_LPSLPMODE

			/* Configure the system Power */
			SystemPower_Config_StopEntry();

			SystemClock_Config_LpRunEntry();		// to solve the power-reset issue after programming

			LP_GPIO_Init();

			while(1)
			{
					HAL_SuspendTick();

					/* Enter Stop Mode */
					HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

					/* Configures system clock after wake-up from STOP: enable HSI, PLL and select
						PLL as system clock source (HSI and PLL are disabled automatically in STOP mode) */
					SystemClock_Config_StopExit();

					HAL_ResumeTick();

					ulOnOff_Tick = HAL_GetTick();
					while(!IsTimeout_ms(ulOnOff_Tick, 100));		// to skip the inactive status when bounce happened

					if(HAL_GPIO_ReadPin(GPIOA, RIGHT_KEY_PIN) == GPIO_PIN_RESET)
					{
							while(!IsTimeout_ms(ulOnOff_Tick, 2000))
							{
									if(HAL_GPIO_ReadPin(GPIOA, RIGHT_KEY_PIN) != GPIO_PIN_RESET)
											break;
							}
							if(IsTimeout_ms(ulOnOff_Tick, 2000))
									break;		// enter the normal mode
					}
					else if(HAL_GPIO_ReadPin(GPIOA, USB_CHARGE_PIN) == GPIO_PIN_RESET)
							break;				// enter the normal mode
			}
#endif
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void LowPower_LpRunMode_Enter(void)
{
#ifdef NEW_AUR700_CODES_LPRUNMODE		// this could be ignored since MSI is default clock after reset

			/* Configure the system clock @ 32 KHz */
			SystemClock_Config_LpRunEntry();

			/* Configure the system Power */
			SystemPower_Config_LpRunEntry();

			/* Enter LP RUN mode */
			HAL_PWREx_EnableLowPowerRunMode();

			/* Wait until the system enters LP RUN and the Regulator is in LP mode */
			while(__HAL_PWR_GET_FLAG(PWR_FLAG_REGLP) == RESET);
#endif
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void LowPower_LpRunMode_Exit(void)
{
#ifdef NEW_AUR700_CODES_LPRUNMODE		// this could be ignored since MSI is default clock after reset
			/* Exit LP RUN mode */
			HAL_PWREx_DisableLowPowerRunMode();

			/* Wait until the system exits LP RUN and the Regulator is in main mode */
			while(__HAL_PWR_GET_FLAG(PWR_FLAG_REGLP) != RESET);
#endif
}


// Consider to use this together with Watchdog in case of error happened during power/clock configurations
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */


