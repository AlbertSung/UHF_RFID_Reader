/**
  ******************************************************************************
  * @file    display_menu.c
  * @author  Albert
  * @version V1.0.0
  * @date    23-Jan-2018
  * @brief   Display menu sub-routine
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "display_menu.h"
#include "sl900a_logger.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MenuFormat_t display_menu;
TaskList_t task_list;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



#ifdef NEW_AUR700_CODES_MENU
/******************************************************************************/
/**
  * @brief  
  * @param  None
  * @retval 
  */
u8 BatIndicator(void)
{
		u8 i, bBatPercent;
		u8 bChargingState = FALSE;

		vSetCursor(96);

		bBatPercent = ADC_PercentGet();

		if(bBatPercent < 10)
		{
				for (i = 6; i < 8; i++)
						OLED_Write_Battery(i);
		}
		else if(bBatPercent < 30)
		{
				for (i = 4; i < 6; i++)
						OLED_Write_Battery(i);
		}
		else if(bBatPercent < 80)
		{
				for (i = 2; i < 4; i++)
						OLED_Write_Battery(i);
		}
		else if(bBatPercent < 255)
		{
				for (i = 0; i < 2; i++)
						OLED_Write_Battery(i);
		}
		else		// bBatPercent == 255
		{
				for (i = 8; i < 10; i++)
						OLED_Write_Battery(i);

				bChargingState = TRUE;
		}

		return bChargingState;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void DoneResponse(void)
{
		bLoopLEDS = LOOP_EN_DONE;
		ulLEDS_Timeout = LEDS_SUCCESS_TIME_MS;
		HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_RESET);

		if(g_tRegisters.s2.ucVibrEnable)
		{
				bLoopVIBR = LOOP_EN_DONE;
				ulVIBR_Timeout = VIBR_SUCCESS_TIME_MS;
				HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_SET);
		}

		if(g_tRegisters.s2.ucBuzzEnable)
		{
				bLoopBUZZ = LOOP_EN_DONE;
				ulBUZZ_Timeout = BUZZ_SUCCESS_TIME_MS;
				HAL_TIM_Base_Start_IT(&Tim3Handle);
		}

		ulEvent_Tick = HAL_GetTick();
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void ErrorResponse(void)
{
		bLoopLEDS = LOOP_EN_FAIL;
		ulLEDS_Timeout = LEDS_WARNING_TIME_MS;
		HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_RESET);

		if(g_tRegisters.s2.ucBuzzEnable)
		{
				bLoopBUZZ = LOOP_EN_FAIL;
				ulBUZZ_Timeout = BUZZ_WARNING_TIME_MS;
				HAL_TIM_Base_Start_IT(&Tim3Handle);
		}

		ulEvent_Tick = HAL_GetTick();
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void TrigDoneOff(void)
{
		bLoopLEDS = LOOP_DISABLE;
		bLoopBUZZ = LOOP_DISABLE;
		bLoopVIBR = LOOP_DISABLE;

		HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_RESET);
		HAL_TIM_Base_Stop_IT(&Tim3Handle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void TrigErrorOff(void)
{
		bLoopMenu = LOOP_DISABLE;
		bLoopLEDS = LOOP_DISABLE;
		bLoopBUZZ = LOOP_DISABLE;

		HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);
		HAL_TIM_Base_Stop_IT(&Tim3Handle);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void CheckBatStatus(void)
{
		if(BatIndicator() == TRUE)
		{
				HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_RESET);				// RED on
				HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
		}
		else
		{
				HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
				HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
		}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void MenuUpdate(u8 bMenuItem)
{
		OLED_Clear_Screen();

		switch(bMenuItem)
		{
		case MAIN_READ_LOGSTATUS:

				vPrintText(1, "[Main Menu]");
				vPrintText(2, "Read logger");
				vPrintText(3, "state");
				break;

		case MAIN_START_LOGGING:

				vPrintText(1, "[Main Menu]");
				vPrintText(2, "Start logging");
				break;

		case MAIN_STOP_LOGGING:

				vPrintText(1, "[Main Menu]");
				vPrintText(2, "Stop logging +");
				vPrintText(3, "Read logger data");
				break;

		case MAIN_WRITE_LOCATION:

				vPrintText(1, "[Main Menu]");
				vPrintText(2, "Write Location");
				vPrintText(3, "ID into logger");
				break;

		case MAIN_READ_DEVICEINFO:

				vPrintText(1, "[Main Menu]");
				vPrintText(2, "Device");
				vPrintText(3, "information");
				break;

		case MAIN_BARCODE_LOGGING:

				vPrintText(1, "[Main Menu]");
				vPrintText(2, "Read BarCode +");
				vPrintText(3, "Start Logging");
				break;

		case MAIN_DEBUG_TEST:

				vPrintText(1, "[Main Menu]");
				vPrintText(2, "Debugging and");
				vPrintText(3, "Testing");
				break;

		default:

				vPrintText(1, "[Main Menu]");
				vPrintText(2, "Not supported");
		}
}


/******************************************************************************/
/**
  * @brief  
  * @param  None
  * @retval None
  */
void MenuDisplay(void)
{
		u8 bDelayDay, bDelayHur, bDelayMin, bDelaySec;
		u8 bShowTime[12], bShowDate[12];
		u8 bShowChar[4];

		switch(g_tRegisters.s2.tMenu.ucItem[display_menu.bMainIdx - 1])
		{
		// ================================================================= // Main page 0
		case MAIN_READ_LOGSTATUS:

				switch(display_menu.bSubIdx)
				{
				// ------------------------------------------------------------- // Sub page 0
				case SUB_PAGE_MAIN:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								MenuUpdate(MAIN_READ_LOGSTATUS);

								bLoopMenu = LOOP_MAIN_CHECK;
								ulCheck_Timeout = ADC_REC_TIME_MS;
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								if(--display_menu.bMainIdx == 0)
										display_menu.bMainIdx = g_tRegisters.s2.tMenu.ucTotalNum;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								if(++display_menu.bMainIdx > g_tRegisters.s2.tMenu.ucTotalNum)
										display_menu.bMainIdx = SUB_PAGE_MAIN;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Start processing");
								vPrintText(2, "logger...");
								vPrintText(3, "");
								vPrintText(4, "");

								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								task_list.bTaskBuf[2] = KEY_TASK_RDLOGGERSTATE;
								task_list.bTaskBuf[3] = KEY_TASK_SETPASSWORD;
								task_list.bTaskBuf[4] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
								HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
								bLoopMenu = LOOP_DISABLE;

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								DoneResponse();

								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_CHARGER_CHANGE)
						{
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								CheckBatStatus();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 1
				case SUB_PAGE_1:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_AUR700_OPEN_SETPW)
								{
										vPrintText(1, "Lock memory");
										vPrintText(2, "failed");
								}
								else
								{
										vPrintText(1, "Read logger");
										vPrintText(2, "failed");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigErrorOff();

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 2
				case SUB_PAGE_2:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Temp.: %.1f C", tSl900a.fSensorValue);
								if(tSl900a.bStatusFlag & 0x80)
										vPrintText(2, "State: activated");
								else
										vPrintText(2, "State: ended");
								vPrintText(3, "Records: %3d/739",tSl900a.uRecordNum);
								vPrintText(4, "Interval: %ds",tSl900a.uLogInterval);

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_SUBMENU_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								bLoopMenu = LOOP_DISABLE;

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								display_menu.bSubIdx = SUB_PAGE_3;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigDoneOff();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 3
				case SUB_PAGE_3:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								bDelayDay = tSl900a.ulDelayTime / (24*60*60);
								bDelayHur = tSl900a.ulDelayTime % (24*60*60) / (60*60);
								bDelayMin = tSl900a.ulDelayTime % (60*60) / (60);
								bDelaySec = tSl900a.ulDelayTime % (60);

								vPrintText(1, "S.D.: %2d/%2d/%2d", tSl900a.StartTime[0],tSl900a.StartTime[1],tSl900a.StartTime[2]);
								vPrintText(2, "S.T.: %2d:%2d:%2d", tSl900a.StartTime[3],tSl900a.StartTime[4],tSl900a.StartTime[5]);

								if(bDelayDay > 0)
										vPrintText(3, "Delay: %dd %dh %dm", bDelayDay, bDelayHur, bDelayMin);
								else if(bDelayHur > 0)
										vPrintText(3, "Delay: %dh %dm %ds", bDelayHur, bDelayMin, bDelaySec);
								else if(bDelayMin > 0)
										vPrintText(3, "Delay: %dm %ds", bDelayMin, bDelaySec);
								else if(bDelaySec > 0)
										vPrintText(3, "Delay: %d sec", bDelaySec);
								else
										vPrintText(3, "Delay: NO");

								if(tSl900a.fBattLevel >= SL900_FULLBAT)
										vPrintText(4, "Battery: 100%%");
								else if(tSl900a.fBattLevel > SL900_LOWBAT)
										vPrintText(4, "Battery: %d%%", ((u8)((tSl900a.fBattLevel - SL900_LOWBAT) * 8) + 1) * 20);
								else
										vPrintText(4, "Battery: 0%%");

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_SUBMENU_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								bLoopMenu = LOOP_DISABLE;

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigDoneOff();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 4
				}
				break;

		// ================================================================= // Main page 1
		case MAIN_START_LOGGING:
			
				switch(display_menu.bSubIdx)
				{
				// ------------------------------------------------------------- // Sub page 0
				case SUB_PAGE_MAIN:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								MenuUpdate(MAIN_START_LOGGING);

								bLoopMenu = LOOP_MAIN_CHECK;
								ulCheck_Timeout = ADC_REC_TIME_MS;
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								if(--display_menu.bMainIdx == 0)
										display_menu.bMainIdx = g_tRegisters.s2.tMenu.ucTotalNum;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								if(++display_menu.bMainIdx > g_tRegisters.s2.tMenu.ucTotalNum)
										display_menu.bMainIdx = SUB_PAGE_MAIN;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Start processing");
								vPrintText(2, "logger...");
								vPrintText(3, "");
								vPrintText(4, "");

								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								task_list.bTaskBuf[2] = KEY_TASK_STLOGGERCHECK;
								task_list.bTaskBuf[3] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
								HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
								bLoopMenu = LOOP_DISABLE;

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_KEY_TRIG;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_3;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_CHARGER_CHANGE)
						{
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								CheckBatStatus();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 1
				case SUB_PAGE_1:

						if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								task_list.bTaskBuf[2] = KEY_TASK_STARTLOGGER;
								task_list.bTaskBuf[3] = KEY_TASK_SETPASSWORD;
								task_list.bTaskBuf[4] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								DoneResponse();

								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_3;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 2
				case SUB_PAGE_2:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Logging activated");
								vPrintText(2, "");
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigDoneOff();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 3
				case SUB_PAGE_3:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_TAG_NO_POWER)
								{
										vPrintText(1, "Battery is too");
										vPrintText(2, "low for use");
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_LOG_ACTIVATED)
								{
										vPrintText(1, "Tag is already");
										vPrintText(2, "activated");
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_NOT_EMPTY)
								{
										vPrintText(1, "Tag is not");
										vPrintText(2, "empty");
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_OPEN_SETPW)
								{
										vPrintText(1, "Lock memory");
										vPrintText(2, "failed");
								}
								else
								{
										vPrintText(1, "Start logger");
										vPrintText(2, "failed");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigErrorOff();

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				}
				break;

		// ================================================================= // Main page 2
		case MAIN_STOP_LOGGING:

				switch(display_menu.bSubIdx)
				{
				// ------------------------------------------------------------- // Sub page 0
				case SUB_PAGE_MAIN:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								MenuUpdate(MAIN_STOP_LOGGING);

								bLoopMenu = LOOP_MAIN_CHECK;
								ulCheck_Timeout = ADC_REC_TIME_MS;
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								if(--display_menu.bMainIdx == 0)
										display_menu.bMainIdx = g_tRegisters.s2.tMenu.ucTotalNum;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								if(++display_menu.bMainIdx > g_tRegisters.s2.tMenu.ucTotalNum)
										display_menu.bMainIdx = SUB_PAGE_MAIN;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Start processing");
								vPrintText(2, "logger...");
								vPrintText(3, "");
								vPrintText(4, "");

								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								task_list.bTaskBuf[2] = KEY_TASK_STOPLOGGER;
								task_list.bTaskBuf[3] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
								HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
								bLoopMenu = LOOP_DISABLE;

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_CHARGER_CHANGE)
						{
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								CheckBatStatus();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 1
				case SUB_PAGE_1:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_AUR700_OPEN_SETPW)
								{
										vPrintText(1, "Lock memory");
										vPrintText(2, "failed");
								}
								else
								{
										vPrintText(1, "Stop logging");
										vPrintText(2, "failed");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigErrorOff();

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 2
				case SUB_PAGE_2:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Stop logging");
								vPrintText(2, "Succeed");
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_WAITNEXT_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_3;
								display_menu.bEvent = EVENT_KEY_TRIG;
						}
						break;
				// ------------------------------------------------------------- // Sub page 3
				case SUB_PAGE_3:

						if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Read logger data");
								vPrintText(2, "");
								vPrintText(3, "");
								vPrintText(4, "");

								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								// need to separate "KEY_TASK_RDLOGGERDATA" from "KEY_TASK_STOPLOGGER" and "KEY_TASK_SETPASSWORD" to prevent a bug
								task_list.bTaskBuf[2] = KEY_TASK_RDLOGGERDATA;
								task_list.bTaskBuf[3] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								display_menu.bSubIdx = SUB_PAGE_5;
								display_menu.bEvent = EVENT_KEY_TRIG;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_4;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 4
				case SUB_PAGE_4:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_AUR700_IS_FULL)
								{
										vPrintText(1, "Memory full!");
										vPrintText(2, "(%d/%d)", uTblLogNum, MAX_LOGGER_NUM);
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_IS_EMPTY)
								{
										vPrintText(1, "Logger is empty");
										vPrintText(2, "");
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_ERASE_FAIL)
								{
										vPrintText(1, "Erase failed");
										vPrintText(2, "");
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_OPEN_SETPW)
								{
										vPrintText(1, "Lock memory");
										vPrintText(2, "failed");
								}
								else
								{
										vPrintText(1, "Read data failed");
										vPrintText(2, "");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigErrorOff();

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 6
				case SUB_PAGE_5:
						if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								task_list.bTaskBuf[2] = KEY_TASK_SETPASSWORD;
								task_list.bTaskBuf[3] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								DoneResponse();

								display_menu.bSubIdx = SUB_PAGE_6;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_4;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 6
				case SUB_PAGE_6:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Read data OK");
								vPrintText(2, "Memory: %d/%d", uTblLogNum, MAX_LOGGER_NUM);
								if(tSl900a.bTLCStatus == 0)
								{
										vPrintText(3, "Alarm: No");
										vPrintText(4, "");
								}
								else
								{
										if(tSl900a.bTLCStatus & 0x08)
												bShowChar[0] = '*';
										else
												bShowChar[0] = ' ';
										if(tSl900a.bTLCStatus & 0x04)
												bShowChar[1] = '*';
										else
												bShowChar[1] = ' ';
										if(tSl900a.bTLCStatus & 0x02)
												bShowChar[2] = '*';
										else
												bShowChar[2] = ' ';
										if(tSl900a.bTLCStatus & 0x01)
												bShowChar[3] = '*';
										else
												bShowChar[3] = ' ';

										vPrintText(3, "Alarm: Yes");
										vPrintText(4, "(EU%c/U%c/L%c/EL%c)", bShowChar[0], bShowChar[1], bShowChar[2], bShowChar[3]);
								}

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigDoneOff();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				}
				break;

		// ================================================================= // Main page 3
		case MAIN_WRITE_LOCATION:
			
				switch(display_menu.bSubIdx)
				{
				// ------------------------------------------------------------- // Sub page 0
				case SUB_PAGE_MAIN:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								MenuUpdate(MAIN_WRITE_LOCATION);

								bLoopMenu = LOOP_MAIN_CHECK;
								ulCheck_Timeout = ADC_REC_TIME_MS;
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								if(--display_menu.bMainIdx == 0)
										display_menu.bMainIdx = g_tRegisters.s2.tMenu.ucTotalNum;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								if(++display_menu.bMainIdx > g_tRegisters.s2.tMenu.ucTotalNum)
										display_menu.bMainIdx = SUB_PAGE_MAIN;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								if((g_tRegisters.s2.ucLocationID[0] | g_tRegisters.s2.ucLocationID[1]) == 0)
								{

										HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
										HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
										bLoopMenu = LOOP_DISABLE;
								}
								else
								{
										vPrintText(1, "Start processing");
										vPrintText(2, "logger...");
										vPrintText(3, "");
										vPrintText(4, "");

										task_list.bTaskIdx = 0;
										task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
										task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
										task_list.bTaskBuf[2] = KEY_TASK_RDLOCATIONID;
										task_list.bTaskBuf[3] = B2E_TASK_NONE;
										bLoopTask = task_list.bTaskBuf[0];

										ulLoop_Timeout = 2000;
										ulLoop_Tick = HAL_GetTick();

										HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
										HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
										bLoopMenu = LOOP_DISABLE;

										display_menu.bEvent = EVENT_MENU_NONE;
								}
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_KEY_TRIG;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_CHARGER_CHANGE)
						{
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								CheckBatStatus();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 1
				case SUB_PAGE_1:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_AUR700_IS_FULL)
								{
										vPrintText(1, "ID full!");
										vPrintText(2, "(%d/%d)", tSl900a.bLocIDNum, MAX_LOCATION_NUM);
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_OPEN_SETPW)
								{
										vPrintText(1, "Lock memory");
										vPrintText(2, "failed");
								}
								else
								{
										vPrintText(1, "Write Location");
										vPrintText(2, "failed");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigErrorOff();

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 2
				case SUB_PAGE_2:

						if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								task_list.bTaskBuf[2] = KEY_TASK_WRLOCATIONID;
								task_list.bTaskBuf[3] = KEY_TASK_SETPASSWORD;
								task_list.bTaskBuf[4] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								DoneResponse();

								display_menu.bSubIdx = SUB_PAGE_3;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_4;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 3
				case SUB_PAGE_3:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Write Location");
								vPrintText(2, "Succeed");
								vPrintText(3, "Locations: %d/%d", tSl900a.bLocIDNum, MAX_LOCATION_NUM);
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigDoneOff();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 4
				case SUB_PAGE_4:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_AUR700_OPEN_SETPW)
								{
										vPrintText(1, "Lock memory");
										vPrintText(2, "failed");
								}
								else
								{
										vPrintText(1, "Write Location");
										vPrintText(2, "failed");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigErrorOff();

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				}
				break;

		// ================================================================= // Main page 4
		case MAIN_READ_DEVICEINFO:
			
				switch(display_menu.bSubIdx)
				{
				// ------------------------------------------------------------- // Sub page 0
				case SUB_PAGE_MAIN:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								MenuUpdate(MAIN_READ_DEVICEINFO);

								bLoopMenu = LOOP_MAIN_CHECK;
								ulCheck_Timeout = ADC_REC_TIME_MS;
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								if(--display_menu.bMainIdx == 0)
										display_menu.bMainIdx = g_tRegisters.s2.tMenu.ucTotalNum;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								if(++display_menu.bMainIdx > g_tRegisters.s2.tMenu.ucTotalNum)
										display_menu.bMainIdx = SUB_PAGE_MAIN;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
								HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
								bLoopMenu = LOOP_DISABLE;

								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_CHARGER_CHANGE)
						{
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								CheckBatStatus();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 1
				case SUB_PAGE_1:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								uTblLogNum = ReadLoggerCount();
								RTC_TimeDateShow(bShowTime, bShowDate);

								vPrintText(1, "ID: %05d", (g_tRegisters.s2.ucDeviceID[0] + g_tRegisters.s2.ucDeviceID[1] * 256));
								vPrintText(2, "Loggers: %d/%d", uTblLogNum, MAX_LOGGER_NUM);
								vPrintText(3, (char*)bShowDate);
								vPrintText(4, (char*)bShowTime);

								bLoopMenu = LOOP_SUB_CHECK;
								ulCheck_Timeout = 500;		// too short period will let MenuDisplay() be entered too often, then the key operation's sensitivity becomes lower
								ulCheck_Tick = HAL_GetTick();
								ulMenu_Timeout = OLED_SUBMENU_TIME_MS;			// used to turn off the submenu
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								RTC_TimeDateShow(bShowTime, bShowDate);

								vPrintText(3, (char*)bShowDate);
								vPrintText(4, (char*)bShowTime);

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								bLoopMenu = LOOP_DISABLE;

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 2
				case SUB_PAGE_2:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(ADC_PercentGet() == 255)
										vPrintText(1, "Battery charging");
								else
										vPrintText(1, "Battery: %3d%%", ADC_PercentGet());
								if(g_tRegisters.s2.ucBuzzEnable)
										vPrintText(2, "Buzzer: ON");
								else
										vPrintText(2, "Buzzer: OFF");
								if(g_tRegisters.s2.ucVibrEnable)
										vPrintText(3, "Vibration: ON");
								else
										vPrintText(3, "Vibration: OFF");
								if(g_tRegisters.s2.ucRFStrg == RF_STRENGTH_LOW)
										vPrintText(4, "RF Strg.: Low");
								else if(g_tRegisters.s2.ucRFStrg == RF_STRENGTH_MED)
										vPrintText(4, "RF Strg.: Med");
								else
										vPrintText(4, "RF Strg.: High");

								bLoopMenu = LOOP_SUB_CHECK;
								ulCheck_Timeout = ADC_REC_TIME_MS;		// don't need fast update
								ulCheck_Tick = HAL_GetTick();
								ulMenu_Timeout = OLED_SUBMENU_TIME_MS;			// used to turn off the submenu
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_CHARGER_CHANGE)
						{
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								if(ADC_PercentGet() == 255)
										vPrintText(1, "Battery charging");
								else
										vPrintText(1, "Battery: %3d%%", ADC_PercentGet());

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								bLoopMenu = LOOP_DISABLE;

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								display_menu.bSubIdx = SUB_PAGE_3;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 3
				case SUB_PAGE_3:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Main:");
								vPrintText(2, "%s", sSTM_FWVer);

								if((sBLE_FWVer[0] | sBLE_FWVer[1] | sBLE_FWVer[2]) == 0)
								{
										vPrintText(3, "Please");
										vPrintText(4, "Restart Device");
								}
								else
								{
										vPrintText(3, "BLE:");
										vPrintText(4, "PGM-Txxxx V%d.%dR%d", sBLE_FWVer[0], sBLE_FWVer[1], sBLE_FWVer[2]);
								}

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_SUBMENU_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								bLoopMenu = LOOP_DISABLE;

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 4
				}
				break;

		// ================================================================= // Main page 5
		case MAIN_BARCODE_LOGGING:

				switch(display_menu.bSubIdx)
				{
				// ------------------------------------------------------------- // Sub page 0
				case SUB_PAGE_MAIN:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								MenuUpdate(MAIN_BARCODE_LOGGING);

								HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_SET);					// BPWR off

								bLoopMenu = LOOP_MAIN_CHECK;
								ulCheck_Timeout = ADC_REC_TIME_MS;
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								if(--display_menu.bMainIdx == 0)
										display_menu.bMainIdx = g_tRegisters.s2.tMenu.ucTotalNum;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								if(++display_menu.bMainIdx > g_tRegisters.s2.tMenu.ucTotalNum)
										display_menu.bMainIdx = SUB_PAGE_MAIN;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Detecting");
								vPrintText(2, "Barcode...");
								vPrintText(3, "");
								vPrintText(4, "");

								HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_RESET);				// BPWR on

								if(InitBarcReader() == ERR_BARC_OK)
								{
										task_list.bTaskIdx = 0;
										task_list.bTaskBuf[0] = KEY_TASK_READBARCODE;
										task_list.bTaskBuf[1] = B2E_TASK_NONE;
										bLoopTask = task_list.bTaskBuf[0];

										ulLoop_Timeout = 20000;
										ulLoop_Tick = HAL_GetTick();

										tSl900a.bBarcNum = 0;		// Reset Barcode number

										g_bBarcTempLen = 0;
										TriggerBarcoder();

										HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
										HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
										bLoopMenu = LOOP_DISABLE;

										display_menu.bEvent = EVENT_MENU_NONE;
								}
								else
								{
										HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
										HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
										bLoopMenu = LOOP_DISABLE;

										display_menu.bSubIdx = SUB_PAGE_MAIN;
										display_menu.bEvent = EVENT_MENU_UPDATE;		// to let BPWR be off then on again
								}
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								DoneResponse();

								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								UntriggerBarcoder();

								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_CHARGER_CHANGE)
						{
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								CheckBatStatus();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 1
				case SUB_PAGE_1:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_AUR700_IS_FULL)
								{
										vPrintText(1, "Barcode full!");
										vPrintText(2, "(%d/%d)", tSl900a.bBarcNum, MAX_BARCODE_NUM);
								}
								else
								{
										vPrintText(1, "Read Barcode");
										vPrintText(2, "failed");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 2
				case SUB_PAGE_2:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "BarCode: %d", tSl900a.bBarcNum);
								if(tSl900a.bBarcNum > 0)
										vPrintText(2, "%s", bBarcLastStr);
								else
										vPrintText(2, "");
								vPrintText(3, "");
								vPrintText(4, "");

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								if(tSl900a.bBarcNum >= MAX_BARCODE_NUM)
								{
										display_menu.bSubIdx = SUB_PAGE_3;
										display_menu.bEvent = EVENT_LPTASK_ERROR;
										display_menu.bData[0] = (u8)ERR_AUR700_IS_FULL;
								}
								else
								{
										display_menu.bSubIdx = SUB_PAGE_3;
										display_menu.bEvent = EVENT_KEY_TRIG;
								}
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								display_menu.bSubIdx = SUB_PAGE_4;
								display_menu.bEvent = EVENT_KEY_TRIG;
						}
						break;
				// ------------------------------------------------------------- // Sub page 3
				case SUB_PAGE_3:

						if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Detecting");
								vPrintText(2, "Barcode...");
								vPrintText(3, "");
								vPrintText(4, "");

								if(InitBarcReader() == ERR_BARC_OK)		// Init can wake up barcode reader from Sleep/Hibernate mode
								{
										task_list.bTaskIdx = 0;
										task_list.bTaskBuf[0] = KEY_TASK_READBARCODE;
										task_list.bTaskBuf[1] = B2E_TASK_NONE;
										bLoopTask = task_list.bTaskBuf[0];

										ulLoop_Timeout = 20000;
										ulLoop_Tick = HAL_GetTick();

										g_bBarcTempLen = 0;
										TriggerBarcoder();

										display_menu.bEvent = EVENT_MENU_NONE;
								}
								else
								{
										display_menu.bSubIdx = SUB_PAGE_MAIN;
										display_menu.bEvent = EVENT_MENU_UPDATE;		// to let BPWR be off then on again
								}
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								DoneResponse();

								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								UntriggerBarcoder();

								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 4
				case SUB_PAGE_4:

						if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Writing Barcode");
								vPrintText(2, "");
								vPrintText(3, "");
								vPrintText(4, "");

								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								task_list.bTaskBuf[2] = KEY_TASK_STLOGGERCHECK;
								task_list.bTaskBuf[3] = KEY_TASK_WRITEBARCODE;
								task_list.bTaskBuf[4] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								display_menu.bSubIdx = SUB_PAGE_7;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_11;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 7
				case SUB_PAGE_7:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Write Barcode");
								vPrintText(2, "Succeed");
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_WAITNEXT_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_8;
								display_menu.bEvent = EVENT_KEY_TRIG;
						}
						break;
				// ------------------------------------------------------------- // Sub page 8
				case SUB_PAGE_8:

						if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Start Logging");
								vPrintText(2, "");
								vPrintText(3, "");
								vPrintText(4, "");

								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_INVENTORY;
								task_list.bTaskBuf[1] = KEY_TASK_OPENAREA;
								task_list.bTaskBuf[2] = KEY_TASK_STARTLOGGER;
								task_list.bTaskBuf[3] = KEY_TASK_SETPASSWORD;
								task_list.bTaskBuf[4] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								DoneResponse();

								display_menu.bSubIdx = SUB_PAGE_10;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_13;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 10
				case SUB_PAGE_10:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Logging activated");
								vPrintText(2, "");
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 11
				case SUB_PAGE_11:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_TAG_NO_POWER)
								{
										vPrintText(1, "Battery is too");
										vPrintText(2, "low for use");
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_LOG_ACTIVATED)
								{
										vPrintText(1, "Tag is already");
										vPrintText(2, "activated");
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_NOT_EMPTY)
								{
										vPrintText(1, "Tag is not");
										vPrintText(2, "empty");
								}
								else if(display_menu.bData[0] == (u8)ERR_AUR700_OPEN_SETPW)
								{
										vPrintText(1, "Lock memory");
										vPrintText(2, "failed");
								}
								else
								{
										vPrintText(1, "Write Barcode");
										vPrintText(2, "data failed");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 13
				case SUB_PAGE_13:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								if(display_menu.bData[0] == (u8)ERR_AUR700_OPEN_SETPW)
								{
										vPrintText(1, "Lock memory");
										vPrintText(2, "failed");
								}
								else
								{
										vPrintText(1, "Start logger");
										vPrintText(2, "failed");
								}
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				}
				break;

		// ================================================================= // Main page 15
		case MAIN_DEBUG_TEST:

				switch(display_menu.bSubIdx)
				{
				// ------------------------------------------------------------- // Sub page 0
				case SUB_PAGE_MAIN:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								MenuUpdate(MAIN_DEBUG_TEST);

								bLoopMenu = LOOP_MAIN_CHECK;
								ulCheck_Timeout = ADC_REC_TIME_MS;
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_KEY_LEFT)
						{
								if(--display_menu.bMainIdx == 0)
										display_menu.bMainIdx = g_tRegisters.s2.tMenu.ucTotalNum;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_RIGHT)
						{
								if(++display_menu.bMainIdx > g_tRegisters.s2.tMenu.ucTotalNum)
										display_menu.bMainIdx = SUB_PAGE_MAIN;

								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								vPrintText(1, "Start processing");
								vPrintText(2, "the test...");
								vPrintText(3, "");
								vPrintText(4, "");

								task_list.bTaskIdx = 0;
								task_list.bTaskBuf[0] = KEY_TASK_DEBUGTEST;
								task_list.bTaskBuf[1] = B2E_TASK_NONE;
								bLoopTask = task_list.bTaskBuf[0];

								ulLoop_Timeout = 2000;
								ulLoop_Tick = HAL_GetTick();

								HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);					// RED off
								HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET); 				// GRN off
								bLoopMenu = LOOP_DISABLE;

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_DONE)
						{
								DoneResponse();

								display_menu.bSubIdx = SUB_PAGE_2;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_LPTASK_ERROR)
						{
								ErrorResponse();

								display_menu.bSubIdx = SUB_PAGE_1;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_CHARGER_CHANGE)
						{
								ulCheck_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_CHECK;
						}
						else if(display_menu.bEvent == EVENT_MENU_CHECK)
						{
								CheckBatStatus();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 1
				case SUB_PAGE_1:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Testing failed");
								vPrintText(2, "Error code is %x", display_menu.bData[0]);
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_FAIL;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigErrorOff();

								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 2
				case SUB_PAGE_2:

						if(display_menu.bEvent == EVENT_MENU_UPDATE)		// Menu change event
						{
								vPrintText(1, "Testing success");
								vPrintText(2, "");
								vPrintText(3, "");
								vPrintText(4, "");

								bLoopMenu = LOOP_EN_DONE;
								ulMenu_Timeout = OLED_MESSAGE_TIME_MS;
								ulMenu_Tick = HAL_GetTick();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_KEY_TRIG)
						{
								TrigDoneOff();

								display_menu.bEvent = EVENT_MENU_NONE;
						}
						else if(display_menu.bEvent == EVENT_TIMEOUT_BACK)
						{
								display_menu.bSubIdx = SUB_PAGE_MAIN;
								display_menu.bEvent = EVENT_MENU_UPDATE;
						}
						break;
				// ------------------------------------------------------------- // Sub page 3
				}
		}

}
#endif



