/**
  ******************************************************************************
  * @file    cmd_protocol_gnet.c
  * @author  Albert
  * @version V1.0.0
  * @date    31-March-2018
  * @brief   commands protocol sub-routine for GNetPlus
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "cmd_protocol_gnet.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CmdProtocol_t cmd_parser;
RepProtocol_t rep_packer;

TblInformation_t tbl_info[TBL_CNT_MAX];

u8 bPackBuf[REP_MAX_DATA_SIZE];
u8 bCurrQueryTbl = 0xFF;

u32 ulStartFreq, ulIncFreq, ulEndFreq;
u32 ulStartFreq_2, ulIncFreq_2, ulEndFreq_2;

const char sSTM_FWVer[] = "PGM-Txxxx V1.0R0";
static const char sEcho_Message[] = "Hello AUR700";
char sBLE_FWVer[3] = {0};


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void FreqListGenerate(u32 ulListStart, u32 ulListEnd, u32 ulListInc, u8 bIndexStart, u8 bSecondZone);
void GetFrequencyReflectedPower(u32 ulFrequency, u8* pbBuffer);


/******************************************************************************/
/**
  * @brief  
  * @param  
  * @retval 
  */
static u16 CRC16_Calculate(u8* pData, u16 data_len, u16 first_crc)
{
    u16 i;
    u8 j;

    volatile u16 CrcValue;

    CrcValue = first_crc;
    i = 0;

		do{

				CrcValue ^= pData[i];
        j = 0x80;

				do{
            if((CrcValue & 0x01) == 1)
            {
                CrcValue >>= 1;
                CrcValue ^= DEF_CRC_POLYNOM;
            }
            else
                CrcValue = (CrcValue>> 1);
            j >>= 1;
        }while(j);

        i++;

    }while(i < data_len);

    return CrcValue;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u8 CRC16_Verify(u8* check_data, u16 check_len)
{
    u16 ulDataCRC;

    ulDataCRC = check_data[check_len - 2];
    ulDataCRC = ((ulDataCRC << 8) | (check_data[check_len - 1]));

		return !(ulDataCRC == CRC16_Calculate(check_data, (check_len - 2), DEF_CRC_PRESET));
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void CRC16_Append(u8* calculated_data, u16 calculated_len)
{
    volatile u16 crc;

    crc = CRC16_Calculate(calculated_data, calculated_len, DEF_CRC_PRESET);

		calculated_data[calculated_len + 1] = (u8)(crc & 0xFF);
    calculated_data[calculated_len] = (u8)((crc >> 8) & 0xFF);
}


//----------------------------------------------------------------------------//
/**
  * @brief  
  * @param  
  * @retval None
  */
void CommandInit(CmdProtocol_t* ptCommand)
{
		ptCommand->uLen = 0;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void CommandParser(u8 bData)
{
		u8 bErrorMsg;

		if(cmd_parser.uLen == 0)
		{
				if(bData == CMD_HEADER_SOH)
						cmd_parser.bBuffer[cmd_parser.uLen++] = bData;
		}
		else
		{
				cmd_parser.bBuffer[cmd_parser.uLen++] = bData;

				if(cmd_parser.uLen >= CMD_BEFORE_DATA_SIZE)
				{
						if(cmd_parser.uLen >= (cmd_parser.tFormat.bLength + CMD_FULL_NO_DATA_LEN))
						{
								// check CRC
								if(CRC16_Verify(&cmd_parser.tFormat.bAddress, (cmd_parser.tFormat.bLength + (CMD_FULL_NO_DATA_LEN - CMD_HEADER_LEN))) == 0)
								{
										CommandExecute();
								}
								else
								{
										bErrorMsg = AUR700_ERROR_CRC;
										ResponseNakData(&bErrorMsg, 1);
								}

								cmd_parser.uLen = 0;
						}
				}
		}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void CommandPacker(u8 bRepCode, const u8* pbArgument, u8 bArgumentLength)
{
		u32 ulPacker_Tick;
		USBD_CDC_HandleTypeDef *hcdc = (&UsbdHandle)->pClassData;

		ulPacker_Tick = HAL_GetTick();

		// To prevent previous rep_packer from being inserted before it is all read out by UART/USB
		if(g_CmdSource == CMD_SOURCE_UART)
				while((Uart2Handle.TxXferCount > 0) && !(IsTimeout_ms(ulPacker_Tick, 1000)));
		else if(g_CmdSource == CMD_SOURCE_USB)
				while((hcdc->TxState == 1) && !(IsTimeout_ms(ulPacker_Tick, 1000)));

		// Clear previous rep_packer data
    memset((void*) &rep_packer, 0x00, sizeof(rep_packer));

    if(pbArgument == NULL)
        bArgumentLength = 0;

    rep_packer.tFormat.bPreamble = CMD_HEADER_SOH;
		rep_packer.tFormat.bAddress = 0;
    rep_packer.tFormat.bRepCode = bRepCode;
    rep_packer.tFormat.bLength = bArgumentLength;

    if(bArgumentLength > 0)
        memcpy(rep_packer.tFormat.bPayload, pbArgument, bArgumentLength);

    CRC16_Append(&rep_packer.tFormat.bAddress, (bArgumentLength + CMD_ADDR_LENGTH_LEN));

    rep_packer.uLen = REP_FULL_NO_DATA_LEN + bArgumentLength;

		// To wait previous rep_packer for being all sent out by UART/USB
		if(g_CmdSource == CMD_SOURCE_UART)
				while(HAL_BUSY == HAL_UART_Transmit_IT(&Uart2Handle, rep_packer.bBuffer, rep_packer.uLen));
		else if(g_CmdSource == CMD_SOURCE_USB)
		{
				USBD_CDC_SetTxBuffer(&UsbdHandle, rep_packer.bBuffer, rep_packer.uLen);
				while(USBD_BUSY == USBD_CDC_TransmitPacket(&UsbdHandle));
		}
}


//----------------------------------------------------------------------------//
/**
  * @brief  
  * @param  None
  * @retval None
  */
void ResponseAck(void)
{
		CommandPacker(CMD_REPLY_ACK, NULL, 0);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ResponseAckData(const u8* pbData, u8 bDataLength)
{
    CommandPacker(CMD_REPLY_ACK, pbData, bDataLength);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void ResponseNak(void)
{
    CommandPacker(CMD_REPLY_NAK, NULL, 0);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ResponseNakData(const u8* pbData, u8 bDataLength)
{
    CommandPacker(CMD_REPLY_NAK, pbData, bDataLength);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ResponseEvent(const u8* pbData, u8 bDataLength)
{
    CommandPacker(CMD_REPLY_EVENT, pbData, bDataLength);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ResponseData(u8 bCmdCode, const u8* pbData, u8 bDataLength)
{
    CommandPacker(bCmdCode, pbData, bDataLength);
}


//----------------------------------------------------------------------------//
/**
  * @brief  
  * @param  
  * @retval None
  */
void TimeOutHandler(u8 bCmdTask)
{
		if((bCmdTask == CMD_TASK_RDPRESSBTN) || (bCmdTask == CMD_TASK_READEPCTID) || (bCmdTask == CMD_TASK_READBARCODE))
		{
				if(bCmdTask == CMD_TASK_READBARCODE)
				{
						UntriggerBarcoder();

						HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_SET);					// BPWR off
				}

				bPackBuf[0] = AUR700_EVENT_ERROR;
				bPackBuf[1] = ERROR_EVENT_TIMEOUT;

				ResponseEvent(bPackBuf, 2);
		}
		else
		{
				bPackBuf[0] = AUR700_ERROR_FAIL;
				ResponseNakData(bPackBuf, 1);
		}
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void CommandExecute(void)
{
		s8 cResult = ERR_NONE;
		u8 i, bMessageLen, bMessageBuf[18];
		u8 bData_Buf[30];
		u16 uData_Len = 0, uRemain_Len, uData_Addr;
    RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;

		switch(cmd_parser.tFormat.bCmdCode)
		{
		case AUR_FW_VERSION:

				ResponseAckData((u8*)sSTM_FWVer, 16);

				cResult = ERR_CMD_DONT_RESPONSE;
				break;

		case AUR_DO_REBOOT:

				ResponseAck();								// to be sent here before being reset
				HAL_Delay(100);								// waiting response to be sent out

				ResetAURDevice();

				cResult = ERR_UNKNOWN;				// unknown error happened if code executed here
				break;

		case AUR_GOTO_ISP:

				HAL_FLASH_Unlock();
				HAL_FLASH_Program(TYPEPROGRAM_WORD, APP_START_ADDRESS-12, 0x55);

				ResponseAck();								// to be sent here before being reset
				HAL_Delay(100);								// waiting response to be sent out

				ResetAURDevice();

				cResult = ERR_UNKNOWN;				// unknown error happened if code executed here
				break;

		case AUR_SET_SETTING:

				cResult = ERR_NONE;

				switch (cmd_parser.tFormat.bPayload[0])
				{
				// Application settings
				case SETTING_DEVICE_ID:

            g_tRegisters.s2.ucDeviceID[0] = cmd_parser.tFormat.bPayload[1];
            g_tRegisters.s2.ucDeviceID[1] = cmd_parser.tFormat.bPayload[2];
						break;

				case SETTING_VIBRATION:

						if(cmd_parser.tFormat.bPayload[1] == VIBR_STATE_DIS)
								g_tRegisters.s2.ucVibrEnable = VIBR_STATE_DIS;
						else if(cmd_parser.tFormat.bPayload[1] == VIBR_STATE_EN)
								g_tRegisters.s2.ucVibrEnable = VIBR_STATE_EN;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				case SETTING_BUZZER:

						if(cmd_parser.tFormat.bPayload[1] == BUZZ_STATE_DIS)
								g_tRegisters.s2.ucBuzzEnable = BUZZ_STATE_DIS;
						else if(cmd_parser.tFormat.bPayload[1] == BUZZ_STATE_EN)
								g_tRegisters.s2.ucBuzzEnable = BUZZ_STATE_EN;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				case SETTING_RF_STRENGTH:

						if(cmd_parser.tFormat.bPayload[1] == RF_STRENGTH_LOW)
								g_tRegisters.s2.ucRFStrg = RF_STRENGTH_LOW;
						else if(cmd_parser.tFormat.bPayload[1] == RF_STRENGTH_MED)
								g_tRegisters.s2.ucRFStrg = RF_STRENGTH_MED;
						else if(cmd_parser.tFormat.bPayload[1] == RF_STRENGTH_HI)
								g_tRegisters.s2.ucRFStrg = RF_STRENGTH_HI;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				case SETTING_SCAN_TIMEOUT:

						if((cmd_parser.tFormat.bPayload[1] >= MIN_SCAN_TIME) && (cmd_parser.tFormat.bPayload[1] <= MAX_SCAN_TIME))
								g_tRegisters.s2.ulScanTagTimeout = cmd_parser.tFormat.bPayload[1] * 1000;		// sec to ms
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				case SETTING_STANDBY_TIMEOUT:

						if((((cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]) >= MIN_STANDBY_TIME) && (((cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]) <= MAX_STANDBY_TIME))
						{
								g_tRegisters.s2.ulEnterIdleTime = ((cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]) * 1000;		// sec to ms

								ulIdle_Timeout = g_tRegisters.s2.ulEnterIdleTime;
								ulIdle_Tick = HAL_GetTick();
						}
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				case SETTING_RTC_DATE:

            g_tRegisters.s2.tRtc.year = ((cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]);
            g_tRegisters.s2.tRtc.month = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tRtc.date = cmd_parser.tFormat.bPayload[4];
            g_tRegisters.s2.tRtc.week = cmd_parser.tFormat.bPayload[5];

						MX_RTC_Init();
						break;

				case SETTING_RTC_TIME:

            g_tRegisters.s2.tRtc.hour = cmd_parser.tFormat.bPayload[1];
            g_tRegisters.s2.tRtc.minute = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tRtc.second = cmd_parser.tFormat.bPayload[3];

						MX_RTC_Init();
						break;

				case SETTING_USER_IDENTIFY:

						if(cmd_parser.tFormat.bPayload[1] == USERID_BY_NONE)
								g_tRegisters.s2.ucUserIdentify = USERID_BY_NONE;
						else if(cmd_parser.tFormat.bPayload[1] == USERID_BY_RFID)
								g_tRegisters.s2.ucUserIdentify = USERID_BY_RFID;
						else if(cmd_parser.tFormat.bPayload[1] == USERID_BY_BARC)
								g_tRegisters.s2.ucUserIdentify = USERID_BY_BARC;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				case SETTING_MENU_SELECT:

						g_tRegisters.s2.tMenu.ucTotalNum = cmd_parser.tFormat.bPayload[1];

            for(i = 0; i < g_tRegisters.s2.tMenu.ucTotalNum; i++)
                g_tRegisters.s2.tMenu.ucItem[i] = cmd_parser.tFormat.bPayload[2 + i];

						// prevent the empty menu display
						if(g_tRegisters.s2.tMenu.ucTotalNum == 0)
						{
								g_tRegisters.s2.tMenu.ucTotalNum = 1;
								g_tRegisters.s2.tMenu.ucItem[0] = MAIN_READ_LOGSTATUS;
						}

						// reset the menu display
						display_menu.bMainIdx = SUB_PAGE_MAIN;
						display_menu.bSubIdx = SUB_PAGE_MAIN;
						display_menu.bEvent = EVENT_MENU_UPDATE;
						break;

				case SETTING_COMPANY_ID:

            g_tRegisters.s2.ucCompanyID = (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1];
						break;

				case SETTING_ERASE_LOGDATA:

						if(cmd_parser.tFormat.bPayload[1] == ERASE_LOG_DIS)
								g_tRegisters.s2.ucEraseLoggerData = ERASE_LOG_DIS;
						else if(cmd_parser.tFormat.bPayload[1] == ERASE_LOG_EN)
								g_tRegisters.s2.ucEraseLoggerData = ERASE_LOG_EN;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				case SETTING_LOCATION_ID:

						g_tRegisters.s2.ucLocationID[0] = cmd_parser.tFormat.bPayload[1];
						g_tRegisters.s2.ucLocationID[1] = cmd_parser.tFormat.bPayload[2];
						break;

				case SETTING_BARCODER_SELECT:

						if(cmd_parser.tFormat.bPayload[1] == BARC_SEL_1D)
								g_tRegisters.s2.uBarcode_Selection = BARC_SEL_1D;
						else if(cmd_parser.tFormat.bPayload[1] == BARC_SEL_2D)
								g_tRegisters.s2.uBarcode_Selection = BARC_SEL_2D;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				case SETTING_LOCK_PROTECT:

						if(cmd_parser.tFormat.bPayload[1] == LOCK_PROT_NONE)
								g_tRegisters.s2.ucLockProtect = LOCK_PROT_NONE;
						else if(cmd_parser.tFormat.bPayload[1] == LOCK_ONLY_USER)
								g_tRegisters.s2.ucLockProtect = LOCK_ONLY_USER;
						else if(cmd_parser.tFormat.bPayload[1] == LOCK_SYS_USER)
								g_tRegisters.s2.ucLockProtect = LOCK_SYS_USER;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;
						break;

				// Frequency settings
				case SETTING_PROFILE_ID:

						g_tRegisters.s2.tFrequencyList.bRegionID = cmd_parser.tFormat.bPayload[1];
						break;

				case SETTING_PROFILE_NAME:

						break;

				case SETTING_START_FREQ:

            g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyHigh = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyMid = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyLow = cmd_parser.tFormat.bPayload[1];

						ulStartFreq = (g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyLow) |
                          (g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyMid << 8) |
                          (g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyHigh << 16);

						break;

				case SETTING_INC_FREQ:

            g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyHigh = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyMid = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyLow = cmd_parser.tFormat.bPayload[1];

						ulIncFreq = (g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyLow) |
                        (g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyMid << 8) |
                        (g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyHigh << 16);

						break;

				case SETTING_END_FREQ:

            ulEndFreq = (cmd_parser.tFormat.bPayload[3] << 16) | (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1];

						g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum = 0;
						g_tRegisters.s2.tFrequencyList.bNumberOfFrequency = 0;

						FreqListGenerate(ulStartFreq, ulEndFreq, ulIncFreq, g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum, 0);

						break;

				case SETTING_2ND_SPEC:

						if(cmd_parser.tFormat.bPayload[1] < 2)
								g_tRegisters.s2.tFrequencyList.ucSecondSpectrum = cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_START2_FREQ:

            g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyHigh = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyMid = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyLow = cmd_parser.tFormat.bPayload[1];

						ulStartFreq_2 = (g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyLow) |
														(g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyMid << 8) |
														(g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyHigh << 16);

						break;

				case SETTING_INC2_FREQ:

            g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyHigh = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyMid = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyLow = cmd_parser.tFormat.bPayload[1];

						ulIncFreq_2 = (g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyLow) |
													(g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyMid << 8) |
													(g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyHigh << 16);

						break;

				case SETTING_END2_FREQ:

            ulEndFreq_2 = (cmd_parser.tFormat.bPayload[3] << 16) | (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1];

						if (g_tRegisters.s2.tFrequencyList.ucSecondSpectrum != 0)
						{
								FreqListGenerate(ulStartFreq_2, ulEndFreq_2, ulIncFreq_2, g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum, 1);
						}

						break;

				case SETTING_LISTEN_TIME:

						if(((cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]) <= MAX_LISTEN_TIME)
								g_tRegisters.s2.tFrequencyList.uListeningTime = (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_IDLE_TIME:

						if(((cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]) <= MAX_IDLE_TIME)
								g_tRegisters.s2.tFrequencyList.uIdleTime = (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_MAX_ALLOCATION:

						if(((cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]) <= MAX_MAX_ALLOCATION)
								g_tRegisters.s2.tFrequencyList.uAllocationTime = (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_RSSI_THRESHOLD:

						if(((s8)cmd_parser.tFormat.bPayload[1] >= MIN_RSSI_THRESHOLD) && ((s8)cmd_parser.tFormat.bPayload[1] <= MAX_RSSI_THRESHOLD))
								g_tRegisters.s2.tFrequencyList.cRSSIThreshold = (s8)cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				// TX/RX power settings
				case SETTING_OUTPUT_POWER:

						if(((s8)cmd_parser.tFormat.bPayload[1] >= MIN_OUTPUT_POWER) && ((s8)cmd_parser.tFormat.bPayload[1] <= MAX_OUTPUT_POWER))
								g_tRegisters.s2.cRFTxPower = (s8)cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_RX_SENSITIVITY:

						if(((s8)cmd_parser.tFormat.bPayload[1] >= GEN2_MOST_SENSITIVITY) && ((s8)cmd_parser.tFormat.bPayload[1] <= GEN2_LEAST_SENSITIVITY))
								g_tRegisters.s2.cRFRxSensitivity = (s8)cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_EXTERNAL_GAIN:

						if(cmd_parser.tFormat.bPayload[1] == POWER_GAIN_0DBM)
								g_tRegisters.s2.bPowerGain = 0;
						else if(cmd_parser.tFormat.bPayload[1] == POWER_GAIN_8DBM)
								g_tRegisters.s2.bPowerGain = PA_GAIN08_SEL;
						else if(cmd_parser.tFormat.bPayload[1] == POWER_GAIN_16DBM)
								g_tRegisters.s2.bPowerGain = PA_GAIN16_SEL;
						else if(cmd_parser.tFormat.bPayload[1] == POWER_GAIN_24DBM)
								g_tRegisters.s2.bPowerGain = PA_GAIN08_SEL | PA_GAIN16_SEL;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				// Gen2 settings
				case SETTING_LINK_FREQ:

						if(cmd_parser.tFormat.bPayload[1] == LINK_FREQ_40)
								g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_40;
						else if(cmd_parser.tFormat.bPayload[1] == LINK_FREQ_80)
								g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_80;
						else if(cmd_parser.tFormat.bPayload[1] == LINK_FREQ_160)
								g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_160;
						else if(cmd_parser.tFormat.bPayload[1] == LINK_FREQ_256)
								g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_256;
						else if(cmd_parser.tFormat.bPayload[1] == LINK_FREQ_320)
								g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_320;
						else if(cmd_parser.tFormat.bPayload[1] == LINK_FREQ_640)
								g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_640;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_RX_DECODE:

						if(cmd_parser.tFormat.bPayload[1] == RX_DECODE_FM0)
								g_tRegisters.s2.bRxDecode = GEN2_COD_FM0;
						else if(cmd_parser.tFormat.bPayload[1] == RX_DECODE_MI2)
								g_tRegisters.s2.bRxDecode = GEN2_COD_MILLER2;
						else if(cmd_parser.tFormat.bPayload[1] == RX_DECODE_MI4)
								g_tRegisters.s2.bRxDecode = GEN2_COD_MILLER4;
						else if(cmd_parser.tFormat.bPayload[1] == RX_DECODE_MI8)
								g_tRegisters.s2.bRxDecode = GEN2_COD_MILLER8;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_Q_BEGIN:

						if((cmd_parser.tFormat.bPayload[1] >= MIN_Q_BEGIN) && (cmd_parser.tFormat.bPayload[1] <= MAX_Q_BEGIN))
								g_tRegisters.s2.bGen2QBegin = cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_SESSION_NUM:

						if(cmd_parser.tFormat.bPayload[1] == SESSION_NUM_0)
								g_tRegisters.s2.bGen2Session = AS3993_REG3C_GEN2_SESSION_S1S0_0;
						else if(cmd_parser.tFormat.bPayload[1] == SESSION_NUM_1)
								g_tRegisters.s2.bGen2Session = AS3993_REG3C_GEN2_SESSION_S1S0_1;
						else if(cmd_parser.tFormat.bPayload[1] == SESSION_NUM_2)
								g_tRegisters.s2.bGen2Session = AS3993_REG3C_GEN2_SESSION_S1S0_2;
						else if(cmd_parser.tFormat.bPayload[1] == SESSION_NUM_3)
								g_tRegisters.s2.bGen2Session = AS3993_REG3C_GEN2_SESSION_S1S0_3;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_PILOT_TONE:

						if(cmd_parser.tFormat.bPayload[1] == PILOT_TONE_NONE)
								g_tRegisters.s2.bGen2Trext = TREXT_OFF;
						else if(cmd_parser.tFormat.bPayload[1] == PILOT_TONE_USED)
								g_tRegisters.s2.bGen2Trext = TREXT_ON;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				// Hardware settings
				case SETTING_BLE_MACADDR:

						// only allow internal command to write the MAC address
						memcpy(g_tRegisters.s2.ucBLEMacAddress, cmd_parser.tFormat.bPayload + 1, 6);

						cResult = ERR_CMD_DONT_RESPONSE;

						break;

				case SETTING_BLE_FWREV:

						// only allow internal command to write the FW revision
						memcpy(sBLE_FWVer, cmd_parser.tFormat.bPayload + 1, 3);

						cResult = ERR_CMD_DONT_RESPONSE;

						break;

				default:

						cResult = ERR_CMD_INCORRECT_PARAM;
				}

				gen2Initialize();

				break;

		case AUR_GET_SETTING:

				cResult = ERR_CMD_DONT_RESPONSE;

				switch (cmd_parser.tFormat.bPayload[0])
				{
				// Application settings
				case SETTING_DEVICE_ID:

            bPackBuf[0] = g_tRegisters.s2.ucDeviceID[0];
            bPackBuf[1] = g_tRegisters.s2.ucDeviceID[1];

						ResponseAckData(bPackBuf, 2);

						break;

				case SETTING_VIBRATION:

						if(g_tRegisters.s2.ucVibrEnable == VIBR_STATE_DIS)
								bPackBuf[0] = VIBR_STATE_DIS;
						else
								bPackBuf[0] = VIBR_STATE_EN;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_BUZZER:

						if(g_tRegisters.s2.ucBuzzEnable == BUZZ_STATE_DIS)
								bPackBuf[0] = BUZZ_STATE_DIS;
						else
								bPackBuf[0] = BUZZ_STATE_EN;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_RF_STRENGTH:

						if(g_tRegisters.s2.ucRFStrg == RF_STRENGTH_LOW)
								bPackBuf[0] = RF_STRENGTH_LOW;
						else if(g_tRegisters.s2.ucRFStrg == RF_STRENGTH_MED)
								bPackBuf[0] = RF_STRENGTH_MED;
						else
								bPackBuf[0] = RF_STRENGTH_HI;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_SCAN_TIMEOUT:

						bPackBuf[0] = g_tRegisters.s2.ulScanTagTimeout / 1000;			// ms to sec

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_STANDBY_TIMEOUT:

						bPackBuf[1]	= ((g_tRegisters.s2.ulEnterIdleTime / 1000) >> 8) & 0xFF;			// ms to sec
						bPackBuf[0]	=  (g_tRegisters.s2.ulEnterIdleTime / 1000) & 0xFF;

						ResponseAckData(bPackBuf, 2);

						break;

				case SETTING_RTC_DATE:

						// You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock
						/* Get the RTC current Time */
						RTC_TimeGet(&stimestructureget);
						/* Get the RTC current Date */
						RTC_DateGet(&sdatestructureget);

						bPackBuf[1] = (sdatestructureget.Year >> 8) & 0xFF;
						bPackBuf[0] =  sdatestructureget.Year & 0xFF;
						bPackBuf[2] =  sdatestructureget.Month;
						bPackBuf[3] =  sdatestructureget.Date;
						bPackBuf[4] =  sdatestructureget.WeekDay;

						ResponseAckData(bPackBuf, 5);

						break;

				case SETTING_RTC_TIME:

						// You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock
						/* Get the RTC current Time */
						RTC_TimeGet(&stimestructureget);
						/* Get the RTC current Date */
						RTC_DateGet(&sdatestructureget);

						bPackBuf[0] =  stimestructureget.Hours;
						bPackBuf[1] =  stimestructureget.Minutes;
						bPackBuf[2] =  stimestructureget.Seconds;

						ResponseAckData(bPackBuf, 3);

						break;

				case SETTING_USER_IDENTIFY:

						if(g_tRegisters.s2.ucUserIdentify == USERID_BY_NONE)
								bPackBuf[0] = USERID_BY_NONE;
						else if(g_tRegisters.s2.ucUserIdentify == USERID_BY_RFID)
								bPackBuf[0] = USERID_BY_RFID;
						else
								bPackBuf[0] = USERID_BY_BARC;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_MENU_SELECT:

						bPackBuf[0] = g_tRegisters.s2.tMenu.ucTotalNum;

            for(i = 0; i < g_tRegisters.s2.tMenu.ucTotalNum; i++)
                bPackBuf[1 + i] = g_tRegisters.s2.tMenu.ucItem[i];

						ResponseAckData(bPackBuf, 1 + g_tRegisters.s2.tMenu.ucTotalNum);

						break;

				case SETTING_COMPANY_ID:

            bPackBuf[1] = (g_tRegisters.s2.ucCompanyID >> 8) & 0xFF;
            bPackBuf[0] =  g_tRegisters.s2.ucCompanyID & 0xFF;

						ResponseAckData(bPackBuf, 2);

						break;

				case SETTING_ERASE_LOGDATA:

						if(g_tRegisters.s2.ucEraseLoggerData == ERASE_LOG_DIS)
								bPackBuf[0] = ERASE_LOG_DIS;
						else
								bPackBuf[0] = ERASE_LOG_EN;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_LOCATION_ID:

						bPackBuf[0] = g_tRegisters.s2.ucLocationID[0];
						bPackBuf[1] = g_tRegisters.s2.ucLocationID[1];

						ResponseAckData(bPackBuf, 2);

						break;

				case SETTING_BARCODER_SELECT:

						if(g_tRegisters.s2.uBarcode_Selection == BARC_SEL_1D)
								bPackBuf[0] = BARC_SEL_1D;
						else
								bPackBuf[0] = BARC_SEL_2D;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_LOCK_PROTECT:

						if(g_tRegisters.s2.ucLockProtect == LOCK_PROT_NONE)
								bPackBuf[0] = LOCK_PROT_NONE;
						else if(g_tRegisters.s2.ucLockProtect == LOCK_ONLY_USER)
								bPackBuf[0] = LOCK_ONLY_USER;
						else
								bPackBuf[0] = LOCK_SYS_USER;

						ResponseAckData(bPackBuf, 1);

						break;

				// Frequency settings
				case SETTING_PROFILE_ID:

						bPackBuf[0] = g_tRegisters.s2.tFrequencyList.bRegionID;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_PROFILE_NAME:

						break;

				case SETTING_START_FREQ:

            bPackBuf[3] = 0;
            bPackBuf[2] = g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyHigh;
            bPackBuf[1] = g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyMid;
            bPackBuf[0] = g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyLow;

						ResponseAckData(bPackBuf, 4);

						break;

				case SETTING_INC_FREQ:

            bPackBuf[3] = 0;
            bPackBuf[2] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyHigh;
            bPackBuf[1] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyMid;
            bPackBuf[0] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyLow;

						ResponseAckData(bPackBuf, 4);

						break;

				case SETTING_END_FREQ:

            bPackBuf[3] = 0;
            bPackBuf[2] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum-1].bFrequencyHigh;
            bPackBuf[1] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum-1].bFrequencyMid;
            bPackBuf[0] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum-1].bFrequencyLow;

						ResponseAckData(bPackBuf, 4);

						break;

				case SETTING_2ND_SPEC:

						bPackBuf[0] = g_tRegisters.s2.tFrequencyList.ucSecondSpectrum;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_START2_FREQ:

            bPackBuf[3] = 0;

						if(g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum < g_tRegisters.s2.tFrequencyList.bNumberOfFrequency)
						{
								bPackBuf[2] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum].bFrequencyHigh;
								bPackBuf[1] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum].bFrequencyMid;
								bPackBuf[0] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum].bFrequencyLow;
						}
						else
						{
								bPackBuf[2] = 0;
								bPackBuf[1] = 0;
								bPackBuf[0] = 0;
						}

						ResponseAckData(bPackBuf, 4);

						break;

				case SETTING_INC2_FREQ:

            bPackBuf[3] = 0;
            bPackBuf[2] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyHigh;
            bPackBuf[1] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyMid;
            bPackBuf[0] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyLow;

						ResponseAckData(bPackBuf, 4);

						break;

				case SETTING_END2_FREQ:

            bPackBuf[3] = 0;

						if(g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum < g_tRegisters.s2.tFrequencyList.bNumberOfFrequency)
						{
								bPackBuf[2] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequency-1].bFrequencyHigh;
								bPackBuf[1] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequency-1].bFrequencyMid;
								bPackBuf[0] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequency-1].bFrequencyLow;
						}
						else
						{
								bPackBuf[2] = 0;
								bPackBuf[1] = 0;
								bPackBuf[0] = 0;
						}

						ResponseAckData(bPackBuf, 4);

						break;

				case SETTING_LISTEN_TIME:

						bPackBuf[1] = (g_tRegisters.s2.tFrequencyList.uListeningTime >> 8) & 0xFF;
						bPackBuf[0] = g_tRegisters.s2.tFrequencyList.uListeningTime & 0xFF;

						ResponseAckData(bPackBuf, 2);

						break;

				case SETTING_IDLE_TIME:

						bPackBuf[1] = (g_tRegisters.s2.tFrequencyList.uIdleTime >> 8) & 0xFF;
						bPackBuf[0] = g_tRegisters.s2.tFrequencyList.uIdleTime & 0xFF;

						ResponseAckData(bPackBuf, 2);

						break;

				case SETTING_MAX_ALLOCATION:

						bPackBuf[1] = (g_tRegisters.s2.tFrequencyList.uAllocationTime >> 8) & 0xFF;
						bPackBuf[0] = g_tRegisters.s2.tFrequencyList.uAllocationTime & 0xFF;

						ResponseAckData(bPackBuf, 2);

						break;

				case SETTING_RSSI_THRESHOLD:

						bPackBuf[0] = g_tRegisters.s2.tFrequencyList.cRSSIThreshold;

						ResponseAckData(bPackBuf, 1);

						break;

				// TX/RX power settings
				case SETTING_OUTPUT_POWER:

						bPackBuf[0] = g_tRegisters.s2.cRFTxPower;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_RX_SENSITIVITY:

						bPackBuf[0] = g_tRegisters.s2.cRFRxSensitivity;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_EXTERNAL_GAIN:

						if(g_tRegisters.s2.bPowerGain == 0)
								bPackBuf[0] = POWER_GAIN_0DBM;
						else if(g_tRegisters.s2.bPowerGain == PA_GAIN08_SEL)
								bPackBuf[0] = POWER_GAIN_8DBM;
						else if(g_tRegisters.s2.bPowerGain == PA_GAIN16_SEL)
								bPackBuf[0] = POWER_GAIN_16DBM;
						else
								bPackBuf[0] = POWER_GAIN_24DBM;

						ResponseAckData(bPackBuf, 1);

						break;

				// Gen2 settings
				case SETTING_LINK_FREQ:

						if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_40)
								bPackBuf[0] = LINK_FREQ_40;
						else if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_80)
								bPackBuf[0] = LINK_FREQ_80;
						else if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_160)
								bPackBuf[0] = LINK_FREQ_160;
						else if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_256)
								bPackBuf[0] = LINK_FREQ_256;
						else if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_320)
								bPackBuf[0] = LINK_FREQ_320;
						else
								bPackBuf[0] = LINK_FREQ_640;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_RX_DECODE:

						if(g_tRegisters.s2.bRxDecode == GEN2_COD_FM0)
								bPackBuf[0] = RX_DECODE_FM0;
						else if(g_tRegisters.s2.bRxDecode == GEN2_COD_MILLER2)
								bPackBuf[0] = RX_DECODE_MI2;
						else if(g_tRegisters.s2.bRxDecode == GEN2_COD_MILLER4)
								bPackBuf[0] = RX_DECODE_MI4;
						else
								bPackBuf[0] = RX_DECODE_MI8;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_Q_BEGIN:

						bPackBuf[0] = g_tRegisters.s2.bGen2QBegin;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_SESSION_NUM:

						if(g_tRegisters.s2.bGen2Session == AS3993_REG3C_GEN2_SESSION_S1S0_0)
								bPackBuf[0] = SESSION_NUM_0;
						else if(g_tRegisters.s2.bGen2Session == AS3993_REG3C_GEN2_SESSION_S1S0_1)
								bPackBuf[0] = SESSION_NUM_1;
						else if(g_tRegisters.s2.bGen2Session == AS3993_REG3C_GEN2_SESSION_S1S0_2)
								bPackBuf[0] = SESSION_NUM_2;
						else
								bPackBuf[0] = SESSION_NUM_3;

						ResponseAckData(bPackBuf, 1);

						break;

				case SETTING_PILOT_TONE:

						if(g_tRegisters.s2.bGen2Trext == TREXT_OFF)
								bPackBuf[0] = PILOT_TONE_NONE;
						else
								bPackBuf[0] = PILOT_TONE_USED;

						ResponseAckData(bPackBuf, 1);

						break;

				// Hardware settings
				case SETTING_BLE_MACADDR:

						memcpy(bPackBuf, g_tRegisters.s2.ucBLEMacAddress, 6);

						ResponseAckData(bPackBuf, 6);

						break;

				case SETTING_BLE_DEVNAME:

						// reply for Nordic's request
						SendDevNameToNordic();

						break;

				case SETTING_BLE_DEVID:

						// reply for Nordic's request
						SendDevIDToNordic();

						break;

				default:

						cResult = ERR_CMD_INCORRECT_PARAM;
				}
				break;

		case AUR_DO_ECHO:

				ResponseAckData((u8*)sEcho_Message, strlen(sEcho_Message));

				cResult = ERR_CMD_DONT_RESPONSE;

				break;

		case AUR_DO_INIT:

				g_tRegisters = DEFALUT_UHF_REGISTERS;
				DefaultInit();

				gen2Initialize();
				ScreenInit();

				break;

		case AUR_DATABASE_ACCESS:

        switch(cmd_parser.tFormat.bPayload[0])
        {
        case DATA_BASE_INFO:

						bPackBuf[0] = DATABASE_VER_SUB;
						bPackBuf[1] = DATABASE_VER_MAIN;
						bPackBuf[2] = TBL_CNT_MAX & 0xFF;
						bPackBuf[3] = (TBL_CNT_MAX >> 8) & 0xFF;

						ResponseAckData(bPackBuf, 4);

						cResult = ERR_CMD_DONT_RESPONSE;

						break;

        case DATA_TABLE_INFO:

						switch((cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1])
						{
						case TBL_TYPE_LOGGER:
						case TBL_TYPE_DETAIL:

								bCurrQueryTbl = (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1];

								uData_Len = 4;
								EEPROM_ReadData(bData_Buf, 0x02, &uData_Len);

								if((bData_Buf[2] == 0x04) && (bData_Buf[3] == 0x00))												// 0x0400 - first position of logger
								{
										bPackBuf[0] = bData_Buf[1];			// bData_Buf[1] is LSB
										bPackBuf[1] = bData_Buf[0];			// bData_Buf[0] is MSB
										tbl_info[(cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]].uTotalRecCnt = (bData_Buf[0] << 8) | bData_Buf[1];
								}
								else
								{
										bPackBuf[0] = 0;
										bPackBuf[1] = 0;
										tbl_info[(cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[1]].uTotalRecCnt = 0;
								}

								bPackBuf[2] = LOGGER_CNT_MAX & 0xFF;
								bPackBuf[3] = (LOGGER_CNT_MAX >> 8) & 0xFF;

								ResponseAckData(bPackBuf, 4);

								cResult = ERR_CMD_DONT_RESPONSE;

								break;

						case TBL_TYPE_USER:

								bCurrQueryTbl = TBL_TYPE_USER;

								uData_Len = 2;
								EEPROM_ReadData(bData_Buf, USERID_ADR_START, &uData_Len);

								if((bData_Buf[0] == 0xFF) && (bData_Buf[1] == 0xFF))
								{
										bPackBuf[0] = 0;
										bPackBuf[1] = 0;
										tbl_info[TBL_TYPE_USER].uTotalRecCnt = 0;
								}
								else
								{
										bPackBuf[0] = bData_Buf[1];			// bData_Buf[1] is LSB
										bPackBuf[1] = bData_Buf[0];			// bData_Buf[0] is MSB
										tbl_info[TBL_TYPE_USER].uTotalRecCnt = (bData_Buf[0] << 8) | bData_Buf[1];
								}

								bPackBuf[2] = USERID_CNT_MAX & 0xFF;
								bPackBuf[3] = (USERID_CNT_MAX >> 8) & 0xFF;

								ResponseAckData(bPackBuf, 4);

								cResult = ERR_CMD_DONT_RESPONSE;

								break;

						default:

								cResult = ERR_CMD_INCORRECT_PARAM;
						}

						break;

        case DATA_OPEN_TABLE:

						if(cmd_parser.tFormat.bPayload[1] == bCurrQueryTbl)
						{
								if(tbl_info[bCurrQueryTbl].bOpened == TBL_STATE_OFF)
								{
										tbl_info[bCurrQueryTbl].bOpened = TBL_STATE_ON;
										tbl_info[bCurrQueryTbl].uCurrRecNum = 0;
								}
								else
										cResult = ERR_CMD_TABLE_OPENED;
						}
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

        case DATA_CLOSE_TABLE:

						tbl_info[bCurrQueryTbl].bOpened = TBL_STATE_OFF;

						bCurrQueryTbl = 0xFF;			// need to Query Table again before Open Table

						break;

        case DATA_GET_RECORD:

						if(cmd_parser.tFormat.bPayload[1] == bCurrQueryTbl)
						{
								if(tbl_info[bCurrQueryTbl].bOpened == TBL_STATE_ON)
								{
										switch(bCurrQueryTbl)
										{
										case TBL_TYPE_LOGGER:

												if(tbl_info[TBL_TYPE_LOGGER].uCurrRecNum < tbl_info[TBL_TYPE_LOGGER].uTotalRecCnt)
												{
														uData_Len = 2;
														uData_Addr = (tbl_info[TBL_TYPE_LOGGER].uCurrRecNum << 2) + LOGGER_STADDR_OFFSET;
														EEPROM_ReadData(bData_Buf, uData_Addr, &uData_Len);

														uData_Len = LOGGER_INFO_OFFSET;
														uData_Addr = (bData_Buf[0] << 8) | bData_Buf[1];
														EEPROM_ReadData(bPackBuf + 2, uData_Addr, &uData_Len);

														bPackBuf[0] = tbl_info[TBL_TYPE_LOGGER].uCurrRecNum & 0xFF;
														bPackBuf[1] = (tbl_info[TBL_TYPE_LOGGER].uCurrRecNum >> 8) & 0xFF;

														ResponseAckData(bPackBuf, uData_Len + 2);

														tbl_info[TBL_TYPE_LOGGER].uCurrRecNum++;

														cResult = ERR_CMD_DONT_RESPONSE;
												}
												else
														cResult = ERR_CMD_TABLE_ENDED;

												break;

										case TBL_TYPE_DETAIL:

												if(tbl_info[TBL_TYPE_DETAIL].uCurrRecNum < tbl_info[TBL_TYPE_DETAIL].uTotalRecCnt)
												{
														uData_Len = 4;
														uData_Addr = (tbl_info[TBL_TYPE_DETAIL].uCurrRecNum << 2) + LOGGER_STADDR_OFFSET;
														EEPROM_ReadData(bData_Buf, uData_Addr, &uData_Len);

														uData_Len = ((bData_Buf[2] << 8) | bData_Buf[3]) - LOGGER_INFO_OFFSET;
														uData_Addr = ((bData_Buf[0] << 8) | bData_Buf[1]) + LOGGER_INFO_OFFSET;

														while(uData_Len > 0)
														{
																uRemain_Len = (uData_Len >= 253) ? 253 : uData_Len;			// since the max length of bPayload including uCurrRecNum(2 bytes) is 255 (0xFF)

																EEPROM_ReadData(bPackBuf + 2, uData_Addr, &uRemain_Len);

																bPackBuf[0] = tbl_info[TBL_TYPE_DETAIL].uCurrRecNum & 0xFF;
																bPackBuf[1] = (tbl_info[TBL_TYPE_DETAIL].uCurrRecNum >> 8) & 0xFF;

																ResponseAckData(bPackBuf, uRemain_Len + 2);

																uData_Addr += uRemain_Len;
																uData_Len -= uRemain_Len;
														}

														tbl_info[TBL_TYPE_DETAIL].uCurrRecNum++;
												}
												else
														cResult = ERR_CMD_TABLE_ENDED;

												break;

										case TBL_TYPE_USER:

												if(tbl_info[TBL_TYPE_USER].uCurrRecNum < tbl_info[TBL_TYPE_USER].uTotalRecCnt)
												{
														uData_Len = 20;
														uData_Addr = (tbl_info[TBL_TYPE_USER].uCurrRecNum * 20) + LOGGER_STADDR_OFFSET + USERID_ADR_START;
														EEPROM_ReadData(bPackBuf, uData_Addr, &uData_Len);

														uData_Len = 0;
														while(bPackBuf[uData_Len] != '\0')
														{
																uData_Len++;
														}

														ResponseAckData(bPackBuf, uData_Len);

														tbl_info[TBL_TYPE_USER].uCurrRecNum++;

														cResult = ERR_CMD_DONT_RESPONSE;
												}
												else
														cResult = ERR_CMD_TABLE_ENDED;

												break;

										default:

												cResult = ERR_CMD_INCORRECT_PARAM;
										}
								}
								else
										cResult = ERR_CMD_TABLE_CLOSED;
						}
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

        case DATA_ADD_RECORD:

						if(tbl_info[bCurrQueryTbl].bOpened == TBL_STATE_ON)
						{
								switch(bCurrQueryTbl)
								{
								case TBL_TYPE_LOGGER:
								case TBL_TYPE_DETAIL:

										cResult = ERR_CMD_INVALID_OPERATION;

										break;

								case TBL_TYPE_USER:

										if(tbl_info[TBL_TYPE_USER].uTotalRecCnt < USERID_CNT_MAX)
										{
												for(i = 0; i < (cmd_parser.tFormat.bLength - 1); i++)
														bData_Buf[i] = cmd_parser.tFormat.bPayload[1 + i];

												bData_Buf[i] = '\0';

												uData_Len = cmd_parser.tFormat.bLength;
												uData_Addr = (tbl_info[TBL_TYPE_USER].uTotalRecCnt * 20) + LOGGER_STADDR_OFFSET + USERID_ADR_START;
												EEPROM_WriteData(bData_Buf, uData_Addr, &uData_Len);

												tbl_info[TBL_TYPE_USER].uTotalRecCnt++;

												bData_Buf[0] = (tbl_info[TBL_TYPE_USER].uTotalRecCnt >> 8) & 0xFF;		// bData_Buf[0] is MSB
												bData_Buf[1] = tbl_info[TBL_TYPE_USER].uTotalRecCnt & 0xFF;		// bData_Buf[1] is LSB

												uData_Len = 2;
												uData_Addr = USERID_ADR_START;
												EEPROM_WriteData(bData_Buf, uData_Addr, &uData_Len);
										}
										else
												cResult = ERR_CMD_TABLE_ENDED;

										break;

								default:

										cResult = ERR_CMD_INCORRECT_PARAM;
								}
						}
						else
								cResult = ERR_CMD_TABLE_CLOSED;

						break;

        case DATA_RESEND_CMD:



						break;

        case DATA_DEL_RECORDS:

						if(tbl_info[bCurrQueryTbl].bOpened == TBL_STATE_ON)
						{
								switch(bCurrQueryTbl)
								{
								case TBL_TYPE_LOGGER:
								case TBL_TYPE_DETAIL:

										uData_Len = 6;
										uData_Addr = 0x02;
										memset(bData_Buf, 0, uData_Len);
										EEPROM_WriteData(bData_Buf, uData_Addr, &uData_Len);

										break;

								case TBL_TYPE_USER:

										uData_Len = 2;
										uData_Addr = USERID_ADR_START;
										memset(bData_Buf, 0, uData_Len);
										EEPROM_WriteData(bData_Buf, uData_Addr, &uData_Len);

										break;

								default:

										cResult = ERR_CMD_INCORRECT_PARAM;
								}
						}
						else
								cResult = ERR_CMD_TABLE_CLOSED;

						break;

				default:

						cResult = ERR_CMD_INCORRECT_PARAM;
				}

				break;

		case AUR_UPDATE_SETTING:

				uData_Len = sizeof(g_tRegisters.s2);

				FLASH_EEPROM_WriteBytes(g_tRegisters.bRegisters, 0, &uData_Len);

				break;

		case AUR_READ_EPC_TID:

				bLoopTask = CMD_TASK_READEPCTID;

				ulLoop_Timeout = cmd_parser.tFormat.bPayload[0] * 1000;
				ulLoop_Tick = HAL_GetTick();

				break;

		case AUR_DO_BUZZER:

				bLoopBUZZ = LOOP_EN_DONE;
				ulBUZZ_Timeout = 1000;
				ulEvent_Tick = HAL_GetTick();
				HAL_TIM_Base_Start_IT(&Tim3Handle);

				break;

		case AUR_DO_LEDS:

				if(cmd_parser.tFormat.bPayload[0] == LED_ALL_OFF)
				{
						HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);
				}
				else if(cmd_parser.tFormat.bPayload[0] == LED_GREEN_ON)
				{
						HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_SET);
				}
				else if(cmd_parser.tFormat.bPayload[0] == LED_RED_ON)
				{
						HAL_GPIO_WritePin(GPIOA, LED1_GRN_PIN, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, LED2_RED_PIN, GPIO_PIN_RESET);
				}
				else
						cResult = ERR_CMD_INCORRECT_PARAM;

				ulCheck_Tick = HAL_GetTick();			// to keep LED status for fixed duration before checking the charging state

				break;

		case AUR_BATT_LEVEL:

				*(u16*)bPackBuf = ADC_AverageGet();

				ResponseAckData(bPackBuf, 2);

				cResult = ERR_CMD_DONT_RESPONSE;

				break;

		case AUR_TAG_RSSI:

				if(uhfIsValidFrequency((cmd_parser.tFormat.bPayload[2] << 16) | (cmd_parser.tFormat.bPayload[1] << 8) | cmd_parser.tFormat.bPayload[0]) == 0)
				{
						gen2ResetInventoryRound();

						uhfSetReaderPowerMode(UHF_READER_IDLE_POWER_MODE);
						uhfSetBaseFrequency((u32)(cmd_parser.tFormat.bPayload[2] << 16) | (u32)(cmd_parser.tFormat.bPayload[1] << 8) | (u32)cmd_parser.tFormat.bPayload[0]);

						bLoopTask = CMD_TASK_QUERYTAGRSSI;

						ulLoop_Timeout = 1000;
						ulLoop_Tick = HAL_GetTick();

						cResult = ERR_CMD_DONT_RESPONSE;
				}
				else
						cResult = ERR_CMD_INCORRECT_PARAM;

				break;

		case AUR_REFLECTED_PWR:

				if(uhfIsValidFrequency((cmd_parser.tFormat.bPayload[2] << 16) | (cmd_parser.tFormat.bPayload[1] << 8) | cmd_parser.tFormat.bPayload[0]) == 0)
				{
						GetFrequencyReflectedPower((u32)(cmd_parser.tFormat.bPayload[2] << 16) | (u32)(cmd_parser.tFormat.bPayload[1] << 8) | (u32)cmd_parser.tFormat.bPayload[0], bPackBuf);

						ResponseAckData(bPackBuf, 3);

						cResult = ERR_CMD_DONT_RESPONSE;
				}
				else
						cResult = ERR_CMD_INCORRECT_PARAM;

				break;

		case AUR_RADIO_MODE:

				switch(cmd_parser.tFormat.bPayload[0])
				{
				case RADIO_TEST_MODE_CONTINUOUS_WAVE:
				case RADIO_TEST_MODE_MODULATION:

						bLoopTask = CMD_TEST_CONTWAVE + cmd_parser.tFormat.bPayload[0];

						g_tRegisters.s2.tFrequencyList.bNumberOfFrequency = 1;

						// close output power
						uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL);

						g_ulFrequency = getFrequency(0);
						uhfSetBaseFrequency(g_ulFrequency);

						gen2DisableHoppingFrequency();

						uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);

						g_tRegisters.s2.g_bRadioTestModeAction = cmd_parser.tFormat.bPayload[0];

						ulLoop_Timeout = MAX_TIME_OUT;		// to prepare for endless loop
						ulLoop_Tick = HAL_GetTick();

						break;

				case RADIO_TEST_MODE_HOPPING:

						bLoopTask = CMD_TEST_HOPPING;

						gen2EnableHoppingFrequency();

						g_tRegisters.s2.g_bRadioTestModeAction = RADIO_TEST_MODE_HOPPING;

						ulLoop_Timeout = MAX_TIME_OUT;		// to prepare for endless loop
						ulLoop_Tick = HAL_GetTick();

						break;

				case RADIO_TEST_MODE_NONE:

						bLoopTask = B2E_TASK_NONE;

            uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL);

						gen2EnableHoppingFrequency();

						g_tRegisters.s2.g_bRadioTestModeAction = RADIO_TEST_MODE_NONE;

						ulLoop_Timeout = 0;						// to end up the loop
						ulLoop_Tick = HAL_GetTick();

						break;

				default:

						cResult = ERR_CMD_INCORRECT_PARAM;
				}

				break;

		case AUR_DO_OLED:

				OLED_Full_Screen();

				bIdleBreak = TRUE;		// when OLED is off in sleep mode
				ulCheck_Tick = HAL_GetTick();			// to keep full display for fixed duration before checking the charging state

				break;

		case AUR_SHOW_MESSAGE:

        OLED_Clear_Screen();
				vPrintText(1, "[Message]");

				bMessageLen = cmd_parser.tFormat.bLength;

				i = 2;		// starting line

				while((bMessageLen > 0) && (i < 5))
				{
						if (bMessageLen > 16)
						{
								memset(bMessageBuf, 0, sizeof(bMessageBuf));
								memcpy(bMessageBuf, cmd_parser.tFormat.bPayload + (i - 2) * 16, 16);

								vPrintText(i++, "%s", bMessageBuf);
								bMessageLen -= 16;
						}
						else
						{
								memset(bMessageBuf, 0, sizeof(bMessageBuf));
								memcpy(bMessageBuf, cmd_parser.tFormat.bPayload + (i - 2) * 16, bMessageLen);

								vPrintText(i++, "%s", bMessageBuf);
								bMessageLen = 0;
						}
				}

				bIdleBreak = TRUE;		// when OLED is off in sleep mode
				ulCheck_Tick = HAL_GetTick();			// to keep LED status for fixed duration before checking the charging state

				break;

		case AUR_UPDATE_BLE_STATE:

				if(cmd_parser.tFormat.bPayload[0] == BLE_STATE_DIS)
						bBLEConnected = BLE_STATE_DIS;
				else if(cmd_parser.tFormat.bPayload[0] == BLE_STATE_EN)
						bBLEConnected = BLE_STATE_EN;
				else
						cResult = ERR_CMD_INCORRECT_PARAM;

				bIdleBreak = TRUE;

				break;

		case AUR_DO_BARCODER:

				if(cmd_parser.tFormat.bPayload[0] == BARC_STATE_DIS)
						HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_SET);
				else if(cmd_parser.tFormat.bPayload[0] == BARC_STATE_EN)
						HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_RESET);
				else
						cResult = ERR_CMD_INCORRECT_PARAM;

				break;

		case AUR_READ_BARCODE:

				HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_RESET);				// BPWR on

				if(InitBarcReader() == ERR_BARC_OK)
				{
						bLoopTask = CMD_TASK_READBARCODE;

						ulLoop_Timeout = cmd_parser.tFormat.bPayload[0] * 1000;
						ulLoop_Tick = HAL_GetTick();

						tSl900a.bBarcNum = 0;		// Reset Barcode number

						g_bBarcTempLen = 0;
						BARCODE_TRIGGER_ACTIVATE(bData_Buf, uData_Len);
						while(HAL_BUSY == HAL_UART_Transmit_IT(&Uart1Handle, bData_Buf, uData_Len));

						delay_ms(100);
				}
				else
				{
						HAL_GPIO_WritePin(GPIOA, BARC_PWR_PIN, GPIO_PIN_SET);					// BPWR off

						cResult = ERR_BARC_CONFIG;

				}

				break;

		case AUR_DO_LOGGER:

				if(cmd_parser.tFormat.bPayload[0] == LOG_CODE_STARTLOG)
				{
						bLoopTask = CMD_TASK_STARTLOGGER;

						ulLoop_Timeout = 1000;
						ulLoop_Tick = HAL_GetTick();

						cResult = ERR_CMD_DONT_RESPONSE;
				}
				else if(cmd_parser.tFormat.bPayload[0] == LOG_CODE_STOPLOG)
				{
						bLoopTask = CMD_TASK_STOPLOGGER;

						ulLoop_Timeout = 1000;
						ulLoop_Tick = HAL_GetTick();

						cResult = ERR_CMD_DONT_RESPONSE;
				}
				else
						cResult = ERR_CMD_INCORRECT_PARAM;

				break;

		case AUR_DO_VIBRATOR:

				bLoopVIBR = LOOP_EN_DONE;
				ulVIBR_Timeout = 1000;
				ulEvent_Tick = HAL_GetTick();
				HAL_GPIO_WritePin(GPIOA, VIBR_OUT_PIN, GPIO_PIN_SET);

				break;

		case AUR_READ_BUTTON:

				bLoopTask = CMD_TASK_RDPRESSBTN;

				ulLoop_Timeout = cmd_parser.tFormat.bPayload[0] * 1000;
				ulLoop_Tick = HAL_GetTick();

				break;

		case AUR_WRITE_EPC:

				if((cmd_parser.tFormat.bLength > 1) && (cmd_parser.tFormat.bLength < 18))
				{
						uWriteWdLen = cmd_parser.tFormat.bLength / 2;			// instead of "(cmd_parser.tFormat.bLength - 1) / 2"
						memset(bWrData, 0, uWriteWdLen * 2);							// to prevent the odd number of EPC data
						memcpy(bWrData, cmd_parser.tFormat.bPayload + 1, cmd_parser.tFormat.bLength - 1);

						bLoopTask = CMD_TASK_WRITEEPC;

						ulLoop_Timeout = cmd_parser.tFormat.bPayload[0] * 1000;
						ulLoop_Tick = HAL_GetTick();

						cResult = ERR_CMD_DONT_RESPONSE;
				}
				else
						cResult = ERR_CMD_INCORRECT_PARAM;

				break;

		case AUR_BLE_BROADCAST:

				SendDevNameToNordic();
				HAL_Delay(100);
				SendDevIDToNordic();

				break;

		case AUR_DEBUG_TEST:

				memset(bPackBuf, 0x11, 64);
				memset(bPackBuf+64, 0x22, 64);
				memset(bPackBuf+128, 0x33, 64);
				memset(bPackBuf+192, 0x44, 63);

				uData_Len = 200;//255;
				cResult = FLASH_EEPROM_WriteBytes(bPackBuf, 0, &uData_Len);

				if(cResult == HAL_OK)
				{
						ResponseEvent(bPackBuf, uData_Len);

						memset(cmd_parser.tFormat.bPayload, 0, uData_Len);

						cResult = FLASH_EEPROM_ReadBytes(cmd_parser.tFormat.bPayload, 0, &uData_Len);

						if(cResult == HAL_OK)
						{
								ResponseEvent(cmd_parser.tFormat.bPayload, uData_Len);

								if(memcmp(cmd_parser.tFormat.bPayload, bPackBuf, uData_Len) == 0)
								{
										bPackBuf[0] = 0xAB;
										bPackBuf[1] = 0xAB;

										ResponseEvent(bPackBuf, 2);
								}
								else
								{
										bPackBuf[0] = 0xCD;
										bPackBuf[1] = 0xCD;

										ResponseEvent(bPackBuf, 2);
								}
						}
				}

				break;

		case NRF_BLE_BROADCAST:

				SendDevNameToNordic();
				HAL_Delay(100);
				SendDevIDToNordic();

				break;

		default:

				cResult = ERR_CMD_INCORRECT_CODE;
		}

		// 
		if(cResult != ERR_CMD_DONT_RESPONSE)
		{
				if(cResult == ERR_NONE)
						ResponseAck();
				else if(cResult == ERR_BARC_CONFIG)
				{
						cResult = (s8)AUR700_ERROR_REJECT;
						ResponseNakData((u8*)&cResult, 1);
				}
				else if((cResult == ERR_CMD_TABLE_OPENED) || (cResult == ERR_CMD_TABLE_CLOSED) || (cResult == ERR_CMD_INVALID_OPERATION))
				{
						cResult = (s8)AUR700_ERROR_DENY;
						ResponseNakData((u8*)&cResult, 1);
				}
				else if(cResult == ERR_CMD_TABLE_ENDED)
				{
						cResult = (s8)AUR700_ERROR_TBLEND;
						ResponseNakData((u8*)&cResult, 1);
				}
				else
				{
						cResult = (s8)AUR700_ERROR_INCORRECT;
						ResponseNakData((u8*)&cResult, 1);
				}
		}
}



//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  None
  * @retval None
  */
void ResetAURDevice(void)
{
    u32 i = 10000;
		uint32_t JumpAddress;
		pFunction Jump_To_Start;

		__set_FAULTMASK(1);

    HAL_RCC_DeInit();

    while(i--);
    NVIC_SystemReset();
    while(1);

    JumpAddress = *(__IO uint32_t*) (NVIC_VECTTAB_FLASH + 0x04);
    Jump_To_Start = (pFunction) JumpAddress;

    __set_MSP(*(__IO uint32_t*) APP_START_ADDRESS);				// Initialize user application's Stack Pointer

		Jump_To_Start();
    NVIC_SystemReset();
    while(1);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void FreqListGenerate(u32 ulListStart, u32 ulListEnd, u32 ulListInc, u8 bIndexStart, u8 bSecondZone)
{
		u32 ulListNext = ulListStart;
		u8 bIndexNext = bIndexStart;

		while (ulListNext <= ulListEnd)
    {
				if(bIndexNext >= MAX_FREQUENCY_LIST_NUM)				// MAX_FREQUENCY_LIST_NUM = 50
						break;		// don't let the final frequency be overwritten by bigger increment,
											// or don't let the full primary table be overwritten by second spectrum
				g_tRegisters.s2.tFrequencyList.tFrequencies[bIndexNext].bFrequencyLow = ulListNext & 0xFF;
				g_tRegisters.s2.tFrequencyList.tFrequencies[bIndexNext].bFrequencyMid = (ulListNext >> 8) & 0xFF;
				g_tRegisters.s2.tFrequencyList.tFrequencies[bIndexNext].bFrequencyHigh = (ulListNext >> 16) & 0xFF;

				ulListNext += ulListInc;

				bIndexNext++;
		}

		if(bSecondZone == 0)
				g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum = bIndexNext;

		g_tRegisters.s2.tFrequencyList.bNumberOfFrequency = bIndexNext;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void GetFrequencyReflectedPower(u32 ulFrequency, u8* pbBuffer)
{
    u8 bPowerMode;
    IQValue_t tNoiseIQValue;
    IQValue_t tIQValue;

    bPowerMode = uhfGetReaderPowerMode();

		uhfSetReaderPowerMode(UHF_READER_IDLE_POWER_MODE);

		g_ulFrequency = ulFrequency;
		uhfSetBaseFrequency(g_ulFrequency);

    uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_REC_ON);
    tNoiseIQValue = uhfGetMixerIQLevel();

    uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);
    tIQValue = uhfGetMixerIQLevel();

    uhfSetReaderPowerMode(bPowerMode);

		pbBuffer[0] = tIQValue.st1.cIValue - tNoiseIQValue.st1.cIValue;
    pbBuffer[1] = tIQValue.st1.cQValue - tNoiseIQValue.st1.cQValue;
    pbBuffer[2] = uhfGetRxMixerGainValue();
}


//----------------------------------------------------------------------------//
/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 AURStartLog(void)
{
		s8 cResult;

		m_bIsAccessTag = 1;
    g_tQueryParams.bTarget = bQueryTarget & GEN2_QRY_TARGET_MASK;//GEN2_QRY_TARGET_A;

		cResult = gen2Inventory();

		if(cResult == ERR_GEN2_OK)
		{
				cResult = OpenPasswordAll(SL900_OPEN);
		}

		if(cResult == ERR_GEN2_OK)
		{
				cResult = StartLogging();
		}

		if(cResult == ERR_GEN2_OK)
		{
				cResult = OpenPasswordAll(SL900_SETPW);
		}

		if(cResult == ERR_GEN2_OK)
		{
				ResponseAck();
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 AURStopLog(void)
{
		s8 cResult;

		m_bIsAccessTag = 1;
    g_tQueryParams.bTarget = bQueryTarget & GEN2_QRY_TARGET_MASK;//GEN2_QRY_TARGET_A;

		cResult = gen2Inventory();

		if(cResult == ERR_GEN2_OK)
		{
				cResult = OpenPasswordAll(SL900_OPEN);
		}

		if(cResult == ERR_GEN2_OK)
		{
				cResult = StopLogging();
		}

		if(cResult == ERR_GEN2_OK)
		{
				cResult = OpenPasswordAll(SL900_SETPW);
		}

		if(cResult == ERR_GEN2_OK)
		{
				ResponseAck();
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 AURQueryTagRSSI(void)
{
		s8 cResult;
		u8 i;
		IQValue_t tIQValue;

		m_bIsAccessTag = 1;
    g_tQueryParams.bTarget = bQueryTarget & GEN2_QRY_TARGET_MASK;//GEN2_QRY_TARGET_A;

		gen2DisableHoppingFrequency();

		cResult = gen2Inventory();

		if(cResult == ERR_GEN2_OK)
		{
				tIQValue = as3993GetMixerIQLevel();

				bPackBuf[0] = tIQValue.st1.cIValue;
				bPackBuf[1] = tIQValue.st1.cQValue;
				bPackBuf[2] = g_tCurrentTag.tTag.cRSSI;
				bPackBuf[3] = g_tCurrentTag.tTag.bEPCLen - 2;

				for(i = 0; i < (g_tCurrentTag.tTag.bEPCLen - 2); i++)
						bPackBuf[4 + i] = g_tCurrentTag.tTag.bEPCs[i];

				ResponseAckData(bPackBuf, 2 + g_tCurrentTag.tTag.bEPCLen);		// 4 + (g_tCurrentTag.tTag.bEPCLen - 2)
		}

    uhfSetReaderPowerMode(UHF_POWER_MODE_STANDBY);

    gen2EnableHoppingFrequency();

    return cResult;
}

/**
  * @brief  
  * @param  None
  * @retval Error code
  */
s8 AURReadEPCTID(void)
{
		s8 cResult;
		u8 i;

		m_bIsAccessTag = 1;
    g_tQueryParams.bTarget = bQueryTarget & GEN2_QRY_TARGET_MASK;//GEN2_QRY_TARGET_A;

		cResult = gen2Inventory();

		if(cResult == ERR_GEN2_OK)
		{
				cResult = SL900ReadTagTID();
		}

		if(cResult == ERR_GEN2_OK)
		{
				bPackBuf[0] = AUR700_EVENT_TAGDATA;
				bPackBuf[1] = g_tCurrentTag.tTag.bEPCLen - 2;

				for(i = 0; i < (g_tCurrentTag.tTag.bEPCLen - 2); i++)
						bPackBuf[2 + i] = g_tCurrentTag.tTag.bEPCs[i];

				bPackBuf[g_tCurrentTag.tTag.bEPCLen] = g_tCurrentTag.bTIDLen;		// 2 + (g_tCurrentTag.tTag.bEPCLen - 2)

				for(i = 0; i < g_tCurrentTag.bTIDLen; i++)
						bPackBuf[(g_tCurrentTag.tTag.bEPCLen + 1) + i] = g_tCurrentTag.bTIDs[i];

				bPackBuf[(g_tCurrentTag.tTag.bEPCLen + 1) + g_tCurrentTag.bTIDLen] = g_tRegisters.s2.ucDeviceID[0];
				bPackBuf[(g_tCurrentTag.tTag.bEPCLen + 2) + g_tCurrentTag.bTIDLen] = g_tRegisters.s2.ucDeviceID[1];

				ResponseEvent(bPackBuf, (g_tCurrentTag.tTag.bEPCLen + 3) + g_tCurrentTag.bTIDLen);
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 AURWriteEPC(u16 uWordCount, u8* pbBankData)
{
		s8 cResult;
		u8 bPCData[PC_LENGTH];

		m_bIsAccessTag = 1;
    g_tQueryParams.bTarget = bQueryTarget & GEN2_QRY_TARGET_MASK;//GEN2_QRY_TARGET_A;

		cResult = gen2Inventory();

		if(cResult == ERR_GEN2_OK)
		{
				memcpy(bPCData, g_tCurrentTag.tTag.bPCs, PC_LENGTH);
				bPCData[0] = (bPCData[0] & 0x07) | (uWordCount << 3);

				cResult = SL900WriteTagBank(MEM_EPC, 0x01, 1, bPCData);		// write PC data
		}

		if(cResult == ERR_GEN2_OK)
		{
				cResult = SL900WriteTagBank(MEM_EPC, 0x02, uWordCount, pbBankData);
		}

		if(cResult == ERR_GEN2_OK)
		{
				ResponseAck();
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 AURReadPressBtn(void)
{
		s8 cResult = ERR_NONE;
		u8 bPressBtn;

		if(HAL_GPIO_ReadPin(GPIOA, LEFT_KEY_PIN) == GPIO_PIN_RESET)
				bPressBtn = PRESS_BTN_BACK;
		else if(HAL_GPIO_ReadPin(GPIOA, RIGHT_KEY_PIN) == GPIO_PIN_RESET)
				bPressBtn = PRESS_BTN_NEXT;
		else if(HAL_GPIO_ReadPin(GPIOA, TRIG_KEY_PIN) == GPIO_PIN_RESET)
				bPressBtn = PRESS_BTN_ACT;
		else
				cResult = ERR_NOMSG;

		if(cResult == ERR_NONE)
		{
				bPackBuf[0] = AUR700_EVENT_PRESSBTN;
				bPackBuf[1] = bPressBtn;

				ResponseEvent(bPackBuf, 2);
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 AURReadBarcode(void)
{
		s8 cResult;

		cResult = ReadBarcode();

		if(cResult == ERR_BARC_OK)
		{
				bPackBuf[0] = AUR700_EVENT_BARCDATA;
				memcpy(bPackBuf + 1, tSl900a.bBarcStr, tSl900a.bBarcLen);

				ResponseEvent(bPackBuf, tSl900a.bBarcLen + 1);
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void SendDevIDToNordic(void)
{
		bPackBuf[0] = SETTING_BLE_DEVID;
		bPackBuf[1] = g_tRegisters.s2.ucDeviceID[0];
		bPackBuf[2] = g_tRegisters.s2.ucDeviceID[1];

		g_CmdSource = CMD_SOURCE_UART;
		ResponseData(AUR_SET_SETTING, bPackBuf, 3);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void SendDevNameToNordic(void)
{
		bPackBuf[0] = SETTING_BLE_DEVNAME;
		memcpy(bPackBuf + 1, g_tRegisters.s2.ucDeviceName, 10);

		g_CmdSource = CMD_SOURCE_UART;
		ResponseData(AUR_SET_SETTING, bPackBuf, 11);
}



