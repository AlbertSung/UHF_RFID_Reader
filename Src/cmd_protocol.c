/**
  ******************************************************************************
  * @file    cmd_protocol.c
  * @author  Albert
  * @version V1.0.0
  * @date    11-July-2017
  * @brief   commands protocol sub-routine
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "cmd_protocol.h"
#include "gen2_config.h"
#include "b2e.h"
#include "barcode_reader.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CmdProtocol_t cmd_parser;
RepProtocol_t rep_packer;

u8 bPackBuf[REP_MAX_DATA_SIZE];

u32 ulStartFreq, ulIncFreq, ulEndFreq;
u32 ulStartFreq_2, ulIncFreq_2, ulEndFreq_2;

const char sFW_Ver[] = "PGM-Txxxx V1.0R0";
static char sSerial_Num[] = "00000000000000000000";		// 20 characters


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
#ifndef CISC_TEST
		if(cmd_parser.uLen == 0)
		{
				if(bData == CMD_HEADER_SOH)
						cmd_parser.bBuffer[cmd_parser.uLen++] = bData;
		}
		else
		{
				cmd_parser.bBuffer[cmd_parser.uLen++] = bData;

				if(cmd_parser.uLen >= cmd_parser.tFormat.bLength)
				{
						// check CRC
						if(CRC16_Verify(&cmd_parser.tFormat.bLength, (cmd_parser.tFormat.bLength - CMD_HEADER_LEN)) == 0)
						{
								CommandExecute();
						}
						else
						{
						}

						cmd_parser.uLen = 0;
				}

		}
#else
		if(bLoopTask == 0x11)					// Miller
		{
				if((bData >= '0') && (bData <= '3'))
						g_tRegisters.s2.bRxDecode = bData - 0x30;

				gen2Initialize();
				bLoopTask = 0;
		}
		else if(bLoopTask == 0x22)		// Q
		{
				if((bData >= '0') && (bData <= '9'))
						g_tRegisters.s2.bGen2QBegin = bData - 0x30;

				gen2Initialize();
				bLoopTask = 0;
		}
		else if(bLoopTask == 0x33)		// Session
		{
				if((bData >= '0') && (bData <= '3'))
						g_tRegisters.s2.bGen2Session = bData - 0x30;

				gen2Initialize();
				bLoopTask = 0;
		}
		else if(bLoopTask == 0x44)		// Tari
		{
				if((bData >= '0') && (bData <= '2'))
						g_tRegisters.s2.bGen2Tari = bData - 0x30;

				gen2Initialize();
				bLoopTask = 0;
		}
		else if(bLoopTask == 0x55)		// BLF
		{
				if(bData == '0')
						g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_40;
				else if(bData == '1')
						g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_160;
				else if(bData == '2')
						g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_213;
				else if(bData == '3')
						g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_256;
				else if(bData == '4')
						g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_320;
				else if(bData == '5')
						g_tRegisters.s2.bGen2LinkFrequency = GEN2_LF_640;

				gen2Initialize();
				bLoopTask = 0;
		}
		else if(bLoopTask == 0x66)		// Power
		{
				if((bData >= '0') && (bData <= '9'))
						g_tRegisters.s2.cRFTxPower = (s8)0 - (s8)(bData - 0x30);

				gen2Initialize();
				bLoopTask = 0;
		}
		else if(bLoopTask == 0x77)		// PA
		{
				if((bData >= '0') && (bData <= '3'))
						g_tRegisters.s2.bPowerGain = bData - 0x30;

				gen2Initialize();
				bLoopTask = 0;
		}
		else
		{
				ulLoop_Tick = HAL_GetTick();

				if(bData == 'I')				// Inventory
						bLoopTask = 1;
				else if(bData == 'W')		// Write Tag
						bLoopTask = 2;
				else if(bData == 'M')		// Miller
						bLoopTask = 0x11;
				else if(bData == 'Q')		// Q
						bLoopTask = 0x22;
				else if(bData == 'S')		// Session
						bLoopTask = 0x33;
				else if(bData == 'T')		// Tari
						bLoopTask = 0x44;
				else if(bData == 'F')		// BLF
						bLoopTask = 0x55;
				else if(bData == 'R')		// Power
						bLoopTask = 0x66;
				else if(bData == 'G')		// PA
						bLoopTask = 0x77;
				else if(bData == 'P')		// Stop
				{
						bLoopTask = 0;
						bCiscTagsNum = 0;

						// for preamble error
						m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
						uhfSetSensitivity(m_cRxSensitivity);

						uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);
				}
		}
#endif
}

/**
  * @brief  
  * @param  
  * @retval None
  */
//void CommandPacker(u8 bRepCode, u8 bRstCode, const u8* pbArgument, u8 bArgumentLength)
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
    rep_packer.tFormat.bLength = bArgumentLength + REP_FULL_NO_DATA_LEN;
    rep_packer.tFormat.bRepCode = bRepCode;

    if(bArgumentLength > 0)
        memcpy(rep_packer.tFormat.bPayload, pbArgument, bArgumentLength);

    CRC16_Append(&rep_packer.tFormat.bLength, (bArgumentLength + REP_LENGTH_OPCODE_LEN));

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
void ResponseAck(u8 bCmdCode)
{
		CommandPacker((bCmdCode | 0x80), NULL, 0);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ResponseAckData(u8 bCmdCode, const u8* pbData, u8 bDataLength)
{
    CommandPacker((bCmdCode | 0x80), pbData, bDataLength);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ResponseNak(u8 bErrorCode)
{
    CommandPacker(bErrorCode, NULL, 0);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ResponseNakData(u8 bErrorCode, const u8* pbData, u8 bDataLength)
{
    CommandPacker(bErrorCode, pbData, bDataLength);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ResponseEvent(u8 bEvent, const u8* pbData, u8 bDataLength)
{
    CommandPacker(bEvent, pbData, bDataLength);
}



//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  None
  * @retval None
  */
void CommandExecute(void)
{
		s8 cResult = ERR_CMD_INCORRECT_PARAM;

		switch(cmd_parser.tFormat.bCmdCode)
		{
		case B2E_FW_VERSION:

				ResponseAckData(cmd_parser.tFormat.bCmdCode, (u8*)sFW_Ver, 16);

				cResult = ERR_CMD_DONT_RESPONSE;
				break;

		case B2E_WRITE_SETTING:

				cResult = ERR_NONE;

				switch (cmd_parser.tFormat.bPayload[0])
				{
				case SETTING_SERIAL_NUM:

						if((cmd_parser.tFormat.bLength > 6) && (cmd_parser.tFormat.bLength <= (20 + 6)))
						{
								memset(sSerial_Num, 0, 20);
								memcpy(sSerial_Num, cmd_parser.tFormat.bPayload+1, cmd_parser.tFormat.bLength - 6);
						}
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_REGION_ID:

						g_tRegisters.s2.tFrequencyList.bRegionID = cmd_parser.tFormat.bPayload[1];

						break;

				case SETTING_START_FREQ:

            g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyHigh = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyMid = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyLow = cmd_parser.tFormat.bPayload[4];

						ulStartFreq = (g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyLow) |
                          (g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyMid << 8) |
                          (g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyHigh << 16);

						break;

				case SETTING_INC_FREQ:

            g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyHigh = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyMid = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyLow = cmd_parser.tFormat.bPayload[4];

						ulIncFreq = (g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyLow) |
                        (g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyMid << 8) |
                        (g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyHigh << 16);

						break;

				case SETTING_END_FREQ:

            ulEndFreq = (cmd_parser.tFormat.bPayload[2] << 16) | (cmd_parser.tFormat.bPayload[3] << 8) | cmd_parser.tFormat.bPayload[4];

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

            g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyHigh = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyMid = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyLow = cmd_parser.tFormat.bPayload[4];

						ulStartFreq_2 = (g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyLow) |
														(g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyMid << 8) |
														(g_tRegisters.s2.tFrequencyList.tFrequencies_2.bFrequencyHigh << 16);

						break;

				case SETTING_INC2_FREQ:

            g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyHigh = cmd_parser.tFormat.bPayload[2];
            g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyMid = cmd_parser.tFormat.bPayload[3];
            g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyLow = cmd_parser.tFormat.bPayload[4];

						ulIncFreq_2 = (g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyLow) |
													(g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyMid << 8) |
													(g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyHigh << 16);

						break;

				case SETTING_END2_FREQ:

            ulEndFreq_2 = (cmd_parser.tFormat.bPayload[2] << 16) | (cmd_parser.tFormat.bPayload[3] << 8) | cmd_parser.tFormat.bPayload[4];

						if (g_tRegisters.s2.tFrequencyList.ucSecondSpectrum != 0)
						{
								FreqListGenerate(ulStartFreq_2, ulEndFreq_2, ulIncFreq_2, g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum, 1);
						}

						break;

				case SETTING_LISTEN_TIME:

						if(((cmd_parser.tFormat.bPayload[1] << 8) | cmd_parser.tFormat.bPayload[2]) <= MAX_LISTEN_TIME)
								g_tRegisters.s2.tFrequencyList.uListeningTime = (cmd_parser.tFormat.bPayload[1] << 8) | cmd_parser.tFormat.bPayload[2];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_IDLE_TIME:

						if(((cmd_parser.tFormat.bPayload[1] << 8) | cmd_parser.tFormat.bPayload[2]) <= MAX_IDLE_TIME)
								g_tRegisters.s2.tFrequencyList.uIdleTime = (cmd_parser.tFormat.bPayload[1] << 8) | cmd_parser.tFormat.bPayload[2];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_MAX_ALLOCATION:

						if(((cmd_parser.tFormat.bPayload[1] << 8) | cmd_parser.tFormat.bPayload[2]) <= MAX_MAX_ALLOCATION)
								g_tRegisters.s2.tFrequencyList.uAllocationTime = (cmd_parser.tFormat.bPayload[1] << 8) | cmd_parser.tFormat.bPayload[2];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_RSSI_THRESHOLD:

						if(((s8)cmd_parser.tFormat.bPayload[1] >= MIN_RSSI_THRESHOLD) && ((s8)cmd_parser.tFormat.bPayload[1] <= MAX_RSSI_THRESHOLD))
								g_tRegisters.s2.tFrequencyList.cRSSIThreshold = (s8)cmd_parser.tFormat.bPayload[1];
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

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

						if((cmd_parser.tFormat.bPayload[1] >= Q_BEGIN_MIN) && (cmd_parser.tFormat.bPayload[1] <= Q_BEGIN_MAX))
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

				case SETTING_TARGET_FLAG:

						if(cmd_parser.tFormat.bPayload[1] == TARGET_FLAG_A)
								g_tRegisters.s2.bGen2Target = GEN2_QRY_TARGET_A;
						else if(cmd_parser.tFormat.bPayload[1] == TARGET_FLAG_B)
								g_tRegisters.s2.bGen2Target = GEN2_QRY_TARGET_B;
						else if(cmd_parser.tFormat.bPayload[1] == TARGET_FLAG_AB)
								g_tRegisters.s2.bGen2Target = GEN2_QRY_TARGET_A | GEN2_QRY_TARGET_BOTH;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				case SETTING_TARI_VALUE:

						if(cmd_parser.tFormat.bPayload[1] == TARI_VALUE_25U)
								g_tRegisters.s2.bGen2Tari = TARI_25;
						else if(cmd_parser.tFormat.bPayload[1] == TARI_VALUE_12U5)
								g_tRegisters.s2.bGen2Tari = TARI_12_5;
						else if(cmd_parser.tFormat.bPayload[1] == TARI_VALUE_6U25)
								g_tRegisters.s2.bGen2Tari = TARI_6_25;
						else
								cResult = ERR_CMD_INCORRECT_PARAM;

						break;

				default:

						cResult = ERR_CMD_INCORRECT_PARAM;
				}

				gen2Initialize();

				break;

		case B2E_READ_SETTING:

				cResult = ERR_CMD_DONT_RESPONSE;
				bPackBuf[0] = cmd_parser.tFormat.bPayload[0];

				switch (cmd_parser.tFormat.bPayload[0])
				{
				case SETTING_SERIAL_NUM:

						memcpy(bPackBuf+1, sSerial_Num, strlen(sSerial_Num));

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, strlen(sSerial_Num) + 1);

						break;

				case SETTING_REGION_ID:

						bPackBuf[1] = g_tRegisters.s2.tFrequencyList.bRegionID;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_START_FREQ:

            bPackBuf[1] = 0;
            bPackBuf[2] = g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyHigh;
            bPackBuf[3] = g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyMid;
            bPackBuf[4] = g_tRegisters.s2.tFrequencyList.tFrequencies[0].bFrequencyLow;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 5);

						break;

				case SETTING_INC_FREQ:

            bPackBuf[1] = 0;
            bPackBuf[2] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyHigh;
            bPackBuf[3] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyMid;
            bPackBuf[4] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq.bFrequencyLow;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 5);

						break;

				case SETTING_END_FREQ:

            bPackBuf[1] = 0;
            bPackBuf[2] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum-1].bFrequencyHigh;
            bPackBuf[3] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum-1].bFrequencyMid;
            bPackBuf[4] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum-1].bFrequencyLow;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 5);

						break;

				case SETTING_2ND_SPEC:

						bPackBuf[1] = g_tRegisters.s2.tFrequencyList.ucSecondSpectrum;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_START2_FREQ:

            bPackBuf[1] = 0;

						if(g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum < g_tRegisters.s2.tFrequencyList.bNumberOfFrequency)
						{
								bPackBuf[2] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum].bFrequencyHigh;
								bPackBuf[3] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum].bFrequencyMid;
								bPackBuf[4] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum].bFrequencyLow;
						}
						else
						{
								bPackBuf[2] = 0;
								bPackBuf[3] = 0;
								bPackBuf[4] = 0;
						}

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 5);

						break;

				case SETTING_INC2_FREQ:

            bPackBuf[1] = 0;
            bPackBuf[2] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyHigh;
            bPackBuf[3] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyMid;
            bPackBuf[4] = g_tRegisters.s2.tFrequencyList.ucIncrementFreq_2.bFrequencyLow;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 5);

						break;

				case SETTING_END2_FREQ:

            bPackBuf[1] = 0;

						if(g_tRegisters.s2.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum < g_tRegisters.s2.tFrequencyList.bNumberOfFrequency)
						{
								bPackBuf[2] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequency-1].bFrequencyHigh;
								bPackBuf[3] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequency-1].bFrequencyMid;
								bPackBuf[4] = g_tRegisters.s2.tFrequencyList.tFrequencies[g_tRegisters.s2.tFrequencyList.bNumberOfFrequency-1].bFrequencyLow;
						}
						else
						{
								bPackBuf[2] = 0;
								bPackBuf[3] = 0;
								bPackBuf[4] = 0;
						}

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 5);

						break;

				case SETTING_LISTEN_TIME:

						bPackBuf[1] = (g_tRegisters.s2.tFrequencyList.uListeningTime >> 8) & 0xFF;
						bPackBuf[2] = g_tRegisters.s2.tFrequencyList.uListeningTime & 0xFF;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 3);

						break;

				case SETTING_IDLE_TIME:

						bPackBuf[1] = (g_tRegisters.s2.tFrequencyList.uIdleTime >> 8) & 0xFF;
						bPackBuf[2] = g_tRegisters.s2.tFrequencyList.uIdleTime & 0xFF;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 3);

						break;

				case SETTING_MAX_ALLOCATION:

						bPackBuf[1] = (g_tRegisters.s2.tFrequencyList.uAllocationTime >> 8) & 0xFF;
						bPackBuf[2] = g_tRegisters.s2.tFrequencyList.uAllocationTime & 0xFF;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 3);

						break;

				case SETTING_RSSI_THRESHOLD:

						bPackBuf[1] = g_tRegisters.s2.tFrequencyList.cRSSIThreshold;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_OUTPUT_POWER:

						bPackBuf[1] = g_tRegisters.s2.cRFTxPower;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_RX_SENSITIVITY:

						bPackBuf[1] = g_tRegisters.s2.cRFRxSensitivity;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_EXTERNAL_GAIN:

						if(g_tRegisters.s2.bPowerGain == 0)
								bPackBuf[1] = POWER_GAIN_0DBM;
						else if(g_tRegisters.s2.bPowerGain == PA_GAIN08_SEL)
								bPackBuf[1] = POWER_GAIN_8DBM;
						else if(g_tRegisters.s2.bPowerGain == PA_GAIN16_SEL)
								bPackBuf[1] = POWER_GAIN_16DBM;
						else
								bPackBuf[1] = POWER_GAIN_24DBM;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_LINK_FREQ:

						if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_40)
								bPackBuf[1] = LINK_FREQ_40;
						else if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_80)
								bPackBuf[1] = LINK_FREQ_80;
						else if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_160)
								bPackBuf[1] = LINK_FREQ_160;
						else if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_256)
								bPackBuf[1] = LINK_FREQ_256;
						else if(g_tRegisters.s2.bGen2LinkFrequency == GEN2_LF_320)
								bPackBuf[1] = LINK_FREQ_320;
						else
								bPackBuf[1] = LINK_FREQ_640;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_RX_DECODE:

						if(g_tRegisters.s2.bRxDecode == GEN2_COD_FM0)
								bPackBuf[1] = RX_DECODE_FM0;
						else if(g_tRegisters.s2.bRxDecode == GEN2_COD_MILLER2)
								bPackBuf[1] = RX_DECODE_MI2;
						else if(g_tRegisters.s2.bRxDecode == GEN2_COD_MILLER4)
								bPackBuf[1] = RX_DECODE_MI4;
						else
								bPackBuf[1] = RX_DECODE_MI8;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_Q_BEGIN:

						bPackBuf[1] = g_tRegisters.s2.bGen2QBegin;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_SESSION_NUM:

						if(g_tRegisters.s2.bGen2Session == AS3993_REG3C_GEN2_SESSION_S1S0_0)
								bPackBuf[1] = SESSION_NUM_0;
						else if(g_tRegisters.s2.bGen2Session == AS3993_REG3C_GEN2_SESSION_S1S0_1)
								bPackBuf[1] = SESSION_NUM_1;
						else if(g_tRegisters.s2.bGen2Session == AS3993_REG3C_GEN2_SESSION_S1S0_2)
								bPackBuf[1] = SESSION_NUM_2;
						else
								bPackBuf[1] = SESSION_NUM_3;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_PILOT_TONE:

						if(g_tRegisters.s2.bGen2Trext == TREXT_OFF)
								bPackBuf[1] = PILOT_TONE_NONE;
						else
								bPackBuf[1] = PILOT_TONE_USED;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_TARGET_FLAG:

						if(g_tRegisters.s2.bGen2Target == GEN2_QRY_TARGET_A)
								bPackBuf[1] = TARGET_FLAG_A;
						else if(g_tRegisters.s2.bGen2Target == GEN2_QRY_TARGET_B)
								bPackBuf[1] = TARGET_FLAG_B;
						else
								bPackBuf[1] = TARGET_FLAG_AB;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				case SETTING_TARI_VALUE:

						if(g_tRegisters.s2.bGen2Tari == TARI_25)
								bPackBuf[1] = TARI_VALUE_25U;
						else if(g_tRegisters.s2.bGen2Tari == TARI_12_5)
								bPackBuf[1] = TARI_VALUE_12U5;
						else
								bPackBuf[1] = TARI_VALUE_6U25;

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 2);

						break;

				default:

						cResult = ERR_CMD_INCORRECT_PARAM;
				}

				break;

		case B2E_RADIO_MODE:

				cResult = ERR_NONE;

				switch (cmd_parser.tFormat.bPayload[0])
				{
				case RADIO_TEST_MODE_CONTINUOUS_WAVE:
				case RADIO_TEST_MODE_MODULATION:

						bLoopTask = B2E_TEST_CONTWAVE + cmd_parser.tFormat.bPayload[0];

						g_tRegisters.s2.tFrequencyList.bNumberOfFrequency = 1;

						// close output power
						uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL);

						g_ulFrequency = getFrequency(0);
						uhfSetBaseFrequency(g_ulFrequency);

						gen2DisableHoppingFrequency();

						uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);

						ulLoop_Timeout = MAX_TIME_OUT;		// to prepare for endless loop
						ulLoop_Tick = HAL_GetTick();

						break;

				case RADIO_TEST_MODE_HOPPING:

						bLoopTask = B2E_TEST_HOPPING;

						gen2EnableHoppingFrequency();

						ulLoop_Timeout = MAX_TIME_OUT;		// to prepare for endless loop
						ulLoop_Tick = HAL_GetTick();

						break;

				case RADIO_TEST_MODE_NONE:

						bLoopTask = B2E_TASK_STOP;

            uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL);

						gen2EnableHoppingFrequency();

						ulLoop_Timeout = 0;						// to end up the loop
						ulLoop_Tick = HAL_GetTick();

						break;

				default:

						cResult = ERR_CMD_INCORRECT_PARAM;
				}

				break;

		case B2E_REFLECTED_POWER:

				if(uhfIsValidFrequency((cmd_parser.tFormat.bPayload[1] << 16) | (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[3]) == 0)
				{
						GetFrequencyReflectedPower((cmd_parser.tFormat.bPayload[1] << 16) | (cmd_parser.tFormat.bPayload[2] << 8) | cmd_parser.tFormat.bPayload[3], bPackBuf);

						ResponseAckData(cmd_parser.tFormat.bCmdCode, bPackBuf, 3);

						cResult = ERR_CMD_DONT_RESPONSE;
				}
				break;

		case B2E_GOTO_ISP:

				HAL_FLASH_Unlock();

				HAL_FLASH_Program(TYPEPROGRAM_WORD, APP_START_ADDRESS-12, 0x55);

				ResponseAck(B2E_GOTO_ISP);		// to be sent here before being reset
				HAL_Delay(100);								// waiting response to be sent out

				ResetB2EDevice();

				cResult = ERR_UNKNOWN;				// unknown error happened if code executed here
				break;

		case B2E_SEARCH_ALL:

				bQueryTarget = g_tRegisters.s2.bGen2Target;

				bLoopTask = B2E_TASK_SEARCH;

				ulLoop_Timeout = ((u32)cmd_parser.tFormat.bPayload[0] << 8) | cmd_parser.tFormat.bPayload[1];
				ulLoop_Tick = HAL_GetTick();

				cResult = ERR_NONE;
				break;

		case B2E_STOP_SEARCH:

				bLoopTask = B2E_TASK_STOP;//B2E_TASK_NONE;

				ulLoop_Timeout = 0;						// to end up the loop
				ulLoop_Tick = HAL_GetTick();

				cResult = ERR_NONE;
				break;

		case B2E_READ_ALL:

				bQueryTarget = g_tRegisters.s2.bGen2Target;

				ulTagPassword = ((u32)cmd_parser.tFormat.bPayload[2] << 24) | ((u32)cmd_parser.tFormat.bPayload[3] << 16)
												 | ((u32)cmd_parser.tFormat.bPayload[4] << 8) | cmd_parser.tFormat.bPayload[5];

				if(cmd_parser.tFormat.bPayload[6] > 0x03)
						break;
				bReadBank = cmd_parser.tFormat.bPayload[6];
				uReadWdAddr = ((u16)cmd_parser.tFormat.bPayload[7] << 8) | cmd_parser.tFormat.bPayload[8];
				uReadWdLen = ((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10];

				bLoopTask = B2E_TASK_READALL;

				ulLoop_Timeout = ((u32)cmd_parser.tFormat.bPayload[0] << 8) | cmd_parser.tFormat.bPayload[1];
				ulLoop_Tick = HAL_GetTick();

				cResult = ERR_NONE;
				break;

		case B2E_WRITE_TAG:

				ulTagPassword = ((u32)cmd_parser.tFormat.bPayload[2] << 24) | ((u32)cmd_parser.tFormat.bPayload[3] << 16)
												 | ((u32)cmd_parser.tFormat.bPayload[4] << 8) | cmd_parser.tFormat.bPayload[5];

				if(cmd_parser.tFormat.bPayload[6] > 0x03)
						break;
				bSelectBank = cmd_parser.tFormat.bPayload[6];
				uSelWdAddr = ((u16)cmd_parser.tFormat.bPayload[7] << 8) | cmd_parser.tFormat.bPayload[8];
				if((((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10]) > (CMD_MAX_DATA_SIZE/2))
						break;
				uSelWdLen = ((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10];
				memcpy(bSelMask, cmd_parser.tFormat.bPayload+11, uSelWdLen * 2);

				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2] > 0x03)
						break;
				bWriteBank = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2];
				uWriteWdAddr = ((u16)cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 1] << 8) | cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 2];
				if((((u16)cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 3] << 8) | cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 4]) > (CMD_MAX_DATA_SIZE/2))
						break;
				uWriteWdLen = ((u16)cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 3] << 8) | cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 4];
				memcpy(bWrData, cmd_parser.tFormat.bPayload + 11 + uSelWdLen*2 + 5, uWriteWdLen * 2);

				bLoopTask = B2E_TASK_WRITETAG;

				ulLoop_Timeout = ((u32)cmd_parser.tFormat.bPayload[0] << 8) | cmd_parser.tFormat.bPayload[1];
				ulLoop_Tick = HAL_GetTick();

				cResult = ERR_NONE;
				break;

		case B2E_READ_TAG:

				ulTagPassword = ((u32)cmd_parser.tFormat.bPayload[2] << 24) | ((u32)cmd_parser.tFormat.bPayload[3] << 16)
												 | ((u32)cmd_parser.tFormat.bPayload[4] << 8) | cmd_parser.tFormat.bPayload[5];

				if(cmd_parser.tFormat.bPayload[6] > 0x03)
						break;
				bSelectBank = cmd_parser.tFormat.bPayload[6];
				uSelWdAddr = ((u16)cmd_parser.tFormat.bPayload[7] << 8) | cmd_parser.tFormat.bPayload[8];
				if((((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10]) > (CMD_MAX_DATA_SIZE/2))
						break;
				uSelWdLen = ((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10];
				memcpy(bSelMask, cmd_parser.tFormat.bPayload+11, uSelWdLen * 2);

				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2] > 0x03)
						break;
				bReadBank = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2];
				uReadWdAddr = ((u16)cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 1] << 8) | cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 2];
				uReadWdLen = ((u16)cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 3] << 8) | cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 4];

				bLoopTask = B2E_TASK_READTAG;

				ulLoop_Timeout = ((u32)cmd_parser.tFormat.bPayload[0] << 8) | cmd_parser.tFormat.bPayload[1];
				ulLoop_Tick = HAL_GetTick();

				cResult = ERR_NONE;
				break;

		case B2E_LOCK_TAG:

				ulTagPassword = ((u32)cmd_parser.tFormat.bPayload[2] << 24) | ((u32)cmd_parser.tFormat.bPayload[3] << 16)
												 | ((u32)cmd_parser.tFormat.bPayload[4] << 8) | cmd_parser.tFormat.bPayload[5];

				if(cmd_parser.tFormat.bPayload[6] > 0x03)
						break;
				bSelectBank = cmd_parser.tFormat.bPayload[6];
				uSelWdAddr = ((u16)cmd_parser.tFormat.bPayload[7] << 8) | cmd_parser.tFormat.bPayload[8];
				if((((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10]) > (CMD_MAX_DATA_SIZE/2))
						break;
				uSelWdLen = ((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10];
				memcpy(bSelMask, cmd_parser.tFormat.bPayload+11, uSelWdLen * 2);

				// Enable all Masks
				g_tLockActions.bEPCMask = 0x03;
				g_tLockActions.bTIDMask = 0x03;
				g_tLockActions.bUserMask = 0x03;
				g_tLockActions.bAccessMask = 0x03;
				g_tLockActions.bKillMask = 0x03;

				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2] < 0x04)
						g_tLockActions.bEPCAction = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2];
				else
						g_tLockActions.bEPCMask = 0x00;
				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 1] < 0x04)
						g_tLockActions.bTIDAction = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 1];
				else
						g_tLockActions.bTIDMask = 0x00;
				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 2] < 0x04)
						g_tLockActions.bUserAction = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 2];
				else
						g_tLockActions.bUserMask = 0x00;
				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 3] < 0x04)
						g_tLockActions.bAccessAction = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 3];
				else
						g_tLockActions.bAccessMask = 0x00;
				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 4] < 0x04)
						g_tLockActions.bKillAction = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 4];
				else
						g_tLockActions.bKillMask = 0x00;

				bLoopTask = B2E_TASK_LOCKTAG;

				ulLoop_Timeout = ((u32)cmd_parser.tFormat.bPayload[0] << 8) | cmd_parser.tFormat.bPayload[1];
				ulLoop_Tick = HAL_GetTick();

				cResult = ERR_CMD_DONT_RESPONSE;
				break;

		case B2E_KILL_TAG:

				if(cmd_parser.tFormat.bPayload[2] > 0x03)
						break;
				bSelectBank = cmd_parser.tFormat.bPayload[2];
				uSelWdAddr = ((u16)cmd_parser.tFormat.bPayload[3] << 8) | cmd_parser.tFormat.bPayload[4];
				if((((u16)cmd_parser.tFormat.bPayload[5] << 8) | cmd_parser.tFormat.bPayload[6]) > (CMD_MAX_DATA_SIZE/2))
						break;
				uSelWdLen = ((u16)cmd_parser.tFormat.bPayload[5] << 8) | cmd_parser.tFormat.bPayload[6];
				memcpy(bSelMask, cmd_parser.tFormat.bPayload+7, uSelWdLen * 2);

				ulTagPassword = ((u32)cmd_parser.tFormat.bPayload[7 + uSelWdLen*2] << 24) | ((u32)cmd_parser.tFormat.bPayload[7 + uSelWdLen*2 + 1] << 16)
												 | ((u32)cmd_parser.tFormat.bPayload[7 + uSelWdLen*2 + 2] << 8) | cmd_parser.tFormat.bPayload[7 + uSelWdLen*2 + 3];

				bLoopTask = B2E_TASK_KILLTAG;

				ulLoop_Timeout = ((u32)cmd_parser.tFormat.bPayload[0] << 8) | cmd_parser.tFormat.bPayload[1];
				ulLoop_Tick = HAL_GetTick();

				cResult = ERR_CMD_DONT_RESPONSE;
				break;

		case B2E_UNTRACE_TAG:

				ulTagPassword = ((u32)cmd_parser.tFormat.bPayload[2] << 24) | ((u32)cmd_parser.tFormat.bPayload[3] << 16)
												 | ((u32)cmd_parser.tFormat.bPayload[4] << 8) | cmd_parser.tFormat.bPayload[5];

				if(cmd_parser.tFormat.bPayload[6] > 0x03)
						break;
				bSelectBank = cmd_parser.tFormat.bPayload[6];
				uSelWdAddr = ((u16)cmd_parser.tFormat.bPayload[7] << 8) | cmd_parser.tFormat.bPayload[8];
				if((((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10]) > (CMD_MAX_DATA_SIZE/2))
						break;
				uSelWdLen = ((u16)cmd_parser.tFormat.bPayload[9] << 8) | cmd_parser.tFormat.bPayload[10];
				memcpy(bSelMask, cmd_parser.tFormat.bPayload+11, uSelWdLen * 2);

				g_tUntraceAction.bRFU = 0;
				g_tUntraceAction.bUBit = 1;			// assert the U bit in XPC_W1

				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2] > 0x01)
						break;
				g_tUntraceAction.bEpcHide = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2];

				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 1] > 0x1F)
						break;
				g_tUntraceAction.bEpcLen = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 1];

				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 2] > 0x02)
						break;
				g_tUntraceAction.bTidHide = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 2];

				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 3] > 0x01)
						break;
				g_tUntraceAction.bUserHide = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 3];

				if(cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 4] > 0x02)
						break;
				g_tUntraceAction.bRange = cmd_parser.tFormat.bPayload[11 + uSelWdLen*2 + 4];

				bLoopTask = B2E_TASK_UNTRACETAG;

				ulLoop_Timeout = ((u32)cmd_parser.tFormat.bPayload[0] << 8) | cmd_parser.tFormat.bPayload[1];
				ulLoop_Tick = HAL_GetTick();

				cResult = ERR_CMD_DONT_RESPONSE;
				break;

#ifdef CISC_TEST
		case B2E_GEN2_SETTING :		// for CISC testing

				if(bLoopTask == B2E_TASK_NONE)
				{
						g_tRegisters.s2.bRxDecode = cmd_parser.tFormat.bPayload[0];			// Miller
						g_tRegisters.s2.bGen2QBegin = cmd_parser.tFormat.bPayload[1];		// Q
						g_tRegisters.s2.bGen2Session = cmd_parser.tFormat.bPayload[2];	// Session
						g_tRegisters.s2.bGen2Tari = cmd_parser.tFormat.bPayload[3];			// Tari
						g_tRegisters.s2.bGen2LinkFrequency = cmd_parser.tFormat.bPayload[4];	// BLF
						g_tRegisters.s2.cRFTxPower = (s8)cmd_parser.tFormat.bPayload[5];		// Power
						g_tRegisters.s2.bPowerGain = cmd_parser.tFormat.bPayload[6];		// PA
						
						gen2Initialize();

						cResult = ERR_NONE;
				}
				break;

		case B2E_MODULATION:	// for CISC testing

        g_tRegisters.s2.tFrequencyList.bNumberOfFrequency = 1;

        // close output power
        uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL);

				uhfSetBaseFrequency((u32)(cmd_parser.tFormat.bPayload[0] << 16) | (u32)(cmd_parser.tFormat.bPayload[1] << 8) | (u32)(cmd_parser.tFormat.bPayload[2]));

        gen2DisableHoppingFrequency();

        uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);

				cResult = ERR_NONE;
				break;
#endif

		case AUR700_TEST:

				// To test Barcode
				ReadBarcode();

				cResult = ERR_CMD_DONT_RESPONSE;
				break;

		default:

				cResult = ERR_CMD_INCORRECT_CODE;
		}

		if(cResult != ERR_CMD_DONT_RESPONSE)
		{
				if(cResult == ERR_NONE)
						ResponseAck(cmd_parser.tFormat.bCmdCode);
				else
						ResponseNak(B2E_ERROR_INCORRECT);
		}

}


//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  None
  * @retval None
  */
void ResetB2EDevice(void)
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


