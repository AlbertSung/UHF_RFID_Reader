/**
  ******************************************************************************
  * @file    cmd_protocol_gnet.h
  * @author  Albert
  * @version V1.0.0
  * @date    31-March-2018
  * @brief   Header for cmd_protocol_gnet.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CMD_PROTOCOL_GNET_H
#define __CMD_PROTOCOL_GNET_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"

/* Defined constants ---------------------------------------------------------*/
#define CMD_MAX_DATA_SIZE					256

#define CMD_HEADER_SOH						0x01

#define CMD_HEADER_LEN						1
#define CMD_ADDRESS_LEN						1
#define CMD_OPCODE_LEN						1
#define CMD_LENGTH_LEN						1
#define CMD_CRC16_LEN							2

#define CMD_BEFORE_DATA_SIZE			(CMD_HEADER_LEN + CMD_ADDRESS_LEN + CMD_OPCODE_LEN + CMD_LENGTH_LEN)	// 4
#define CMD_ADDR_LENGTH_LEN				(CMD_ADDRESS_LEN + CMD_OPCODE_LEN + CMD_LENGTH_LEN)		// 3
#define CMD_FULL_NO_DATA_LEN			(CMD_HEADER_LEN + CMD_ADDR_LENGTH_LEN + CMD_CRC16_LEN)		// 6


#define REP_MAX_DATA_SIZE					256

#define REP_HEADER_LEN						1
#define REP_ADDRESS_LEN						1
#define REP_OPCODE_LEN						1
#define REP_LENGTH_LEN						1
#define REP_CRC16_LEN							2

#define REP_ADDR_LENGTH_LEN				(REP_ADDRESS_LEN + REP_OPCODE_LEN + REP_LENGTH_LEN)		// 3
#define REP_FULL_NO_DATA_LEN			(REP_HEADER_LEN + REP_ADDR_LENGTH_LEN + REP_CRC16_LEN)		// 6


#define DEF_CRC_PRESET            0xFFFF
#define DEF_CRC_POLYNOM           0xA001


#define CMD_REPLY_ACK            	0x06		// Success
#define CMD_REPLY_NAK            	0x15		// Error
#define CMD_REPLY_EVENT						0x12		// Event


#define MAX_READTAG_LEN						200//16	// the maximum of whole packet including bank data is 255 bytes
#define MAX_PACKET_CNT						65535		// 0xFFFF for reading out all bank data when word length is zero

#define MAX_TIME_OUT							65535		// 0xFFFF (ms) for 2 bytes parameters in SW command


#define MAX_Q_BEGIN								8
#define MIN_Q_BEGIN								0
#define MAX_LISTEN_TIME						10000
#define MAX_IDLE_TIME							10000
#define MAX_MAX_ALLOCATION				10000
#define MAX_RSSI_THRESHOLD	 			(-20)
#define MIN_RSSI_THRESHOLD				(-100)
#define MAX_OUTPUT_POWER		 			(0)
#define MIN_OUTPUT_POWER					(-19)
#define MAX_SCAN_TIME				 			60
#define MIN_SCAN_TIME							5
#define MAX_STANDBY_TIME		 			600
#define MIN_STANDBY_TIME					60

#define DATABASE_VER_MAIN					00
#define DATABASE_VER_SUB					01
#define LOGGER_CNT_MAX						57
#define USERID_CNT_MAX						20
#define USERID_ADR_START					0xFE6D


/* Exported constants --------------------------------------------------------*/
// AUR700 Command Set
typedef enum
{
		AUR_FW_VERSION = 0x01,

		AUR_DO_REBOOT = 0x1E,
		AUR_GOTO_ISP = 0x1F,

		AUR_SET_SETTING = 0x20,
		AUR_GET_SETTING = 0x21,
		AUR_DO_ECHO = 0x22,
		AUR_DO_INIT = 0x23,
		AUR_DATABASE_ACCESS = 0x24,
		AUR_UPDATE_SETTING = 0x25,

		AUR_READ_EPC_TID = 0x30,
		AUR_DO_BUZZER = 0x31,
		AUR_DO_LEDS = 0x32,
		AUR_BATT_LEVEL = 0x33,
		AUR_TAG_RSSI = 0x34,
		AUR_REFLECTED_PWR = 0x35,
		AUR_RADIO_MODE = 0x36,
		AUR_DO_OLED = 0x3A,

		AUR_SHOW_MESSAGE = 0x40,
		AUR_UPDATE_BLE_STATE = 0x42,
		AUR_DO_BARCODER = 0x45,
		AUR_READ_BARCODE = 0x46,
		AUR_DO_LOGGER = 0x47,
		AUR_DO_VIBRATOR = 0x48,
		AUR_READ_BUTTON = 0x49,
		AUR_WRITE_EPC = 0x4A,
		AUR_BLE_BROADCAST = 0x4B,

		AUR_DEBUG_TEST = 0x99,				// for AUR700 testing
		NRF_BLE_BROADCAST = 0xAA,

} AUR700_CmdSet;

// AUR700 Setting Code
typedef enum
{
		// Application settings
		SETTING_DEVICE_ID = 0x01,
		SETTING_VIBRATION = 0x02,
		SETTING_BUZZER = 0x03,
		SETTING_RF_STRENGTH = 0x04,
		SETTING_SCAN_TIMEOUT = 0x05,
		SETTING_STANDBY_TIMEOUT = 0x06,
		SETTING_RTC_DATE = 0x07,
		SETTING_RTC_TIME = 0x08,
		SETTING_USER_IDENTIFY = 0x09,
		SETTING_MENU_SELECT = 0x0A,
		SETTING_COMPANY_ID = 0x0B,
		SETTING_ERASE_LOGDATA = 0x0C,
		SETTING_LOCATION_ID = 0x0D,
		SETTING_BARCODER_SELECT = 0x0E,
		SETTING_LOCK_PROTECT = 0x0F,

		// Frequency settings
		SETTING_PROFILE_ID = 0x10,
		SETTING_START_FREQ = 0x11,
		SETTING_END_FREQ = 0x12,
		SETTING_INC_FREQ = 0x13,
		SETTING_LISTEN_TIME = 0x14,
		SETTING_IDLE_TIME = 0x15,
		SETTING_MAX_ALLOCATION = 0x16,
		SETTING_RSSI_THRESHOLD = 0x17,
		SETTING_2ND_SPEC = 0x18,
		SETTING_START2_FREQ = 0x19,
		SETTING_END2_FREQ = 0x1A,
		SETTING_INC2_FREQ = 0x1B,
		SETTING_PROFILE_NAME = 0x1C,

		// TX/RX power settings
		SETTING_OUTPUT_POWER = 0x20,
		SETTING_RX_SENSITIVITY = 0x21,
		SETTING_EXTERNAL_GAIN = 0x22,

		// Gen2 settings
		SETTING_LINK_FREQ = 0x30,
		SETTING_RX_DECODE = 0x31,
		SETTING_Q_BEGIN = 0x32,
		SETTING_SESSION_NUM = 0x33,
		SETTING_PILOT_TONE = 0x34,

		// Hardware settings
		SETTING_BLE_MACADDR = 0x40,
		SETTING_BLE_DEVNAME = 0x41,
		SETTING_BLE_DEVID = 0x42,
		SETTING_BLE_FWREV = 0x43,

} AUR700_SetCode;

// AUR700 Database Code
typedef enum
{
		DATA_BASE_INFO = 0x01,
		DATA_TABLE_INFO = 0x02,
		DATA_OPEN_TABLE = 0x03,
		DATA_CLOSE_TABLE = 0x04,
		DATA_GET_RECORD = 0x05,
		DATA_ADD_RECORD = 0x06,
		DATA_RESEND_CMD = 0x07,
		DATA_DEL_RECORDS = 0x08,

} AUR700_DataCode;

// AUR700 Table Type
typedef enum
{
		TBL_TYPE_LOGGER = 0x00,
		TBL_TYPE_DETAIL = 0x01,
		TBL_TYPE_USER = 0x02,
		TBL_CNT_MAX = 0x03,

} AUR700_TblType;

// AUR700 Table State
typedef enum
{
		TBL_STATE_OFF = 0x00,
		TBL_STATE_ON = 0x01,

} AUR700_TblState;

// AUR700 Error Set
typedef enum
{
		AUR700_ERROR_DENY = 0xE0,
		AUR700_ERROR_INCORRECT = 0xE4,
		AUR700_ERROR_TBLEND = 0xE6,
		AUR700_ERROR_CRC = 0xE7,
		AUR700_ERROR_FAIL = 0xE8,
		AUR700_ERROR_REJECT = 0xE9,
		AUR700_ERROR_ADDRESS = 0xEE,

		AUR700_ERROR_UNKNOWN = 0xEF,

} AUR700_ErrSet;

// AUR700 Event Set
typedef enum
{
		AUR700_EVENT_ERROR = 0x15,

		AUR700_EVENT_TAGDATA = 0x30,
		AUR700_EVENT_BLEDEVICE = 0x31,
		AUR700_EVENT_BARCDATA = 0x32,
		AUR700_EVENT_PRESSBTN = 0x33,

} AUR700_EvtSet;

// AUR700 Error Event
typedef enum
{
		ERROR_EVENT_TIMEOUT = 0xE1,
		ERROR_EVENT_BLEDIS = 0xE2,

} AUR700_ErrEvt;

// AUR700 Loop Task
typedef enum
{
		B2E_TASK_NONE = 0x00,

		B2E_TASK_STOP = 0x01,
		B2E_TASK_SEARCH = 0x02,
		B2E_TASK_READALL = 0x03,
		B2E_TASK_WRITETAG = 0x04,
		B2E_TASK_READTAG = 0x05,
		B2E_TASK_LOCKTAG = 0x06,
		B2E_TASK_KILLTAG = 0x07,
		B2E_TASK_UNTRACETAG = 0x08,

		B2E_TEST_CONTWAVE = 0x10,
		B2E_TEST_MODULATION = 0x11,
		B2E_TEST_HOPPING = 0x12,

		CMD_TASK_QUERYTAGRSSI = 0x20,
		CMD_TASK_RDPRESSBTN = 0x21,
		CMD_TASK_READEPCTID = 0x22,
		CMD_TASK_WRITEEPC = 0x23,
		CMD_TASK_STARTLOGGER = 0x24,
		CMD_TASK_STOPLOGGER = 0x25,
		CMD_TASK_READBARCODE = 0x26,

		CMD_TEST_CONTWAVE = 0x40,
		CMD_TEST_MODULATION = 0x41,
		CMD_TEST_HOPPING = 0x42,

		KEY_TASK_INVENTORY = 0x80,
		KEY_TASK_STLOGGERCHECK = 0x81,
		KEY_TASK_STARTLOGGER = 0x82,
		KEY_TASK_STOPLOGGER = 0x83,

		KEY_TASK_RDLOGGERSTATE = 0x90,
		KEY_TASK_RDLOGGERDATA = 0x91,
		KEY_TASK_RDLOCATIONID = 0x92,
		KEY_TASK_READBARCODE = 0x93,

		KEY_TASK_WRLOCATIONID = 0xA0,
		KEY_TASK_WRITEBARCODE = 0xA1,

		KEY_TASK_OPENAREA = 0xB0,
		KEY_TASK_SETPASSWORD = 0xB1,
		KEY_TASK_CLRPASSWORD = 0xB2,

		KEY_TASK_DEBUGTEST = 0xF0,

} B2E_LoopTask;


/* ---------------------------------------------------------------------------*/
// Link Frequency
typedef enum
{
		LINK_FREQ_40 = 0x00,
		LINK_FREQ_80 = 0x01,
		LINK_FREQ_160 = 0x02,
		LINK_FREQ_256 = 0x03,
		LINK_FREQ_320 = 0x04,
		LINK_FREQ_640 = 0x05,
} SET_LinkFreq;

// RX Decode
typedef enum
{
		RX_DECODE_FM0 = 0x00,
		RX_DECODE_MI2 = 0x01,
		RX_DECODE_MI4 = 0x02,
		RX_DECODE_MI8 = 0x03,
} SET_RxDecode;

// Session
typedef enum
{
		SESSION_NUM_0 = 0x00,
		SESSION_NUM_1 = 0x01,
		SESSION_NUM_2 = 0x02,
		SESSION_NUM_3 = 0x03,
} SET_SessionNum;

// Pilot Tone
typedef enum
{
		PILOT_TONE_NONE = 0x00,
		PILOT_TONE_USED = 0x01,
} SET_PilotTone;

// Target Flag
typedef enum
{
		TARGET_FLAG_A = 0x00,
		TARGET_FLAG_B = 0x01,
		TARGET_FLAG_AB = 0x02,
} SET_TargetFlag;

// Tari Value
typedef enum
{
		TARI_VALUE_25U = 0x00,
		TARI_VALUE_12U5 = 0x01,
		TARI_VALUE_6U25 = 0x02,
} SET_TariValue;

// External Gain
typedef enum
{
		POWER_GAIN_0DBM = 0x00,
		POWER_GAIN_8DBM = 0x01,
		POWER_GAIN_16DBM = 0x02,
		POWER_GAIN_24DBM = 0x03,
} SET_PowerGain;

// Vibrator State
typedef enum
{
		VIBR_STATE_DIS = 0x00,
		VIBR_STATE_EN = 0x01,
} SET_VibrState;

// Buzzer State
typedef enum
{
		BUZZ_STATE_DIS = 0x00,
		BUZZ_STATE_EN = 0x01,
} SET_BuzzState;

// RF Strength
typedef enum
{
		RF_STRENGTH_LOW = 0x00,
		RF_STRENGTH_MED = 0x01,
		RF_STRENGTH_HI = 0x02,
} SET_RfStrength;

// User Identification
typedef enum
{
		USERID_BY_NONE = 0x00,
		USERID_BY_BARC = 0x01,
		USERID_BY_RFID = 0x02,
} SET_UserIdentify;

// Erase Logger Data
typedef enum
{
		ERASE_LOG_DIS = 0x00,
		ERASE_LOG_EN = 0x01,
} SET_EraseLog;

// Barcoder Selection
typedef enum
{
		BARC_SEL_1D = 0x00,
		BARC_SEL_2D = 0x01,
} SET_BarcSel;

// Lock Protection
typedef enum
{
		LOCK_PROT_NONE = 0x00,
		LOCK_ONLY_USER = 0x01,
		LOCK_SYS_USER = 0x02,
} SET_LockProt;

// LED Color
typedef enum
{
		LED_ALL_OFF = 0x00,
		LED_GREEN_ON = 0x01,
		LED_RED_ON = 0x02,
} CTRL_LedColor;

// Pressed Button
typedef enum
{
		PRESS_BTN_ACT = 0x00,
		PRESS_BTN_BACK = 0x01,
		PRESS_BTN_NEXT = 0x02,
} EVENT_PressBtn;

// Logger Code
typedef enum
{
		LOG_CODE_STARTLOG = 0x01,
		LOG_CODE_STOPLOG = 0x02,
} CTRL_LogCode;

// Barcoder State
typedef enum
{
		BARC_STATE_DIS = 0x00,
		BARC_STATE_EN = 0x01,
} CTRL_BarcState;

// BLE State
typedef enum
{
		BLE_STATE_DIS = 0x00,
		BLE_STATE_EN = 0x01,
} CTRL_BleState;


/* Exported types ------------------------------------------------------------*/
typedef struct
{
    u8 bPreamble;                                           // SOH: 0x01
	u8 bAddress;                                            // Device Address(Machine ID)
    u8 bCmdCode;                                            // Command code
    u8 bLength;                                             // Parameters length
	u8 bPayload[CMD_MAX_DATA_SIZE];													// Payload
    u16 uCrc16;																							// CRC
} CmdFormat_t;

typedef struct
{
    u8 bPreamble;                                           // SOH: 0x01
	u8 bAddress;                                            // Device Address(Machine ID)
    u8 bRepCode;	                                          // Reply code
    u8 bLength;                                             // Parameters length
	u8 bPayload[REP_MAX_DATA_SIZE];													// Payload
    u16 uCrc16;																							// CRC
} RepFormat_t;


#pragma anon_unions
typedef struct
{
    u16 uLen;
    union
    {
        CmdFormat_t tFormat;
        u8 bBuffer[sizeof(CmdFormat_t)];
    };
} CmdProtocol_t;

#pragma anon_unions
typedef struct
{
    u16 uLen;
    union
    {
        RepFormat_t tFormat;
        u8 bBuffer[sizeof(RepFormat_t)];
    };
} RepProtocol_t;

typedef struct
{
    u16 uTotalRecCnt;
    u16 uCurrRecNum;
    u8  bOpened;
} TblInformation_t;


typedef void (*pFunction)(void);


/* Exported variables --------------------------------------------------------*/
extern CmdProtocol_t cmd_parser;
extern RepProtocol_t rep_packer;
extern TblInformation_t tbl_info[];

extern u8 bPackBuf[];

extern const char sSTM_FWVer[];
extern char sBLE_FWVer[];

/* Exported functions --------------------------------------------------------*/
void CommandParser(u8 bData);
void CommandExecute(void);

void ResponseNak(void);
void ResponseNakData(const u8* pbData, u8 bDataLength);
void ResponseEvent(const u8* pbData, u8 bDataLength);
void ResponseData(u8 bCmdCode, const u8* pbData, u8 bDataLength);

void TimeOutHandler(u8 bCmdTask);

void ResetAURDevice(void);
s8 AURQueryTagRSSI(void);
s8 AURReadPressBtn(void);
s8 AURReadBarcode(void);
s8 AURReadEPCTID(void);
s8 AURStartLog(void);
s8 AURStopLog(void);
s8 AURWriteEPC(u16 uWordCount, u8* pbBankData);

void SendDevIDToNordic(void);
void SendDevNameToNordic(void);


#ifdef __cplusplus
}
#endif

#endif /* __CMD_PROTOCOL_GNET_H */
