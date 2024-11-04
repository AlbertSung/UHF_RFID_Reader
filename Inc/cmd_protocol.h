/**
  ******************************************************************************
  * @file    cmd_protocol.h
  * @author  Albert
  * @version V1.0.0
  * @date    11-July-2017
  * @brief   Header for cmd_protocol.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CMD_PROTOCOL_H
#define __CMD_PROTOCOL_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"

/* Defined constants ---------------------------------------------------------*/
#define CMD_MAX_DATA_SIZE					256

#define CMD_HEADER_SOH						0x01

#define CMD_HEADER_LEN						1
#define CMD_ADDRESS_LEN						0
#define CMD_LENGTH_LEN						1
#define CMD_OPCODE_LEN						1
#define CMD_CRC16_LEN							2

#define CMD_BEFORE_DATA_SIZE			(CMD_HEADER_LEN + CMD_LENGTH_LEN + CMD_OPCODE_LEN)	// 3
#define CMD_LENGTH_OPCODE_LEN			(CMD_LENGTH_LEN + CMD_OPCODE_LEN)		// 2
#define CMD_FULL_NO_DATA_LEN			(CMD_HEADER_LEN + CMD_LENGTH_OPCODE_LEN + CMD_CRC16_LEN)		// 5


#define REP_MAX_DATA_SIZE					256

#define REP_HEADER_LEN						1
#define REP_LENGTH_LEN						1
#define REP_OPCODE_LEN						1
#define REP_RESULT_LEN						0
#define REP_CRC16_LEN							2

#define REP_LENGTH_OPCODE_LEN			(REP_LENGTH_LEN + REP_OPCODE_LEN)		// 2
#define REP_FULL_NO_DATA_LEN			(REP_HEADER_LEN + REP_LENGTH_OPCODE_LEN + REP_CRC16_LEN)		// 5


#define DEF_CRC_PRESET            0xFFFF
#define DEF_CRC_POLYNOM           0xA001


#define CMD_REPLY_ACK            	0x80		// Acknowledged reply with (operation code | 0x80)
#define CMD_REPLY_NAK            	0xFF		// Non-acknowledge reply with always 0xFF


#define EVENT_CNT_NUM_DATA_LEN		5				// the length of Packet Count(2) + Packet Number(2) + Bank Data Length(1)


#define MAX_READTAG_LEN						200//16	// the maximum of whole packet including bank data is 255 bytes
#define MAX_PACKET_CNT						65535		// 0xFFFF for reading out all bank data when word length is zero

#define MAX_TIME_OUT							65535		// 0xFFFF (ms) for 2 bytes parameters in SW command

#define MAX_LISTEN_TIME						10000
#define MAX_IDLE_TIME							10000
#define MAX_MAX_ALLOCATION				10000
#define MAX_RSSI_THRESHOLD	 			(-20)
#define MIN_RSSI_THRESHOLD				(-100)
#define MAX_OUTPUT_POWER		 			(0)
#define MIN_OUTPUT_POWER					(-19)



/* Exported constants --------------------------------------------------------*/
// B2E Command Set
typedef enum
{
		B2E_FW_VERSION = 0x01,
		B2E_WRITE_SETTING = 0x04,
		B2E_READ_SETTING = 0x05,
		B2E_GOTO_ISP = 0x09,
		B2E_REFLECTED_POWER = 0x0A,
		B2E_RADIO_MODE = 0x0B,

		B2E_SEARCH_ALL = 0x20,
		B2E_READ_ALL = 0x21,
		B2E_WRITE_TAG = 0x22,
		B2E_STOP_SEARCH = 0x23,
		B2E_LOCK_TAG = 0x24,
		B2E_KILL_TAG = 0x25,
		B2E_READ_TAG = 0x26,
		B2E_UNTRACE_TAG = 0x27,

#ifdef CISC_TEST
		B2E_GEN2_SETTING = 0x66,	// for CISC testing
		B2E_MODULATION = 0x77,		// for CISC testing
#endif

		AUR700_TEST = 0x99,				// for AUR700 testing

} B2E_CmdSet;

// B2E Setting Code
typedef enum
{
		// Device settings
		SETTING_SERIAL_NUM = 0x00,

		// Frequency settings
		SETTING_REGION_ID = 0x10,
		SETTING_START_FREQ = 0x11,
		SETTING_END_FREQ = 0x12,
		SETTING_INC_FREQ = 0x13,

		SETTING_2ND_SPEC = 0x18,
		SETTING_START2_FREQ = 0x19,
		SETTING_END2_FREQ = 0x1A,
		SETTING_INC2_FREQ = 0x1B,

		SETTING_LISTEN_TIME = 0x14,
		SETTING_IDLE_TIME = 0x15,
		SETTING_MAX_ALLOCATION = 0x16,
		SETTING_RSSI_THRESHOLD = 0x17,

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
		SETTING_TARGET_FLAG = 0x35,
		SETTING_TARI_VALUE = 0x36,

} B2E_SetCode;


// B2E Event Set
typedef enum
{
		B2E_EVENT_ERROR = 0x60,
		B2E_EVENT_INVENTORY = 0x61,
		B2E_EVENT_READTAG = 0x62,

} B2E_EvtSet;


// B2E Error Set
typedef enum
{
		B2E_ERROR_SUCCESS = 0x00,
		B2E_ERROR_INCORRECT = 0x01,
		B2E_ERROR_REJECT = 0x02,
		B2E_ERROR_DENY = 0x03,
		B2E_ERROR_FAIL = 0x04,
		B2E_ERROR_TBLEND = 0x05,
		B2E_ERROR_TIMEOUT = 0x06,

		B2E_ERROR_UNKNOWN = 0xFF,

} B2E_ErrSet;


// B2E Loop Task
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

// Q Begin
typedef enum
{
		Q_BEGIN_MIN = 0x00,
		Q_BEGIN_MAX = 0x08,
} SET_QBegin;

// Session
typedef enum
{
		SESSION_NUM_0 = 0x00,
		SESSION_NUM_1 = 0x01,
		SESSION_NUM_2 = 0x02,
		SESSION_NUM_3 = 0x03,
//	SESSION_NUM_SL = 0x04,
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
//	TARGET_FLAG_BA = 0x03,
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


/* Exported types ------------------------------------------------------------*/
typedef struct
{
    u8 bPreamble;                                           // SOH: 0x01
    u8 bLength;                                             // Packet length from SOH to CRC
    u8 bCmdCode;                                            // Command code
	u8 bPayload[CMD_MAX_DATA_SIZE];													// Payload
    u16 uCrc16;																							// CRC
} CmdFormat_t;

typedef struct
{
    u8 bPreamble;                                           // SOH: 0x01
    u8 bLength;                                             // Packet length from SOH to CRC
    u8 bRepCode;	                                          // Reply code
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


typedef void (*pFunction)(void);


/* Exported variables --------------------------------------------------------*/
extern CmdProtocol_t cmd_parser;
extern RepProtocol_t rep_packer;
extern u8 bPackBuf[];

extern const char sFW_Ver[];
extern unsigned char sTest_Str[];

/* Exported functions --------------------------------------------------------*/
void CommandParser(u8 bData);
void CommandExecute(void);
void ResponseEvent(u8 bEvent, const u8* pbData, u8 bDataLength);
void ResponseAck(u8 bCmdCode);
void ResetB2EDevice(void);



#ifdef __cplusplus
}
#endif

#endif /* __CMD_PROTOCOL_H */
