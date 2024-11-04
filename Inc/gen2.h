/**
  ******************************************************************************
  * @file    gen2.h
  * @author  Albert
  * @version V1.0.0
  * @date    14-June-2017
  * @brief   Header for gen2.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GEN_2_H
#define __GEN_2_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"
#include "as3993.h"
#include "platform.h"
#include "b2e_errno.h"

	 
/* Defined constants ---------------------------------------------------------*/

// Definition for the maximal length
#define EPC_LENGTH                          62          //32// number of bytes for EPC, standard allows up to 62 bytes
#define TID_LENGTH                          12          // Definition for the maximal TID length in an inventory round in bytes
#define PC_LENGTH                           2           // Definition for the PC length
#define CRC_LENGTH                          2           // Definition for the CRC length


#define GEN2_ACCESS_PASSWORD_WORD_LENGTH    2
#define GEN2_ACCESS_PASSWORD_BYTE_LENGTH    (GEN2_ACCESS_PASSWORD_WORD_LENGTH * 2)


#if RUN_ON_AS3993
#define GEN2_USE_AUTO_ACK_DEFAULT           0
#define GEN2_QUERY_MIN_Q                    1
#define GEN2_QUERY_DEFAULT_Q                1
#define GEN2_QUERY_MAX_Q                    10//5//3		// 5 => 32 tags
#else
#define GEN2_USE_AUTO_ACK_DEFAULT           1
//#define INVENTORY_USE_SELECT
#define GEN2_QUERY_TARGET_XCHANGE
#define GEN2_QUERY_MIN_Q                    0
#define GEN2_QUERY_DEFAULT_Q                0
#define GEN2_QUERY_MAX_Q                    0
#endif

#define GEN2_QUERY_TO_BREAK_BY_SAME_Q_COUNT	5//3

#if RUN_ON_AS3980
#define GEN2_TIMEOUT                        (60u)
#else
#define GEN2_TIMEOUT                        (10u)
#endif

#define GEN2_LONG_ACCESS_TIMEOUT            (0xFF)


#define HOPPING_OFFSET_DIR_TO_LOW           0
#define HOPPING_OFFSET_DIR_TO_HIGH          1

#define GEN2_RX_CENTER_OFFSET_FREQUENCY_KHZ 100     // 100KHz = TxFreq-RxFreq
#define GEN2_RX_LBT_ADJUST_FREQUENCY_KHZ    200
#define GEN2_RX_LOW_OFFSET_FREQUENCY_KHZ    (GEN2_RX_CENTER_OFFSET_FREQUENCY_KHZ+GEN2_RX_LBT_ADJUST_FREQUENCY_KHZ)
#define GEN2_RX_HIGH_OFFSET_FREQUENCY_KHZ   (GEN2_RX_CENTER_OFFSET_FREQUENCY_KHZ+GEN2_RX_LBT_ADJUST_FREQUENCY_KHZ)


// Protocol configuration settings
#define GEN2_LF_40                          0       // link frequency 40 kHz
#define GEN2_LF_80                          0       // link frequency 80 kHz
#define GEN2_LF_160                         6       // link frequency 160 kHz
#define GEN2_LF_213                         8       // link frequency 213 kHz
#define GEN2_LF_256                         9       // link frequency 256 kHz
#define GEN2_LF_320                         12      // link frequency 320 kHz
#define GEN2_LF_640                         15      // link frequency 640 kHz

// Rx coding values
#define GEN2_COD_FM0                        0       // FM coding for rx
#define GEN2_COD_MILLER2                    1       // MILLER2 coding for rx
#define GEN2_COD_MILLER4                    2       // MILLER4 coding for rx
#define GEN2_COD_MILLER8                    3       // MILLER8 coding for rx


// EPC Memory Banks
#define MEM_RES                             0x00        // Definition for EPC tag memory bank: reserved
#define MEM_EPC                             0x01        // Definition for EPC tag memory bank: EPC memory
#define MEM_TID                             0x02        // Definition for EPC tag memory bank: TID
#define MEM_USER                            0x03        // Definition for EPC tag memory bank: user


// EPC Select Target
#define GEN2_IINV_S0                        0x00        // Inventoried (S0) // Definition for inventory: Inventoried (S0)
#define GEN2_IINV_S1                        0x01        // 001: Inventoried (S1)   // Definition for inventory: 001: Inventoried (S1)
#define GEN2_IINV_S2                        0x02        // 010: Inventoried (S2)    // Definition for inventory: 010: Inventoried (S2)
#define GEN2_IINV_S3                        0x03        // 011: Inventoried (S3)    // Definition for inventory: 011: Inventoried (S3)
#define GEN2_IINV_SL                        0x04        // 100: SL  // Definition for inventory: 100: SL

// Select Command Action
#define GEN2_ACT_MATCH_SEL_A_NM_UNSEL_B     0
#define GEN2_ACT_MATCH_SEL_A                1
#define GEN2_ACT_NM_UNSEL_B                 2
#define GEN2_ACT_MATCH_NEGAGE_SEL_A_B       3
#define GEN2_ACT_MATCH_UNSEL_B_NM_SEL_A     4
#define GEN2_ACT_MATCH_UNSEL_B              5
#define GEN2_ACT_NM_SEL_A                   6
#define GEN2_ACT_NM_NEGAGE_SEL_A_B          7


// Query Target
#define GEN2_QRY_TARGET_A                   0
#define GEN2_QRY_TARGET_B                   1


// Query Select
#define GEN2_QRY_SEL_ALL                    0
#define GEN2_QRY_SEL_ALL_1                  1
#define GEN2_QRY_SEL_UNSEL                  2
#define GEN2_QRY_SEL_SEL                    3


// Gen2 Data Length
#define GEN2_HEADER_BIT_LENGTH              0x01        // 1 bit, It's break byte
#define GEN2_HANDLE_LENGTH                  0x02
#define GEN2_CRC16_LENGTH                   0x02


#define TAG_FLAG_NONE                       (0x00u)
#define TAG_FLAG_NEW_TAG                    (0x01u)
#define TAG_FLAG_SENT_TAG                   (0x02u)


// EPC Commands
#define EPC_QUERYREP                				0x00            // Definition for queryrep EPC command
#define EPC_ACK                     				0x01            // Definition for acknowldege EPC command
#define EPC_QUERY                   				0x08            // Definition for query EPC command
#define EPC_QUERYADJUST             				0x09            // Definition for query adjust EPC command
#define EPC_SELECT                  				0x0A            // Definition for select EPC command
#define EPC_NAK                     				0xC0            // Definition for not acknowldege EPC command
#define EPC_REQRN                   				0xC1            // Definition for request new RN16 or handle
#define EPC_READ                    				0xC2            // Definition for read EPC command
#define EPC_WRITE                   				0xC3            // Definition for write EPC command
#define EPC_KILL                    				0xC4            // Definition for kill EPC command
#define EPC_LOCK                    				0xC5            // Definition for lock EPC command
#define EPC_ACCESS                  				0xC6            // Definition for access EPC command
#define EPC_BLOCKWRITE              				0xC7            // Definition for blockwrite EPC command
#define EPC_BLOCKERASE              				0xC8            // Definition for blockerase EPC command
#define EPC_BLOCKPERMALOCK          				0xC9            // Definition for block permalock EPC command
#define EPC_UNTRACE_HIGH										0xE2						// Definition for untraceable EPC command (high byte)
#define EPC_UNTRACE_LOW											0x00						// Definition for untraceable EPC command (low byte)

//
#define READ_TAG_BUFFER_SIZE        				208    // Select: 20, 40, 80, 160, 192, Max 208
#define READ_TAG_BUFFER_WORD_SIZE   				(READ_TAG_BUFFER_SIZE / 2)
#define READ_TAG_BUFFER_FULL_SIZE   				(GEN2_HEADER_BIT_LENGTH + GEN2_HANDLE_LENGTH + GEN2_CRC16_LENGTH + READ_TAG_BUFFER_SIZE)

#define GEN2_READ_TAG_RETRY_TIME            5
#define GEN2_WRITE_TAG_RETRY_TIME           10
#define GEN2_ERASE_TAG_RETRY_TIME           3
#define GEN2_ACCESS_TAG_RETRY_TIME          3
#define GEN2_KILL_TAG_RETRY_TIME          	3
#define GEN2_LOCK_TAG_RETRY_TIME          	3
#define GEN2_UNTRACE_TAG_RETRY_TIME					3
#define GEN2_SENSITIVITY_RETRY_TIME					10

#define GEN2_MOST_SENSITIVITY								(-90)
#define GEN2_LEAST_SENSITIVITY							(-54)
#define GEN2_STEP_SENSITIVITY								3//1

#define GEN2_INVENTORY_END_TIME							5//10//50				// for 10s timeout verification

#define GEN2_ALOHA_QFP_INC									0.45//0.35
#define GEN2_ALOHA_QFP_DEC									0.2//0.15


/* Exported types ------------------------------------------------------------*/
#pragma anon_unions
typedef struct
{
    u32 ulLiveTicks;
    u8 bFlag;
    u8 bIQValue;
    s8 cRFU;
    s8 cRSSI;
    u8 bEPCLen;
    union
    {
        u8 bFullEPCs[(PC_LENGTH + EPC_LENGTH)];
        struct
        {
            u8 bPCs[PC_LENGTH];
            u8 bEPCs[EPC_LENGTH];
        };
    };
} TagInfo_t;

typedef union
{
    struct
    {
        u8 bKillMask:2;
        u8 bAccessMask:2;
        u8 bEPCMask:2;
        u8 bTIDMask:2;
        u8 bUserMask:2;
        u8 bKillAction:2;
        u8 bAccessAction:2;
        u8 bEPCAction:2;
        u8 bTIDAction:2;
        u8 bUserAction:2;
        u8 :4;
    };
    u8 bBuffer[3];
} LockTagActions_t;

typedef struct
{
    union
    {
        u8 bPasswords[GEN2_ACCESS_PASSWORD_BYTE_LENGTH];
        u16 uPasswords[GEN2_ACCESS_PASSWORD_BYTE_LENGTH / 2];
        u32 ulPassword;
    };
    u8 bTIDLen;
    u8 bTIDs[TID_LENGTH * 4];
    union
    {
        u16 uHandle;
        u8 bHandles[sizeof(u16)];
    };
    TagInfo_t tTag;
} AccessTagInfo_t;

typedef struct
{
    u8 bQueryStatus;
    u8 bCurrentQ;
    u8 bQNoChangeCount;
    u16 uCollisions;
    u16 uAckCount;			// testing for Acknowledged count
    u16 uNorespCount;		// testing for No-response count
    u16 uQueryCount;
		s8 cRxSensitivity;
} QueryInfo_t;

typedef union
{
    struct
    {   // Low Byte
        u8 bSelect:2;
        u8 bTRext:1;
        u8 bMiller:2;
        u8 bTRcalDivideRatio:1;
        u8 :2;
        // High Byte
        u8 :1;
        u8 bQBegin:4;
        u8 bTarget:1;
        u8 bSession:2;
    };
    u8 bBytes[2];
    u16 uValue;
} __attribute__ ((aligned(2))) gen2QueryParams_t;

typedef union
{
    struct
    {
        u8 bTarget:3;
        u8 bAction:3;
        u8 bMemBank:2;
        u32 ulPointer;
        u8 bLength;
        u8 bMasks[32];			// Gen2 only allows max 255 bits (about 32 bytes) length of Mask
        u8 :7;
        u8 bTruncation:1;
    };
    u8 bBuffer[32+7];
} gen2SelectParams_t;

typedef union
{
    u16 uRN16;
    u8 bRN16s[sizeof(u16)];
} RN16_t;

typedef struct
{
		u8 bRFU:2;
		u8 bUBit:1;
		u8 bUserHide:1;
		u8 bRange:2;
		u8 bTidHide:2;
		union
		{
				struct
				{
						u8 bEpcLen:5;
						u8 bEpcHide:1;
				};
				u8 bEpcAction:6;
		};
		u8 :2;
} __attribute__ ((aligned(2))) UntraceAction_t;


/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern AccessTagInfo_t g_tCurrentTag;

extern u16 m_uFoundTagsNum;
extern s8 m_cRxSensitivity;

extern u32 g_ulFrequency;

extern u8 m_bIsAccessTag;
extern u8 g_bIsHoppingFrequency;

extern u8 bQueryPlusOnes;

extern u16 g_bEndInventoryNum;
extern u8 g_bEndInventoryRound;

extern LockTagActions_t g_tLockActions;
extern UntraceAction_t g_tUntraceAction;


/* Exported functions --------------------------------------------------------*/
void gen2Initialize(void);
void gen2ResetInventoryRound(void);
s8 gen2Inventory(void);
s8 gen2SelectTagTID(AccessTagInfo_t* ptTag);
s8 gen2SelectTagBank(u8 bSelBank, u32 ulByteAddress, u8 bMaskByte, u8* pbBankMask);
s8 gen2ReadTagTID(AccessTagInfo_t* ptTag);
s8 gen2ReadTag(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8 bWordCount, u8* pbBuffer, u16* puLength);
s8 gen2WriteTag(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8 bWordCount, u8* pbBuffer, u16* puLength);
s8 gen2BlockErase(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8 bWordCount);
s8 gen2LockTag(AccessTagInfo_t* ptTag, LockTagActions_t* ptLockActions);
s8 gen2AccessTag(AccessTagInfo_t* ptTag);
s8 gen2KillTag(AccessTagInfo_t* ptTag, u8 bRFU);
s8 gen2UntraceTag(AccessTagInfo_t* ptTag, UntraceAction_t* ptUntraceAction);
void checkHoppingFrequencies(void);

u16 insertByteToBytes(u8* pbDestBuffer, u16 uDestBitLen, u16 uOffset, u16 uLen, u8 bValue);
s8 uhfTxRxGen2Bytes(u8 bCommand, const u8* pbTxBuf, u16 uTxBitLen, u8* pbRxBuf, u16* puRxBitLen, u8 bTimeout, u8 bFollowCmd);


/* Defined functions ---------------------------------------------------------*/
#define ufhReadFifoBytes(pbBuffer, bLength)  																		uhfReadBytes(AS3993_REG_FIFO, pbBuffer, bLength)

#define uhfWriteByte(bAddress, bValue)                                          as3993WriteByte(bAddress, bValue)
#define uhfWriteBytes(bAddress, pbBuffers, bLength)                             as3993WriteBytes(bAddress, pbBuffers, bLength)
#define uhfWriteCmdWithBytes(bCommand, bAddress, pbBuffer, bLength)             as3993WriteCmdWithBytes(bCommand, bAddress, pbBuffer, bLength)
#define uhfWriteCmd(bCommand)                                                   as3993WriteCmd(bCommand)
#define uhfReadBytes(bAddress, pbBuffer, bLength)                               as3993ReadBytes(bAddress, pbBuffer, bLength)
#define uhfReadByte(bAddress)                                                   as3993ReadByte(bAddress)

#define uhfResponse                                                             as3993Response
#define uhfGetResponse()                                                        as3993GetResponse()
#define uhfClrResponse()                                                        as3993ClrResponse()
#define uhfClrResponseMask(uMask)                                               as3993ClrResponseMask(uMask)

#define uhfWaitForResponse(uWaitMask)                                           as3993WaitForResponse(uWaitMask)
#define uhfWaitForResponseTimed(uWaitMask, ulCounter)                           as3993WaitForResponseTimed(uWaitMask, ulCounter)


#define uhfMeasureMaxRSSI(ListeningTime, RSSIThreshold)                         as3993MeasureMaxRSSI(ListeningTime, RSSIThreshold)
#define uhfGetRssi(RSSISelect, IQValue)                                         as39932GetRssi(RSSISelect, IQValue)

#define uhfSetSensitivity(RFSensitivity)                                        as3993SetSensitivity(RFSensitivity)

#define uhfGetReaderPowerMode()                                                 as3993GetReaderPowerMode()
#define uhfSetReaderPowerMode(bPowerMode)                                       as3993SetReaderPowerMode(bPowerMode)
#define uhfSetAntennaPower(PowerOn)                                             as3993SetAntennaPower(PowerOn)

#define uhfGetMixerIQLevel()																										as3993GetMixerIQLevel()
#define uhfGetRxMixerGainValue()																								as3993GetRxMixerGainValue()

#define uhfIsHaveRFPowerOff()                                                   as3993IsHaveRFPowerOff()
#define uhfClearHaveRFPowerOff()                                                as3993ClearHaveRFPowerOff()

#define uhfIsValidFrequency(ulFrequency)																				as3993IsValidFrequency(ulFrequency)

#define uhfSetAutoAckMode(AutoAckMode)                                          as3993SetAutoAckMode(AutoAckMode)
#define uhfSetGen2Session(Session)                                              as3993SetGen2Session(Session)
#define uhfSetGen2Protocol()                                                    as3993SetGen2Protocol()

#define uhfSetBaseFrequency(BaseFrequency)                                      as3993SetBaseFrequency(AS3993_REG_PLLMAIN1, BaseFrequency, NULL)
#define uhfSetRFTxPower(RFPower)                                                um900SetRFTxPower(RFPower)

#define gen2IsHoppingFrequency()            																		g_bIsHoppingFrequency
#define gen2EnableHoppingFrequency()        																		g_bIsHoppingFrequency = TRUE
#define gen2DisableHoppingFrequency()       																		g_bIsHoppingFrequency = FALSE

#define uhfInitialize()                                                         as3993Initialize()




#define getFrequency(Index)    \
    ((((u32)g_tRegisters.s2.tFrequencyList.tFrequencies[Index].bFrequencyHigh)<<16) \
    | (((u32)g_tRegisters.s2.tFrequencyList.tFrequencies[Index].bFrequencyMid)<<8) \
    | ((u32)g_tRegisters.s2.tFrequencyList.tFrequencies[Index].bFrequencyLow))

#define setFrequency(Index, Frequency)     \
{   \
    g_tRegisters.s2.tFrequencyList.tFrequencies[Index].bFrequencyHigh=((Frequency>>16) & 0xFF);   \
    g_tRegisters.s2.tFrequencyList.tFrequencies[Index].bFrequencyMid)=((Frequency>>8) & 0xFF); \
    g_tRegisters.s2.tFrequencyList.tFrequencies[Index].bFrequencyLow=(Frequency & 0xFF);    \
}



#ifdef __cplusplus
}
#endif

#endif /* __GEN_2_H */


