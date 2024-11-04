/**
  ******************************************************************************
  * @file    gen2_config.h
  * @author  Albert
  * @version V1.0.0
  * @date    19-June-2017
  * @brief   Header for gen2_config.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GEN2_CONFIG_H
#define __GEN2_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"
#include "gen2.h"


/* Exported definitions ------------------------------------------------------*/
#define GEN2_QRY_TARGET_MASK								0x01
#define GEN2_QRY_TARGET_BOTH								0x80

#define MAX_FREQUENCY_LIST_NUM							50

#define AUR700_DEFAULT  										"AUR700"

/* Exported functions --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

	 
typedef struct
{
  u8 year;
  u8 month;
  u8 date;
  u8 week;
  u8 hour;
  u8 minute;
  u8 second;  
} __attribute__ ((aligned(1))) rtcParameter_t;

typedef struct
{
  u8 ucTotalNum;
  u8 ucItem[8];
} __attribute__ ((aligned(1))) menu_t;

typedef struct
{
    u8 bFrequencyHigh;
    u8 bFrequencyMid;
    u8 bFrequencyLow;
} __attribute__ ((aligned(1))) uhfFrequency_t;


#pragma anon_unions
typedef struct
{
    u8 bRegionID;		// 0xFF Default, 0xFE User Define

    u16 uAllocationTime;
    u16 uListeningTime;
    u16 uIdleTime;

    s8 cRSSIThreshold;

		u8 bNumberOfFrequency;		// total
		u8 bNumberOfFrequencyOfFirstSpectrum;

    u8 ucSecondSpectrum;

		uhfFrequency_t tFrequencies_2;

		uhfFrequency_t ucIncrementFreq;
    uhfFrequency_t ucIncrementFreq_2;  

		uhfFrequency_t tFrequencies[MAX_FREQUENCY_LIST_NUM];
} uhfFrequencyList_t;


#define UHF_MAX_REGISTERS       255
typedef union
{
    struct
    {
        s8 cRFTxPower;
        s8 cRFRxSensitivity;
        u8 bPowerGain;

				u8 bRxDecode;
        u8 bGen2Session;
        u8 bGen2LinkFrequency;
        u8 bGen2Trext;
				u8 bGen2Target;
        u8 bGen2QBegin;
        u8 bGen2Tari;

        uhfFrequencyList_t tFrequencyList;

        u8 bAutoMode;

        u8 g_bRadioTestModeAction;

        u8 ucDeviceID[2];
        u8 ucUserIdentify;
        u16 ucCompanyID;
        u8 ucLocationID[2];
        u8 ucDeviceName[10];

        u8 ucVibrEnable;
        u8 ucBuzzEnable;

        u16 ulScanTagTimeout;
        u32 ulEnterIdleTime;

        u8 ucRFStrg;
        u8 ucEraseLoggerData;
        u8 ucBLEMacAddress[6];
        u8 ucLockProtect;
        u8 uBarcode_Selection;

        rtcParameter_t tRtc;
        menu_t tMenu;

    } s2;
    u8 bRegisters[UHF_MAX_REGISTERS];
} uhfRegisters_t;



/* Exported constants --------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/
extern uhfRegisters_t g_tRegisters;
extern volatile const uhfRegisters_t DEFALUT_UHF_REGISTERS;



#ifdef __cplusplus
}
#endif

#endif /* __GEN2_CONFIG_H */



