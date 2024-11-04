/**
  ******************************************************************************
  * @file    gen2_config.c
  * @author  Albert
  * @version V1.0.0
  * @date    19-June-2017
  * @brief   Gen2 configuration
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "gen2_config.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uhfRegisters_t g_tRegisters;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define FrequencyItem(Frequency)    ((uhfFrequency_t){.bFrequencyHigh=((Frequency>>16) & 0xFF), .bFrequencyMid=((Frequency>>8) & 0xFF), .bFrequencyLow=(Frequency & 0xFF)})


/******************************************************************************/

#define _DEFAULT_PROFILE_USA_
//#define _DEFAULT_PROFILE_EUROPE_
//#define _DEFAULT_PROFILE_EUROPE_USA_



// Default Gen2 configuration, this is the only supported configuration for AS3980.
// TARI_25, GEN2_LF_40, GEN2_COD_MILLER8, TREXT_ON, SEL_ALL, GEN2_SESSION_S0, TARGET_A
// Start value for Q for Gen2 inventory rounds, this is the only supported value for AS3980.
// Q Begin = 0;




/**
  * @brief  This union/structure is for registers' default settings
  * @param  None
  * @retval None
  */
volatile const uhfRegisters_t /*__attribute__((section(".DEFALUT_UHF_REGISTERS")))*/ DEFALUT_UHF_REGISTERS=
{
    {
        .cRFTxPower = (-9),
        .cRFRxSensitivity = (-62),
        .bPowerGain = PA_GAIN16_SEL,

#if RUN_ON_AS3980
        .bRxDecode = GEN2_COD_MILLER8,
        .bGen2Session = AS3993_REG3C_GEN2_SESSION_S1S0_0,
        .bGen2LinkFrequency = GEN2_LF_40,
        .bGen2Trext = TREXT_ON,
				.bGen2Target = GEN2_QRY_TARGET_A,
        .bGen2QBegin = 0,
        .bGen2Tari = TARI_25,
#else
        .bRxDecode = GEN2_COD_MILLER4,
        .bGen2Session = AS3993_REG3C_GEN2_SESSION_S1S0_0,
        .bGen2LinkFrequency = GEN2_LF_256,
        .bGen2Trext = TREXT_ON,
				.bGen2Target = GEN2_QRY_TARGET_A,
        .bGen2QBegin = 1,
        .bGen2Tari = TARI_25,
#endif

#ifdef _DEFAULT_PROFILE_EUROPE_USA_
        .tFrequencyList.bRegionID = 0xFF,
        .tFrequencyList.uAllocationTime = ???,
#elif defined(_DEFAULT_PROFILE_EUROPE_)
        .tFrequencyList.bRegionID = 0x01,
        .tFrequencyList.uAllocationTime = 10000,
#else		// defined(_DEFAULT_PROFILE_USA_)
        .tFrequencyList.bRegionID = 0x03,
        .tFrequencyList.uAllocationTime = 400,
#endif

				.tFrequencyList.uListeningTime = 1,
				.tFrequencyList.uIdleTime = 0,
        .tFrequencyList.cRSSIThreshold = (-40),
				.tFrequencyList.ucSecondSpectrum = 0,

#ifdef _DEFAULT_PROFILE_EUROPE_USA_

				.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum = 8,

				.tFrequencyList.tFrequencies[0] = FrequencyItem(865700),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[1] = FrequencyItem(914000),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[2] = FrequencyItem(866300),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[3] = FrequencyItem(914500),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[4] = FrequencyItem(866700),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[5] = FrequencyItem(915000),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[6] = FrequencyItem(867300),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[7] = FrequencyItem(915500),  //u32 ulFrequency:24;

        .tFrequencyList.bNumberOfFrequency = 8,

#elif 	defined(_DEFAULT_PROFILE_EUROPE_)

				.tFrequencyList.bNumberOfFrequencyOfFirstSpectrum = 4,
				.tFrequencyList.ucIncrementFreq = {.bFrequencyHigh=((600>>16) & 0xFF), .bFrequencyMid=((600>>8) & 0xFF), .bFrequencyLow=(600 & 0xFF)},

        .tFrequencyList.tFrequencies[0] = {.bFrequencyHigh=((865700>>16) & 0xFF), .bFrequencyMid=((865700>>8) & 0xFF), .bFrequencyLow=(865700 & 0xFF)},//FrequencyItem(865700),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[1] = {.bFrequencyHigh=((866300>>16) & 0xFF), .bFrequencyMid=((866300>>8) & 0xFF), .bFrequencyLow=(866300 & 0xFF)},//FrequencyItem(866300),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[2] = {.bFrequencyHigh=((866900>>16) & 0xFF), .bFrequencyMid=((866900>>8) & 0xFF), .bFrequencyLow=(866900 & 0xFF)},//FrequencyItem(866700),  //u32 ulFrequency:24;
        .tFrequencyList.tFrequencies[3] = {.bFrequencyHigh=((867500>>16) & 0xFF), .bFrequencyMid=((867500>>8) & 0xFF), .bFrequencyLow=(867500 & 0xFF)},//FrequencyItem(867300),   //u32 ulFrequency:24;

				.tFrequencyList.ucIncrementFreq_2 = {.bFrequencyHigh=((600>>16) & 0xFF), .bFrequencyMid=((600>>8) & 0xFF), .bFrequencyLow=(600 & 0xFF)},
				.tFrequencyList.bNumberOfFrequency = 4,

#else 	//defined(_DEFAULT_PROFILE_USA_)

        .tFrequencyList.bNumberOfFrequencyOfFirstSpectrum = 50,
				.tFrequencyList.ucIncrementFreq = {.bFrequencyHigh=((500>>16) & 0xFF), .bFrequencyMid=((500>>8) & 0xFF), .bFrequencyLow=(500 & 0xFF)},

        .tFrequencyList.tFrequencies[0] = {.bFrequencyHigh=((902750>>16) & 0xFF), .bFrequencyMid=((902750>>8) & 0xFF), .bFrequencyLow=(902750 & 0xFF)},//FrequencyItem(902750),
        .tFrequencyList.tFrequencies[1] = {.bFrequencyHigh=((903250>>16) & 0xFF), .bFrequencyMid=((903250>>8) & 0xFF), .bFrequencyLow=(903250 & 0xFF)},//FrequencyItem(903250),
        .tFrequencyList.tFrequencies[2] = {.bFrequencyHigh=((903750>>16) & 0xFF), .bFrequencyMid=((903750>>8) & 0xFF), .bFrequencyLow=(903750 & 0xFF)},//FrequencyItem(903750),
        .tFrequencyList.tFrequencies[3] = {.bFrequencyHigh=((904250>>16) & 0xFF), .bFrequencyMid=((904250>>8) & 0xFF), .bFrequencyLow=(904250 & 0xFF)},//FrequencyItem(904250),
        .tFrequencyList.tFrequencies[4] = {.bFrequencyHigh=((904750>>16) & 0xFF), .bFrequencyMid=((904750>>8) & 0xFF), .bFrequencyLow=(904750 & 0xFF)},//FrequencyItem(904750),
        .tFrequencyList.tFrequencies[5] = {.bFrequencyHigh=((905250>>16) & 0xFF), .bFrequencyMid=((905250>>8) & 0xFF), .bFrequencyLow=(905250 & 0xFF)},//FrequencyItem(905250),
        .tFrequencyList.tFrequencies[6] = {.bFrequencyHigh=((905750>>16) & 0xFF), .bFrequencyMid=((905750>>8) & 0xFF), .bFrequencyLow=(905750 & 0xFF)},//FrequencyItem(905750),
        .tFrequencyList.tFrequencies[7] = {.bFrequencyHigh=((906250>>16) & 0xFF), .bFrequencyMid=((906250>>8) & 0xFF), .bFrequencyLow=(906250 & 0xFF)},//FrequencyItem(906250),
        .tFrequencyList.tFrequencies[8] = {.bFrequencyHigh=((906750>>16) & 0xFF), .bFrequencyMid=((906750>>8) & 0xFF), .bFrequencyLow=(906750 & 0xFF)},//FrequencyItem(906750),
        .tFrequencyList.tFrequencies[9] = {.bFrequencyHigh=((907250>>16) & 0xFF), .bFrequencyMid=((907250>>8) & 0xFF), .bFrequencyLow=(907250 & 0xFF)},//FrequencyItem(907250),

				.tFrequencyList.tFrequencies[10] = {.bFrequencyHigh=((907750>>16) & 0xFF), .bFrequencyMid=((907750>>8) & 0xFF), .bFrequencyLow=(907750 & 0xFF)},//FrequencyItem(907750),
        .tFrequencyList.tFrequencies[11] = {.bFrequencyHigh=((908250>>16) & 0xFF), .bFrequencyMid=((908250>>8) & 0xFF), .bFrequencyLow=(908250 & 0xFF)},//FrequencyItem(908250),
        .tFrequencyList.tFrequencies[12] = {.bFrequencyHigh=((908750>>16) & 0xFF), .bFrequencyMid=((908750>>8) & 0xFF), .bFrequencyLow=(908750 & 0xFF)},//FrequencyItem(908750),
        .tFrequencyList.tFrequencies[13] = {.bFrequencyHigh=((909250>>16) & 0xFF), .bFrequencyMid=((909250>>8) & 0xFF), .bFrequencyLow=(909250 & 0xFF)},//FrequencyItem(909250),
        .tFrequencyList.tFrequencies[14] = {.bFrequencyHigh=((909750>>16) & 0xFF), .bFrequencyMid=((909750>>8) & 0xFF), .bFrequencyLow=(909750 & 0xFF)},//FrequencyItem(909750),
        .tFrequencyList.tFrequencies[15] = {.bFrequencyHigh=((910250>>16) & 0xFF), .bFrequencyMid=((910250>>8) & 0xFF), .bFrequencyLow=(910250 & 0xFF)},//FrequencyItem(910250),
        .tFrequencyList.tFrequencies[16] = {.bFrequencyHigh=((910750>>16) & 0xFF), .bFrequencyMid=((910750>>8) & 0xFF), .bFrequencyLow=(910750 & 0xFF)},//FrequencyItem(910750),
        .tFrequencyList.tFrequencies[17] = {.bFrequencyHigh=((911250>>16) & 0xFF), .bFrequencyMid=((911250>>8) & 0xFF), .bFrequencyLow=(911250 & 0xFF)},//FrequencyItem(911250),
        .tFrequencyList.tFrequencies[18] = {.bFrequencyHigh=((911750>>16) & 0xFF), .bFrequencyMid=((911750>>8) & 0xFF), .bFrequencyLow=(911750 & 0xFF)},//FrequencyItem(911750),
        .tFrequencyList.tFrequencies[19] = {.bFrequencyHigh=((912250>>16) & 0xFF), .bFrequencyMid=((912250>>8) & 0xFF), .bFrequencyLow=(912250 & 0xFF)},//FrequencyItem(912250),

				.tFrequencyList.tFrequencies[20] = {.bFrequencyHigh=((912750>>16) & 0xFF), .bFrequencyMid=((912750>>8) & 0xFF), .bFrequencyLow=(912750 & 0xFF)},//FrequencyItem(912750),
        .tFrequencyList.tFrequencies[21] = {.bFrequencyHigh=((913250>>16) & 0xFF), .bFrequencyMid=((913250>>8) & 0xFF), .bFrequencyLow=(913250 & 0xFF)},//FrequencyItem(913250),
        .tFrequencyList.tFrequencies[22] = {.bFrequencyHigh=((913750>>16) & 0xFF), .bFrequencyMid=((913750>>8) & 0xFF), .bFrequencyLow=(913750 & 0xFF)},//FrequencyItem(913750),
        .tFrequencyList.tFrequencies[23] = {.bFrequencyHigh=((914250>>16) & 0xFF), .bFrequencyMid=((914250>>8) & 0xFF), .bFrequencyLow=(914250 & 0xFF)},//FrequencyItem(914250),
        .tFrequencyList.tFrequencies[24] = {.bFrequencyHigh=((914750>>16) & 0xFF), .bFrequencyMid=((914750>>8) & 0xFF), .bFrequencyLow=(914750 & 0xFF)},//FrequencyItem(914750),
        .tFrequencyList.tFrequencies[25] = {.bFrequencyHigh=((915250>>16) & 0xFF), .bFrequencyMid=((915250>>8) & 0xFF), .bFrequencyLow=(915250 & 0xFF)},//FrequencyItem(915250),
        .tFrequencyList.tFrequencies[26] = {.bFrequencyHigh=((915750>>16) & 0xFF), .bFrequencyMid=((915750>>8) & 0xFF), .bFrequencyLow=(915750 & 0xFF)},//FrequencyItem(915750),
        .tFrequencyList.tFrequencies[27] = {.bFrequencyHigh=((916250>>16) & 0xFF), .bFrequencyMid=((916250>>8) & 0xFF), .bFrequencyLow=(916250 & 0xFF)},//FrequencyItem(916250),
        .tFrequencyList.tFrequencies[28] = {.bFrequencyHigh=((916750>>16) & 0xFF), .bFrequencyMid=((916750>>8) & 0xFF), .bFrequencyLow=(916750 & 0xFF)},//FrequencyItem(916750),
        .tFrequencyList.tFrequencies[29] = {.bFrequencyHigh=((917250>>16) & 0xFF), .bFrequencyMid=((917250>>8) & 0xFF), .bFrequencyLow=(917250 & 0xFF)},//FrequencyItem(917250),

        .tFrequencyList.tFrequencies[30] = {.bFrequencyHigh=((917750>>16) & 0xFF), .bFrequencyMid=((917750>>8) & 0xFF), .bFrequencyLow=(917750 & 0xFF)},//FrequencyItem(917750),
        .tFrequencyList.tFrequencies[31] = {.bFrequencyHigh=((918250>>16) & 0xFF), .bFrequencyMid=((918250>>8) & 0xFF), .bFrequencyLow=(918250 & 0xFF)},//FrequencyItem(918250),
        .tFrequencyList.tFrequencies[32] = {.bFrequencyHigh=((918750>>16) & 0xFF), .bFrequencyMid=((918750>>8) & 0xFF), .bFrequencyLow=(918750 & 0xFF)},//FrequencyItem(918750),
        .tFrequencyList.tFrequencies[33] = {.bFrequencyHigh=((919250>>16) & 0xFF), .bFrequencyMid=((919250>>8) & 0xFF), .bFrequencyLow=(919250 & 0xFF)},//FrequencyItem(919250),
        .tFrequencyList.tFrequencies[34] = {.bFrequencyHigh=((919750>>16) & 0xFF), .bFrequencyMid=((919750>>8) & 0xFF), .bFrequencyLow=(919750 & 0xFF)},//FrequencyItem(919750),
        .tFrequencyList.tFrequencies[35] = {.bFrequencyHigh=((920250>>16) & 0xFF), .bFrequencyMid=((920250>>8) & 0xFF), .bFrequencyLow=(920250 & 0xFF)},//FrequencyItem(920250),
        .tFrequencyList.tFrequencies[36] = {.bFrequencyHigh=((920750>>16) & 0xFF), .bFrequencyMid=((920750>>8) & 0xFF), .bFrequencyLow=(920750 & 0xFF)},//FrequencyItem(920750),
        .tFrequencyList.tFrequencies[37] = {.bFrequencyHigh=((921250>>16) & 0xFF), .bFrequencyMid=((921250>>8) & 0xFF), .bFrequencyLow=(921250 & 0xFF)},//FrequencyItem(921250),
        .tFrequencyList.tFrequencies[38] = {.bFrequencyHigh=((921750>>16) & 0xFF), .bFrequencyMid=((921750>>8) & 0xFF), .bFrequencyLow=(921750 & 0xFF)},//FrequencyItem(921750),
        .tFrequencyList.tFrequencies[39] = {.bFrequencyHigh=((922250>>16) & 0xFF), .bFrequencyMid=((922250>>8) & 0xFF), .bFrequencyLow=(922250 & 0xFF)},//FrequencyItem(922250),

        .tFrequencyList.tFrequencies[40] = {.bFrequencyHigh=((922750>>16) & 0xFF), .bFrequencyMid=((922750>>8) & 0xFF), .bFrequencyLow=(922750 & 0xFF)},//FrequencyItem(922750),
        .tFrequencyList.tFrequencies[41] = {.bFrequencyHigh=((923250>>16) & 0xFF), .bFrequencyMid=((923250>>8) & 0xFF), .bFrequencyLow=(923250 & 0xFF)},//FrequencyItem(923250),
        .tFrequencyList.tFrequencies[42] = {.bFrequencyHigh=((923750>>16) & 0xFF), .bFrequencyMid=((923750>>8) & 0xFF), .bFrequencyLow=(923750 & 0xFF)},//FrequencyItem(923750),
        .tFrequencyList.tFrequencies[43] = {.bFrequencyHigh=((924250>>16) & 0xFF), .bFrequencyMid=((924250>>8) & 0xFF), .bFrequencyLow=(924250 & 0xFF)},//FrequencyItem(924250),
        .tFrequencyList.tFrequencies[44] = {.bFrequencyHigh=((924750>>16) & 0xFF), .bFrequencyMid=((924750>>8) & 0xFF), .bFrequencyLow=(924750 & 0xFF)},//FrequencyItem(924750),
        .tFrequencyList.tFrequencies[45] = {.bFrequencyHigh=((925250>>16) & 0xFF), .bFrequencyMid=((925250>>8) & 0xFF), .bFrequencyLow=(925250 & 0xFF)},//FrequencyItem(925250),
        .tFrequencyList.tFrequencies[46] = {.bFrequencyHigh=((925750>>16) & 0xFF), .bFrequencyMid=((925750>>8) & 0xFF), .bFrequencyLow=(925750 & 0xFF)},//FrequencyItem(925750),
        .tFrequencyList.tFrequencies[47] = {.bFrequencyHigh=((926250>>16) & 0xFF), .bFrequencyMid=((926250>>8) & 0xFF), .bFrequencyLow=(926250 & 0xFF)},//FrequencyItem(926250),
        .tFrequencyList.tFrequencies[48] = {.bFrequencyHigh=((926750>>16) & 0xFF), .bFrequencyMid=((926750>>8) & 0xFF), .bFrequencyLow=(926750 & 0xFF)},//FrequencyItem(926750),
        .tFrequencyList.tFrequencies[49] = {.bFrequencyHigh=((927250>>16) & 0xFF), .bFrequencyMid=((927250>>8) & 0xFF), .bFrequencyLow=(927250 & 0xFF)},//FrequencyItem(927250),

				.tFrequencyList.ucIncrementFreq_2 = {.bFrequencyHigh=((500>>16) & 0xFF), .bFrequencyMid=((500>>8) & 0xFF), .bFrequencyLow=(500 & 0xFF)},
				.tFrequencyList.bNumberOfFrequency = 50,

#endif

        .g_bRadioTestModeAction = RADIO_TEST_MODE_NONE,

				.ucDeviceID = {0x00, 0x00},
				.ucUserIdentify = USERID_BY_NONE,
        .ucCompanyID = 0,
				.ucLocationID = {0x00, 0x01},
        .ucDeviceName = AUR700_DEFAULT,

        .ucVibrEnable = VIBR_STATE_EN,
        .ucBuzzEnable = BUZZ_STATE_EN,

        .ulScanTagTimeout = 10000,
				.ulEnterIdleTime = 120000,

        .ucRFStrg = RF_STRENGTH_MED,
        .ucEraseLoggerData = ERASE_LOG_DIS,
				.ucBLEMacAddress = {0},
				.ucLockProtect = LOCK_PROT_NONE,
				.uBarcode_Selection = BARC_SEL_2D,

        .tRtc.year = 15,
        .tRtc.month = 11,
        .tRtc.date = 10,
        .tRtc.week = RTC_WEEKDAY_TUESDAY,
        .tRtc.hour = 10,
        .tRtc.minute = 10,
        .tRtc.second = 0,

				.tMenu.ucTotalNum = 6,
				.tMenu.ucItem[0] = MAIN_READ_LOGSTATUS,
				.tMenu.ucItem[1] = MAIN_START_LOGGING,
				.tMenu.ucItem[2] = MAIN_STOP_LOGGING,
				.tMenu.ucItem[3] = MAIN_WRITE_LOCATION,
				.tMenu.ucItem[4] = MAIN_READ_DEVICEINFO,
				.tMenu.ucItem[5] = MAIN_BARCODE_LOGGING,
				.tMenu.ucItem[6] = MAIN_DEBUG_TEST,				// only for debugging and testing
				.tMenu.ucItem[7] = 7,											// Maximum is 14 based on the size of bMainIdx
    }
};





