/**
  ******************************************************************************
  * @file    as3993.h
  * @author  Albert
  * @version V1.0.0
  * @date    03-June-2017
  * @brief   Header for as3993.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AS_3993_H
#define __AS_3993_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"
#include "platform.h"
#include "b2e_errno.h"

/* Exported types ------------------------------------------------------------*/
#pragma anon_unions
typedef union
{
    struct
    {
        s8 cIValue;
        s8 cQValue;
    } st1;
    s8 cValues[2];
    u8 bValues[2];
    u16 uIQValue;
} __attribute__ ((aligned(2))) IQValue_t;

typedef struct
{
    u8 bRegValue;
    u8 bDividerKHz;
} PLL_DIVIDER_T;

	 
/* Exported constants --------------------------------------------------------*/
extern volatile u16 g_uAS3993IRQStatus;
extern u8 g_bIsRFPowerOff, m_bReaderPowerMode;

/* Exported functions --------------------------------------------------------*/
s8 as3993Initialize(void);
void AS3993_IRQ_Callback(void);

void as3993WriteByte(u8 bAddress, u8 bValue);
void as3993WriteBytes(u8 bAddress, const u8* pbBuffer, u8 bLength);
void as3993WriteCmd(u8 bCommand);
void as3993WriteCmdWithBytes(u8 bCommand, u8 bAddress, const u8* pbBuffer, u8 bLength);
u8 as3993ReadByte(u8 bAddress);
void as3993ReadBytes(u8 bAddress, u8* pbBuffer, u8 bLength);

static void as3993RestoreRegisters(void);
static s8 as3993WaitingLockPLL(void);
void as3993WaitForStartup(void);

void as3993WaitForResponse(u16 uWaitMask);
void as3993WaitForResponseTimed(u16 uWaitMask, u32 ulCounter);

s8 as3993GetGainEffectRSSIValue(void);
s8 as39932GetRssi(u8 bRSSISelect, u8* pbIQValue);
s8 as3993MeasureMaxRSSI(u16 uMeasureTimeMilliseconds, s8 cCenterRSSI);
s8 as3993GetRxMixerGainValue(void);
IQValue_t as3993GetMixerIQLevel(void);

u8 as3993GetReaderPowerMode(void);
s8 as3993SetReaderPowerMode(u8 bPowerMode);
void as3993SetAntennaPower(u8 bIsPowerOn);

void as3993SetBaseFrequency(u8 bPLLRegister, u32 ulBaseFrequency, u8 bPLLDivUser);

s8 as3993SetSensitivity(s8 cSensitivity);
s8 as3993SetRFTxPower(s8 cPower);

void as3993SetAutoAckMode(u8 uAutoAckMode);
void as3993SetGen2Session(u8 bSession);
void as3993SetGen2Protocol(void);

s8 as3993IsValidFrequency(u32 ulFrequency);



/* Defined functions ---------------------------------------------------------*/
// Definition for marco
#define uhfBeginWriteAndRead()              {UHF_DISEXTIRQ(); UHF_NCS_SELECT();}
#define uhfEndWriteAndRead()                {UHF_NCS_DESELECT(); UHF_ENEXTIRQ();}

#define spiWrite(Buffer, BufferLength)			SPITxRx(Buffer, NULL, BufferLength)
#define spiRead(Buffer, BufferLength)       SPITxRx(NULL, Buffer, BufferLength)

#define as3993Response                      g_uAS3993IRQStatus
#define as3993GetResponse()                 g_uAS3993IRQStatus
#define as3993ClrResponse()                 {g_uAS3993IRQStatus = 0;}
#define as3993ClrResponseMask(uMask)        {g_uAS3993IRQStatus &= (~(uMask));}

#define as3993GetSensitivity()              as3993GetGainEffectRSSIValue()

#define as3993IsHaveRFPowerOff()            (g_bIsRFPowerOff)
#define as3993ClearHaveRFPowerOff()         {g_bIsRFPowerOff = FALSE;}


// AS3993_SPI_MODE, Bit 6~7 -------------------- //
#define AS3993_SPI_MODE_WRITE               0x00
#define AS3993_SPI_MODE_READ                0x40
#define AS3993_SPI_MODE_COMMAND             0x80


// TrExt definitions
#define TREXT_OFF                           0       // use short preamble
#define TREXT_ON                            1       // use long preamble

// Tari definitions
#define TARI_6_25                           0       // set tari to 6.25us
#define TARI_12_5                           1       // set tari to 12.5us
#define TARI_25                             2       // set tari to 25us

// RSSI definitions
#define RSSI_MODE_REALTIME                  0x00
#define RSSI_MODE_PILOT                     0x04
#define RSSI_MODE_2NDBYTE                   0x06
#define RSSI_MODE_PEAK                      0x08

//  * at 40kHz BLF one gen2 slot takes ~40ms, we are going to wait
//  * 50ms (in as3993WaitForResponse()) to be on the safe side.
//  * delay in us which will be used as3993WaitForResponse() loop
#define WAIT_FOR_RESPONSE_DELAY             10
// ** max delay in us which we will wait for an response from AS3993
#define WAIT_FOR_RESPONSE_TIMEOUT           5000000ULL   	// SL900A Timeout up to 3 ms
#define WAIT_FOR_LONG_RESPONSE_TIMEOUT      30000000ULL		// 30 ms
// ** number of loop iterations in as3993WaitForResponse()
#define WAIT_FOR_RESPONSE_COUNT             (30000/10)//(WAIT_FOR_RESPONSE_TIMEOUT/WAIT_FOR_RESPONSE_DELAY)
#define WAIT_FOR_LONG_RESPONSE_COUNT        (50000/10)//(WAIT_FOR_LONG_RESPONSE_TIMEOUT/WAIT_FOR_RESPONSE_DELAY)

#define AS3993_INVALID_G_VALUE              0

#define AS3993_INVALID_RSSI                 (-128)

#define AS3993_NOMINAL_SENSITIVITY          (-68)

#define AS3993_SINGLE_ENDED_G_VALUE         (-77)       // About SENSITIVITY
#define AS3993_DIFFERENTIAL_G_VALUE         (-71)       // About SENSITIVITY

#define DEVIATION_FREQUENCY  								0

#define AS3993_MIN_TARGET_FREQUENCY         840000ul
#define AS3993_MAX_TARGET_FREQUENCY         960000ul

// time in ms to wait for AS3980 to be ready
#define AS3980WAITTIME          						510

// Wait for AS3980 to be ready.
#if RUN_ON_AS3980
#define WaitForAS3980()     								{delay_ms(AS3980WAITTIME);}
#else
#define WaitForAS3980()     								{}
#endif

#define CHIP_FIFO_BUFFER_SIZE               24
#define IRQ_FIFO_LESS_SIZE                  6
#define IRQ_FIFO_BUFFER_SIZE                (CHIP_FIFO_BUFFER_SIZE - IRQ_FIFO_LESS_SIZE)

	
	 
// Registers map ------------------------------- //
#define AS3993_REG_STATUSCTRL               0x00
#define AS3993_REG_PROTOCOLCTRL             0x01
#define AS3993_REG_TXOPTIONS                0x02
#define AS3993_REG_RXOPTIONS                0x03
#define AS3993_REG_TRCALHIGH                0x04
#define AS3993_REG_TRCALLOW                 0x05
#define AS3993_REG_AUTOACKTIMER             0x06
#define AS3993_REG_RXNORESPONSEWAITTIME     0x07
#define AS3993_REG_RXWAITTIME               0x08
#define AS3993_REG_RXFILTER                 0x09
#define AS3993_REG_RXMIXERGAIN              0x0A
#define AS3993_REG_REGULATORCONTROL         0x0B
#define AS3993_REG_RFOUTPUTCONTROL          0x0C
#define AS3993_REG_MISC1                    0x0D
#define AS3993_REG_MISC2                    0x0E
#define AS3993_REG_MEASUREMENTCONTROL       0x10
#define AS3993_REG_VCOCONTROL               0x11
#define AS3993_REG_CPCONTROL                0x12
#define AS3993_REG_MODULATORCONTROL1        0x13
#define AS3993_REG_MODULATORCONTROL2        0x14
#define AS3993_REG_MODULATORCONTROL3        0x15
#define AS3993_REG_MODULATORCONTROL4        0x16
#define AS3993_REG_PLLMAIN1                 0x17
#define AS3993_REG_PLLMAIN2                 0x18
#define AS3993_REG_PLLMAIN3                 0x19
#define AS3993_REG_PLLAUX1                  0x1A
#define AS3993_REG_PLLAUX2                  0x1B
#define AS3993_REG_PLLAUX3                  0x1C
#define AS3993_REG_ICD                      0x1D
#define AS3993_REG_MIXOPTS                  0x22
#define AS3993_REG_TEST1                    0x23
#define AS3993_REG_TEST2                    0x24
#define AS3993_REG_TEST3                    0x25
#define AS3993_REG_TEST4                    0x26
#define AS3993_REG_TXRESHAPE                0x27
#define AS3993_REG_STATUSPAGE               0x29
#define AS3993_REG_AGCANDSTATUS             0x2A
#define AS3993_REG_RSSILEVELS               0x2B
#define AS3993_REG_AGL                      0x2C
#define AS3993_REG_ADC                      0x2D
#define AS3993_REG_COMMANDSTATUS            0x2E
#define AS3993_REG_DEVICEVERSION            0x33
#define AS3993_REG_IRQMASK1                 0x35
#define AS3993_REG_IRQMASK2                 0x36
#define AS3993_REG_IRQSTATUS1               0x37
#define AS3993_REG_IRQSTATUS2               0x38
#define AS3993_REG_FIFOSTATUS               0x39
#define AS3993_REG_RXLENGTHUP               0x3A
#define AS3993_REG_RXLENGTHLOW              0x3B
#define AS3993_REG_TXSETTING                0x3C
#define AS3993_REG_TXLENGTHUP               0x3D
#define AS3993_REG_TXLENGTHLOW              0x3E
#define AS3993_REG_FIFO                     0x3F

// Reader commands ----------------------------- //
#define AS3993_CMD_IDLE                     0x80
#define AS3993_CMD_DIRECT_MODE              0x81
#define AS3993_CMD_SOFT_INIT                0x83
#define AS3993_CMD_HOP_TO_MAIN_FREQUENCY    0x84
#define AS3993_CMD_HOP_TO_AUX_FREQUENCY     0x85
#define AS3993_CMD_TRIGGERADCCON            0x87
#define AS3993_CMD_TRIG_RX_FILTER_CAL       0x88
#define AS3993_CMD_DEC_RX_FILTER_CAL        0x89
#define AS3993_CMD_INC_RX_FILTER_CAL        0x8A
#define AS3993_CMD_TRANSMCRC                0x90
#define AS3993_CMD_TRANSMCRCEHEAD           0x91
#define AS3993_CMD_TRANSMNOCRC              0x92
#define AS3993_CMD_DELAY_TRANSMIT_CRC       0x93
#define AS3993_CMD_DELAY_TRANSMIT_NO_CRC    0x94
#define AS3993_CMD_CLOSE_SLOT_SEQUENCE      0x95
#define AS3993_CMD_BLOCKRX                  0x96
#define AS3993_CMD_ENABLERX                 0x97
#define AS3993_CMD_QUERY                    0x98
#define AS3993_CMD_QUERYREP                 0x99
#define AS3993_CMD_QUERYADJUSTUP            0x9A
#define AS3993_CMD_QUERYADJUSTNIC           0x9B
#define AS3993_CMD_QUERYADJUSTDOWN          0x9C
#define AS3993_CMD_ACK                      0x9D
#define AS3993_CMD_NAK                      0x9E
#define AS3993_CMD_REQRN                    0x9F
#define AS3993_CMD_SUPPLY_AUTO_LEVEL        0xA2
#define AS3993_CMD_SUPPLY_MANUAL_LEVEL      0xA3
#define AS3993_CMD_VCO_AUTO_RANGE           0xA4
#define AS3993_CMD_VCO_MANUAL_RANGE         0xA5
#define AS3993_CMD_AGL_ON                   0xA6
#define AS3993_CMD_AGL_OFF                  0xA7
#define AS3993_CMD_STORE_RSSI               0xA8
#define AS3993_CMD_CLEAR_RSSI               0xA9
#define AS3993_CMD_ANTI_COLL_ON             0xAA
#define AS3993_CMD_ANTI_COLL_OFF            0xAB

// IRQ Mask register --------------------------- //
#define AS3993_IRQ1_NORESP                  0x01        // ** AS3993 interrupt: no response bit
#define AS3993_IRQ1_AUTOACK                 0x02        // ** AS3993 interrupt: Auto ACK finished
#define AS3993_IRQ1_HEADER                  0x08        // ** AS3993 interrupt: header bit / 2bytes
#define AS3993_IRQ1_RXERR                   0x10        // ** AS3993 IRQ status register: receive error
#define AS3993_IRQ1_FIFO                    0x20        // ** AS3993 interrupt: fifo bit
#define AS3993_IRQ1_RX                      0x40        // ** AS3993 IRQ status register: receive complete
#define AS3993_IRQ1_TX                      0x80        // ** AS3993 IRQ status register: transmit complete

#define AS3993_IRQ2_PREAMBLE                0x01        // ** AS3993 interrupt: error 3 bit: preamble/fifo overflow
#define AS3993_IRQ2_RXCOUNT                 0x02        // ** AS3993 interrupt: error 2 bit: rxcount
#define AS3993_IRQ2_CRCERROR                0x04        // ** AS3993 interrupt: error1 bit: CRC/autohop
#define AS3993_IRQ2_END_CMD                 0x40        // ** AS3993 interrupt: cmd bit: end of command
#define AS3993_IRQ2_END_ANA                 0x80

#define AS3993_IRQ1_MASK_ALL                0xFB
#define AS3993_IRQ2_MASK_ALL                0xC7

// *AS3993_REG_FIFO STATUS register
// ** AS3993 AS3993_REG_FIFO status register: FIFO overflow
#define AS3993_FIFOSTAT_OVERFLOW            0x20

#define RESP_NORESINTERRUPT                 AS3993_IRQ1_NORESP
#define RESP_AUTOACK                        AS3993_IRQ1_AUTOACK
#define RESP_HEADERBIT                      AS3993_IRQ1_HEADER
#define RESP_RXERR                          AS3993_IRQ1_RXERR
#define RESP_FIFO                           AS3993_IRQ1_FIFO
#define RESP_RXIRQ                          AS3993_IRQ1_RX
#define RESP_TXIRQ                          AS3993_IRQ1_TX
#define RESP_PREAMBLEERROR                 (AS3993_IRQ2_PREAMBLE << 8)
#define RESP_RXCOUNTERROR                  (AS3993_IRQ2_RXCOUNT  << 8)
#define RESP_CRCERROR                      (AS3993_IRQ2_CRCERROR << 8)
#define RESP_END_CMD                       (AS3993_IRQ2_END_CMD  << 8)
#define RESP_END_ANA                       (AS3993_IRQ2_END_ANA  << 8)

#if RUN_ON_AS3980   // as3980 produces irq_err2 (without irq_err) if new epc is read 500ms after last one.
#define RESP_RXDONE_OR_ERROR  (RESP_RXIRQ | RESP_AUTOACK | RESP_RXERR | RESP_NORESINTERRUPT | RESP_RXCOUNTERROR)
#else
#define RESP_RXDONE_OR_ERROR  (RESP_RXIRQ | RESP_AUTOACK | RESP_RXERR | RESP_NORESINTERRUPT)
#endif
// RESP_FIFOOVERFLOW does not work reliably


// Registers setting --------------------------- //
// Register 0x00
#define AS3993_REG00_STANDBY                0x80    //  1 = Standby Mode
#define AS3993_REG00_AGC_ENABLE             0x04    //  1 = AGC Enable On
#define AS3993_REG00_RX_ENABLE              0x02    //  1 = Receiver Enable
#define AS3993_REG00_TX_ENABLE              0x01    //  1 = Transmitter (Tx RF field) and Receiver Enable
#define AS3993_REG00_TX_RX_ENABLE           (AS3993_REG00_RX_ENABLE | AS3993_REG00_TX_ENABLE)
#define AS3993_REG00_TX_RX_DISABLE          0x00
#define AS3993_REG00_TX_RX_MASK             (AS3993_REG00_TX_RX_ENABLE)

// Register 0x01
#define AS3993_REG01_RX_WITHOUT_CRC					0x80
#define AS3993_REG01_DECODER_DISABLE        0x40    // 1: Disables any decoding and signal sensing automatics in the receiver. It is advised to set this bit high when continuous analogue measurements are performed.
#define AS3993_REG01_AUTO_ACK_NONE          0x00
#define AS3993_REG01_AUTO_ACK_ENABLE        0x10
#define AS3993_REG01_AUTO_ACK_WITH_REQRN    0x20
#define AS3993_REG01_AUTO_ACK_MASK          0x30
#define AS3993_REG01_PROTOCOL_GEN2          0x00
#define AS3993_REG01_PROTOCOL_ISO18000_6    0x01
#define AS3993_REG01_PROTOCOL_MASK          0x07

// Register 0x0A
#define AS3993_REG0A_BASEGAIN_MASK          0xC0
#define AS3993_REG0A_BASEGAIN_POS			      0x06
#define AS3993_REG0A_INC_BASEGAIN           0x20    //  1: Increase baseband gain, 0: Decrease baseband gain
#define AS3993_REG0A_HYSTERESIS_INC_MASK    0x1C    // Steps 5 (0~4), Step 3 dB, 0~12 dB
#define AS3993_REG0A_HYSTERESIS_INC_POS		  0x02
#define AS3993_REG0A_MIX_DIFF_NOMINAL       0x00
#define AS3993_REG0A_MIX_DIFF_DEC_8DBM      0x01
#define AS3993_REG0A_MIX_DIFF_INC_10DBM     0x02
#define AS3993_REG0A_MIX_SE_DEC_6DBM        0x00
#define AS3993_REG0A_MIX_SE_NOMINAL         0x01
#define AS3993_REG0A_MIX_SE_INC_6DBM        0x03
#define AS3993_REG0A_MIX_RX_MIXER_MASK      0x03

// Register 0x0C
#define AS3993_REG0C_LO_GAIN_6DB	              0x80
#define AS3993_REG0C_LO_GAIN_POS                0x07
#define AS3993_REG0C_LO_SOURCE_INTERNAL_PA      0x40
#define AS3993_REG0C_LO_SOURCE_RFOPX_RFONX      0x00
#define AS3993_REG0C_LO_SOURCE_SELECT_POS       0x06
#define AS3993_REG0C_ENABLE_INTERNAL_PA_AUTO    0x20
#define AS3993_REG0C_ENABLE_INTERNAL_PA_MANUAL  0x00
#define AS3993_REG0C_ENABLE_INTERNAL_PA_POS     0x05
#define AS3993_REG0C_MAIN_PA_MASK               0x0C
#define AS3993_REG0C_MAIN_PA_DISABLE            0x00
#define AS3993_REG0C_MAIN_PA_7MA 						    0x04
#define AS3993_REG0C_MAIN_PA_14MA    						0x08
#define AS3993_REG0C_MAIN_PA_22MA    						0x0C
#define AS3993_REG0C_MAIN_PA_POS                0x02
#define AS3993_REG0C_RF_LOW_POWER_MASK          0x03
#define AS3993_REG0C_RF_LOW_POWER_DISABLE       0x00
#define AS3993_REG0C_RF_LOW_POWER_7MA 			    0x01
#define AS3993_REG0C_RF_LOW_POWER_14MA   				0x02
#define AS3993_REG0C_RF_LOW_POWER_22MA   				0x03
#define AS3993_REG0C_RF_LOW_POWER_POS           0x00

// Register 0x0D
#define AS3993_REG0D_SINGLE_ENDED           0x04    //  1 = Single ended input, 0 = Differential input

// Register 0x10
#define AS3993_REG10_DTO_OAD_MASK           0xC0    // Digital test output modes
#define AS3993_REG10_ATO_OAD_MASK           0x30    // Analog test output modes
#define AS3993_REG10_ADC_MEAS_MASK          0x0F
#define AS3993_REG10_ADC_MEAS_NC            0x00    // 0000b: NC
#define AS3993_REG10_ADC_MEAS_MIXER_DC_I    0x01    // 0001b: Mixer DC level I-channel
#define AS3993_REG10_ADC_MEAS_MIXER_DC_Q    0x02    // 0010b: Mixer DC level Q-channel
#define AS3993_REG10_ADC_MEAS_ADC_PIN       0x03    // 0011b: ADC pin
#define AS3993_REG10_ADC_MEAS_INTERNAL_RF   0x04    // 0100b: Internal RF level
#define AS3993_REG10_ADC_MEAS_VEXT          0x07    // 0111b: VEXT level
#define AS3993_REG10_ADC_MEAS_VDD_B         0x08    // 1000b: VDD_B level
#define AS3993_REG10_ADC_MEAS_VEXT_PA       0x09    // 1001b: VEXT_PA level
#define AS3993_REG10_ADC_MEAS_VDD_PA        0x0A    // 1010b: VDD_PA level
#define AS3993_REG10_ADC_MEAS_RSSI_I        0x0B    // 1011b: RSSI I level
#define AS3993_REG10_ADC_MEAS_RSSI_Q        0x0C    // 1100b: RSSI Q level
#define AS3993_REG10_ADC_MEAS_RFOPX_POWER   0x0F    // 1111b: RFOPX, RFONX power level

// Register 0x11
#define AS3993_REG11_VCO_MEAS_ENABLE        0x80

// Register 0x15
#define AS3993_REG15_RF_TX_POWER_MASK       0x1F
#define AS3993_REG15_RF_TX_POWER_DEC_8DB    0x08
#define AS3993_REG15_RF_TX_POWER_DEC_12DB   0x10
#define AS3993_REG15_RF_TX_POWER_1DB_MASK   0x07
#define AS3993_REG15_RF_TX_POWER_DEC_7DB    0x07

// Register 0x17
#define AS3993_REG17_PLL_DIVIDER_MASK       0x70
#define AS3993_REG17_PLL_DIVIDER_125K       0x40
#define AS3993_REG17_PLL_DIVIDER_100K       0x50
#define AS3993_REG17_PLL_DIVIDER_50K        0x60
#define AS3993_REG17_PLL_DIVIDER_25K        0x70
#define AS3993_REG17_PLL_DIVIDER_POS  			0x04

// Register 0x29
#define AS3993_REG29_SEL_REG2D_MASK         0xC0
#define AS3993_REG29_SEL_REG2D_ADC          0x00
#define AS3993_REG29_SEL_REG2C_MASK         0x30
#define AS3993_REG29_SEL_REG2C_VCO          0x10
#define AS3993_REG29_SEL_REG2B_RSSI_MASK    0x0F
#define AS3993_REG29_SEL_REG2B_REAL_TIME    RSSI_MODE_REALTIME    // 0000b: Real time RSSI I,Q

// Register 0x2A
#define AS3993_REG2A_AGC_MASK               0x70    // bit4~6, Steps 7, Step size 3 db
#define AS3993_REG2A_RF_OK                  0x04    // 1 = Indicates that the RF carrier is stable.
#define AS3993_REG2A_PLL_OK                 0x02    // 1 = Indicates that the PLL is locked to the RF carrier frequency.
#define AS3993_REG2A_OSC_OK                 0x01    // 1 = Indicates that the reference oscillator frequency is stable.

// Register 0x2C
#define AS3993_REG2C_VCO_VOL_RESULT_MASK    0x07

// Register 0x2E
#define AS3993_REG2E_ADC_CONV_DONE          0x08    // Signals the completion of the direct command Trigger AD conversion (87h). Triggers IRQ.

// Register 0x35
#define AS3993_REG35_IRQ_NORESP_ENABLE      0x01    // In case irq_noresp interrupt is disabled, the receive operation is never interrupted by the No Response Timer.
#define AS3993_REG35_IRQ_AUTOACK_ENABLE     0x02
#define AS3993_REG35_IRQ_HEADER_ENABLE      0x08
#define AS3993_REG35_IRQ_RXERR_ENABLE       0x10
#define AS3993_REG35_IRQ_FIFO_ENABLE        0x20
#define AS3993_REG35_IRQ_RX_ENABLE          0x40
#define AS3993_REG35_IRQ_TX_ENABLE          0x80

// Register 0x3A
#define AS3993_REG3A_RX_N2_NOCRC            0x80
#define AS3993_REG3A_FIFO_DIR_IRQ2          0x40
#define AS3993_REG3A_REP_IRQ2               0x20
#define AS3993_REG3A_AUTO_ERR_LEN           0x10
#define AS3993_REG3A_RX_INT_SETTINGS_MASK   (AS3993_REG3A_RX_N2_NOCRC | AS3993_REG3A_FIFO_DIR_IRQ2 | AS3993_REG3A_REP_IRQ2 | AS3993_REG3A_AUTO_ERR_LEN)
#define AS3993_REG3A_RX_LENGTH_MSB_MASK     0x0F

// Register 0x3C
#define AS3993_REG3C_GEN2_SESSION_MASK      0x03
#define AS3993_REG3C_GEN2_SESSION_S1S0_0    0x00
#define AS3993_REG3C_GEN2_SESSION_S1S0_1    0x01
#define AS3993_REG3C_GEN2_SESSION_S1S0_2    0x02
#define AS3993_REG3C_GEN2_SESSION_S1S0_3    0x03


// Query AutoAck Mode
#define UHF_AUTO_MODE_NONE                  0
#define UHF_AUTO_MODE_ACK                   1
#define UHF_AUTO_MODE_ACK_REQ_RN            2

// Power Mode
#define UHF_POWER_MODE_UNKNOWN              0       // Never Set Power mode
#define UHF_POWER_MODE_ERROR                1       // Chip Initialize Error
#define UHF_POWER_MODE_INITIALIZE           2       // Chip Initialize
#define UHF_POWER_MODE_POWER_DOWN           3
#define UHF_POWER_MODE_STANDBY              4
#define UHF_POWER_MODE_NORMAL               5
#define UHF_POWER_MODE_NORMAL_REC_ON        6
#define UHF_POWER_MODE_NORMAL_RF_ON         7

#define UHF_READER_IDLE_POWER_MODE          UHF_POWER_MODE_POWER_DOWN

// Power Setting
#define UHF_OFF                             0
#define UHF_ON                              1

#define UHF_NEVER_HOP                       (-1)

// 
#define UHF_INPUT_MIXER_DIFFERENTIAL        0
#define UHF_INPUT_MIXER_SINGLE_ENDED        1

#if RUN_ON_AS3980
#define UHF_DEF_INPUT_MIXERS                (UHF_INPUT_MIXER_SINGLE_ENDED)
#else
#define UHF_DEF_INPUT_MIXERS                (UHF_INPUT_MIXER_DIFFERENTIAL)
#endif



#ifdef __cplusplus
}
#endif

#endif /* __AS_3993_H */



