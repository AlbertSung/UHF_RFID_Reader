/**
  ******************************************************************************
  * @file    as3993_registers.h
  * @author  Albert
  * @version V1.0.0
  * @date    03-June-2017
  * @brief   Header for as3993_registers.c module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AS_3993_REGISTERS_H
#define __AS_3993_REGISTERS_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "as3993.h"
	 

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

	 

// AS3993_REG_ICD-1+1+6, -1=>No 0x0F, +1=>Len for 0x00~0x1D, +6=> 6 Other Register
#define AS3993_NUMBER_OF_CONTROL_REGISTERS      (AS3993_REG_ICD+6)  //(AS3993_REG_ICD-1+1+6)
#define AS3993_NUMBER_OF_REGISTERS              (AS3993_REG_FIFO+1)
	 

 	

#pragma anon_unions
typedef union
{
    struct
    {
        union
        {
            struct
            {
                u8 bStatus_rf_on:1; 	// 1: Tx RF field and receiver are enabled
                u8 bStatus_rec_on:1; 	// 1: The receiver is enabled
                u8 bStatus_agc_on:1; 	// 1: AGC ON
                u8 :4;								// RFU
                u8 bStatus_standby:1; // 1: Standby mode
            } s0;
            u8 bReg00_Status;
        } u0;
        union
        {
            struct
            {
                u8 bProtocol_protocol:3;    		// 000b: EPC Class1 Gen2/ISO18000-6C, 001b: ISO18000-6 Type A/B direct mode
                u8 :1;													// RFU
                u8 bProtocol_auto_ack:2; 				// 00b: No Auto ACK, 01b: AutoACK, 10b: AutoACK+ReqRN, 11b: RFU, do not set
                u8 bProtocol_direct_mode:1; 		// 0: Normal operation, 1: Disables any decoding and signal sensing automatics in the receiver
                u8 bProtocol_rx_without_crc:1; 	// 0: Rx with CRC, 1: Rx without CRC
            } s1;
            u8 bReg01_Protocol;
        } u1;
        union
        {
            struct
            {
                u8 bTxOptions_tari:3;   // 000b: Tari = 6.25 us ,001b: Tari =12.5 us ,010b: Tari =25 us
                u8 :1;									// RFU
                u8 bTxOptions_tx_one:2; // 00b: 1.50 * Tari, 01b: 1.66 * Tari, 10b: 1.83 * Tari, 11b: 2.00 * Tari
                u8 :2;									// RFU
            } s2;
            u8 bReg02_TxOptions;
        } u2;
        union
        {
            struct
            {
                u8 bRxOptions_rx_coding:3; 					// 000b: FM0, 001b: M2, 010b: M4 011b: M8
                u8 bRxOptions_TRext:1;  						// 1: Long preamble, 0: Short preamble
                u8 bRxOptions_rx_link_frequency:4; 	// 0000b: 40 kHz, 0110b: 160 kHz, 1001b: 250 kHz, 1100b: 320 kHz, 1111b: 640 kHz
            } s3;
            u8 bReg03_RxOptions;
        } u3;
        union
        {
            struct
            {
                u8 bTRcalHigh_TRcalHigh:4;	// TRcal bits with bReg05_TRcalLow
                u8 :1;											// RFU
                u8 bTRcalHigh_gain_id1x5:1; // Adapt gain 1.5x of the reflected RF power level (mixer DC level) indicator
                u8 bTRcalHigh_gain_id2x:1;  // Adapt gain 2x of the reflected RF power level (mixer DC level) indicator
                u8 bTRcalHigh_low_vsp_LO:1; // 1: Adaptation to low supply for the LO phase shifter
            } s4;
            u8 bReg04_TRcalHigh;
        } u4;
        u8 bReg05_TRcalLow; 				// Range: 0.1 us - 409 us, Steps: 4096, Step size: 0.1 us, Gen2 defines a range from 17.2 us to 225 us
        u8 bReg06_AutoACKWaitTime;  // Time used in the AutoACK procedure. Range: 0 - 816 us, Step size: 3.2 us
        u8 bReg07_RxNoResponseTime; // Step size: 25.6 us, Range: 25.6 us - 6502 us (1 - 254). 255: No response time: 26.2 ms
        u8 bReg08_RxWaitTime;       // Step size: 6.4 us, Range: 6.4 us - 1632 us (1 - 255), 00h: The receiver is enabled immediately after Tx
        union
        {
            struct
            {
                u8 bRxFilter_high_pass:3;
                u8 bRxFilter_low_pass:3;
                u8 bRxFilter_bypass1:1;
                u8 bRxFilter_bypass2:1;
            } s9;
            u8 bReg09_RxFilterSettings;	// Set to FFh: 40kHz link frequency
        } u9;
        union
        {
            struct
            {
                u8 bRxMixerGain_mixer_input_range:2;
                    // Differential Rx mixer:
                    // 00b: Nominal gain
                    // 01b: 8 dB attenuation
                    // 10b: 10 dB gain increase
                    // Single ended Rx mixer:
                    // 00b: 6 dB mixer gain decrease
                    // 01b: Nominal gain
                    // 11b: 6 dB mixer gain increase
								u8 bRxMixerGain_hysteresis_increase:3;	// Steps: 5, Step Size: 3 dB, 000b: 0dB, 100b: 12dB
                u8 bRxMixerGain_baseband_sign:1;    		// 0: Decrease baseband gain, 1: Increase baseband gain
								u8 bRxMixerGain_baseband_gain:2;    		// Steps: 4, Step Size: 3 dB, 00b: 0 dB, 11b: 9dB
            } s10;
            u8 bReg0A_RxMixerGain;
        } u10;
        union
        {
            struct
            {
                u8 bVoltageRegulator_rvs:3;
                u8 bVoltageRegulator_rvs_rf:3;
										// Manual setting:
										// For correct operation the regulator voltage drop should be 300 mv or more
										// Min: 000b: 2.7 V
										// Max: 111b: 3.4 V
										// Steps: 8
										// Step size: 0.1 V
										// Automatic setting:
										// 001b: Target voltage drop > 250 mV
										// 011b: Target voltage drop > 300 mV
										// 111b: Target voltage drop > 350 mV
								u8 bVoltageRegulator_pa_bias_2_times:1;	// 1: Increase bias two times
                u8 bVoltageRegulator_pa_bias_4_times:1;	// 1: Increase bias four times
            } s11;
            u8 bReg0B_VoltageRegulator;
        } u11;
        union
        {
            struct
            {
								u8 bRFOutputAndLO_rf_low_power:2;								// 00b: Disable, 01b: 7 mA, 10b: 14 mA, 11b: 22 mA
                u8 bRFOutputAndLO_main_pa:2;										// 00b: Disable, 01b: 7 mA, 10b: 14 mA, 11b: 22 mA
                u8 :1;																					// RFU
                u8 bRFOutputAndLO_internal_voltage_regulator:1;	// VDD_PA regulator is automatically enabled via bRFOutputAndLO_main_pa
								u8 bRFOutputAndLO_LO_source:1;									// 0: LO source is RFOPX/RFONX, 1: LO source is pre-driver
								u8 bRFOutputAndLO_LO_gain:1;										// 0: Nominal, 1: 6 dB gain in LO path
            } s12;
            u8 bReg0C_RFOutputAndLO;
        } u12;
        union
        {
            struct
            {
                u8 :2;																	// RFU
								u8 bMiscellaneous1_single_ended:1;			// 0: Differential input, 1: Single ended input
                u8 bMiscellaneous1_open_drain:1;				// Valid for MISO, IQR, CLSYS
								u8 bMiscellaneous1_miso_pull_down_1:1;	// 1: Enable a pull down resistor on MISO when NCS is high
                u8 bMiscellaneous1_miso_pull_down_2:1;	// 1: Enable a pull down resistor on MISO when NCS is low and MISO is not driven by 3993
                u8 bMiscellaneous1_hs_oad:1;						// Valid for OAD, OAD2, ADC
                u8 bMiscellaneous1_hs_output:1;					// Valid for MISO, IQR, CLSYS
            } s13;
            u8 bReg0D_Miscellaneous1;
        } u13;
        union
        {
            struct
            {
								u8 bMiscellaneous2_clock_output:3;			// 000b: Off, 100b: 4 MHz, 001b: 5 MHz, 010b: 10 MHz, 011b: 20 MHz
                u8 bMiscellaneous2_rx_filter_select:1;	// 1: Enables changing the hp calibration, 0: Enables changing the lp calibration
                u8 :2;																	// RFU
								u8 bMiscellaneous2_oscillator_mode:2;		// 00b: Normal operation, 01b: External sinus TCXO AC, 10b: Disable auto power saving, 11b: RFU
            } s14;
            u8 bReg0E_Miscellaneous2;
        } u14;
        union
        {
            struct
            {
                u8 bMeasurement_adc_select:4;
                u8 bMeasurement_analog_test_output:2;
                u8 bMeasurement_digital_test_output:2;
            } s15;
            u8 bReg10_Measurement;
        } u15;
        union
        {
            struct
            {
                u8 bVCOControl_manual_vco_range:4;
                u8 bVCOControl_internal_osc_bias:3;
                u8 bVCOControl_vco_measurement:1;
            } ss16;
            u8 bReg11_VCOControl;
        } uu16;
        union
        {
            struct
            {
                u8 bChargePump_current:3;
                u8 bChargePump_loop_filter_c3:3;
                u8 bChargePump_loop_filter_r3:2;
            } s17;
            u8 bReg12_ChargePump;
        } u17;
        union
        {
            struct
            {
                u8 bModulator1_ask_rate:2;
                u8 bModulator1_low_pass_filter:1;
                u8 :2;																// RFU
                u8 bModulator1_aux_to_low_power:1;
                u8 bModulator1_main_to_high_power:1;
                u8 :1;																// RFU
            } s18;
            u8 bReg13_Modulator1;
        } u18;
        union
        {
            struct
            {
                u8 bModulator2_delimiter_length:6;
                u8 bModulator2_pr_ask_enable:1;
                u8 bModulator2_ook_ask_enable:1;
            } s19;
            u8 bReg14_Modulator2;
        } u19;
        union
        {
            struct
            {
								u8 bModulator3_tx_power_small_adjust:3;	// 000b: Nominal, 001b: -1 dB, 111b: -7 dB, Step size: -1 dB
                u8 bModulator3_tx_power_big_adjust:2;  	// 00b: 0 dB nominal, 01b: -8 dB, 10b: -12 dB
                u8 bModulator3_linear_select:1;
                u8 bModulator3_rf_on_off_time:2;
            } s20;
            u8 bReg15_Modulator3;
        } u20;
        u8 bReg16_Modulator4_Tari;
        union
        {
            struct
            {
                u8 bPLLMain1_divider_b_msb:4;
                u8 bPLLMain1_PLL_divider:3;  	// 100b: 125 kHz, 101b: 100 kHz, 110b: 50 kHz, 111b: 25 kHz, Others: RFU, do not set
                u8 :1;
            } s22;
            u8 bReg17_PLLMain1;
        } u22;
        union
        {
            struct
            {
                u8 bPLLMain2_divider_a_msb:2;
                u8 bPLLMain2_divider_b_lsb:6;
            } s23;
            u8 bReg18_PLLMain2;
        } u23;
        u8 bReg19_PLLMain3_divider_a_lsb;
        union
        {
            struct
            {
                u8 bPLLAuxiliary1_divider_b_msb:4;
                u8 :4;															// RFU
            } s25;
            u8 bReg1A_PLLAuxiliary1;
        } u25;
        union
        {
            struct
            {
                u8 bPLLAuxiliary2_divider_a_msb:2;
                u8 bPLLAuxiliary2_divider_b_lsb:6;
            } s26;
            u8 bReg1B_PLLAuxiliary2;
        } u26;
        u8 bReg1C_PLLAuxiliary3_divider_a_lsb;
        union
        {
            struct
            {
                u8 bICD_IQ_Select_ICD_threshold:4;
                u8 bICD_IQ_Select_IQ_threshold:4;
            } s28;
            u8 bReg1D_ICD_IQ_Select;
        } u28;
        union
        {
            struct
            {
                u8 bEmitterCoupledMixer_voltage_range:3;
                u8 bEmitterCoupledMixer_sink_current_adjust:3;
                u8 bEmitterCoupledMixer_dec_device_bias:2;
            } s29;
            u8 bReg22_EmitterCoupledMixer;
        } u29;
        union
        {
            struct
            {
                u8 bStatusPage_reg2Bpage:4;
                u8 bStatusPage_reg2Cpage:2;
                u8 bStatusPage_reg2Dpage:2;
            } s30;
            u8 bReg29_StatusPage;
        } u30;
        union
        {
            struct
            {
                u8 bIRQMask1_no_response:1;
                u8 bIRQMask1_auto_ack:1;
                u8 :1;											// RFU
                u8 bIRQMask1_header:1;
                u8 bIRQMask1_err:1;
                u8 bIRQMask1_fifo:1;
                u8 bIRQMask1_rx:1;
                u8 bIRQMask1_tx:1;
            } s31;
            u8 bReg35_IRQMask1;
        } u31;
        union
        {
            struct
            {
                u8 bIRQMask2_err3_preamble_fifo:1;
                u8 bIRQMask2_err2_rx_length:1;
                u8 bIRQMask2_err1_crc:1;
                u8 :3;															// RFU
                u8 bIRQMask2_cmd:1;
                u8 bIRQMask2_ana_rf:1;
            } ss32;
            u8 bReg36_IRQMask2;
        } uu32;
        union
        {
            struct
            {
                u8 bRxLength_rx_length_msb:4;
                u8 bRxLength_auto_error_code:1;
                u8 bRxLength_repeat_2nd:1;
                u8 bRxLength_direct_2nd:1;
                u8 bRxLength_rx_without_crc:1;
            } s33;
            u8 bReg3A_RxLength;
        } u33;
        union
        {
            struct
            {
                u8 bTxSettings_gen2_session:2;
                u8 bTxSettings_force_TRcal:1;
                u8 bTxSettings_tx_crc_type:1;	// 0: CRC-16, 1: CRC-5
                u8 :4;												// RFU
            } s34;
            u8 bReg3C_TxSettings;
        } u34;
    } s;
    u8 bRegisters[AS3993_NUMBER_OF_CONTROL_REGISTERS];
} __attribute__((transparent_union)) __attribute__ ((aligned(1))) AS3993ControlRegisters_t;



/* Exported constants --------------------------------------------------------*/
extern const AS3993ControlRegisters_t AS3993_CONTROL_REGISTER_DEFAULT;
extern const u8 AS3993_CONTROL_REGISTERS[AS3993_NUMBER_OF_CONTROL_REGISTERS];
extern const u8 AS3993_CONTROL_REGISTER_INDEXS[AS3993_NUMBER_OF_REGISTERS];



#ifdef __cplusplus
}
#endif

#endif /* __AS_3993_REGISTERS_H */



