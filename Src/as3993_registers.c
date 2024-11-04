/**
  ******************************************************************************
  * @file    as3993_registers.c
  * @author  Albert
  * @version V1.0.0
  * @date    03-June-2017
  * @brief   as3993 registers definition
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "as3993_registers.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/******************************************************************************/


/**
  * @brief  This union/structure is for registers' default settings
  * @param  None
  * @retval None
  */
#pragma anon_unions
const AS3993ControlRegisters_t AS3993_CONTROL_REGISTER_DEFAULT =
{
    {
        {
            {
                .bStatus_rf_on = 0,
                .bStatus_rec_on = 0,
#if RADON || FEMTO2 || FEMTO2_1
                .bStatus_agc_on = 1,
#else
								.bStatus_agc_on = 0,
#endif
                .bStatus_standby = 0,
            },
            // .bReg00_Status = 0x02,
        },
        {
            {
                .bProtocol_protocol = 0,
                .bProtocol_auto_ack = 0,
                .bProtocol_direct_mode = 0,
                .bProtocol_rx_without_crc = 0,
            },
            // .bReg01_Protocol = 0,
        },
        {
            {
#if FEMTO2 || FETMO2_1
                .bTxOptions_tari = TARI_25,
#else
                .bTxOptions_tari = TARI_25,
                // TARI_6_25, TARI_12_5, TARI_25
#endif
                .bTxOptions_tx_one = 3,
            },
            // .bReg02_TxOptions = 0x30,
        },
        {
            {
                .bRxOptions_rx_coding = 2,
                .bRxOptions_TRext = TREXT_ON,
                .bRxOptions_rx_link_frequency = 0x0C,
            },
            // .bReg03_RxOptions = 0xCA,
        },
        {
            {
                .bTRcalHigh_TRcalHigh = 2,
                .bTRcalHigh_gain_id1x5 = 0,
                .bTRcalHigh_gain_id2x = 0,
                .bTRcalHigh_low_vsp_LO = 0,
            },
            // .bReg04_TRcalHigh = 0,
        },
        .bReg05_TRcalLow = 0x9B,
        .bReg06_AutoACKWaitTime = 0x04,
        .bReg07_RxNoResponseTime = 0x0F,
        .bReg08_RxWaitTime = 0x07,
        {
            {
                .bRxFilter_high_pass = 4,
                .bRxFilter_low_pass = 4,
                .bRxFilter_bypass1 = 0,
                .bRxFilter_bypass2 = 0,
            },
            // .bReg09_RxFilterSettings = 0,
        },
        {
            {
                .bRxMixerGain_mixer_input_range = 0,
                .bRxMixerGain_hysteresis_increase = 0,
                .bRxMixerGain_baseband_sign = 1,
                .bRxMixerGain_baseband_gain = 0,
            },
            // .bReg0A_RxMixerGain = 0x20,
        },
        {
            {
                .bVoltageRegulator_rvs = 3,
                .bVoltageRegulator_rvs_rf = 3,
                .bVoltageRegulator_pa_bias_2_times = 0,
#ifdef EXT_PA
                .bVoltageRegulator_pa_bias_4_times = 0,
#else
                .bVoltageRegulator_pa_bias_4_times = 1,
#endif
            },
#ifdef EXT_PA
            // .bReg0B_VoltageRegulator = 0x1B,
#else
            // .bReg0B_VoltageRegulator = 0x9B,
#endif
        },
        {
#ifdef EXT_PA
            {
                .bRFOutputAndLO_rf_low_power = 2,
                .bRFOutputAndLO_main_pa = 0,
                .bRFOutputAndLO_internal_voltage_regulator = 1,
                .bRFOutputAndLO_LO_source = 0,
                .bRFOutputAndLO_LO_gain = 0,
            },
            // .bReg0C_RFOutputAndLO = 0x22,
#else
            {
                .bRFOutputAndLO_rf_low_power = 0,
                .bRFOutputAndLO_main_pa = 2,
                .bRFOutputAndLO_internal_voltage_regulator = 1,
                .bRFOutputAndLO_LO_source = 1,
                .bRFOutputAndLO_LO_gain = 0,
            },
            // .bReg0C_RFOutputAndLO = 0x68,
#endif
        },
        {
            {
                .bMiscellaneous1_single_ended = UHF_DEF_INPUT_MIXERS,
                .bMiscellaneous1_open_drain = 0,
                .bMiscellaneous1_miso_pull_down_1 = 0,
                .bMiscellaneous1_miso_pull_down_2 = 0,
                .bMiscellaneous1_hs_oad = 0,
#if !RADON
                .bMiscellaneous1_hs_output = 1,
#else
                .bMiscellaneous1_hs_output = 0,
#endif
            },
#if !RADON
            // .bReg0D_Miscellaneous1 = 0x80,
#else
            // .bReg0D_Miscellaneous1 = 0,
#endif
        },
        {
            {
#if FEMTO2 || FEMTO2_1 || RADON
                .bMiscellaneous2_clock_output = 4,
#else
                .bMiscellaneous2_clock_output = 4,
#endif
                .bMiscellaneous2_rx_filter_select = 0,
                .bMiscellaneous2_oscillator_mode = 0,
            },
#if FEMTO2 || FEMTO2_1 || RADON
            // .bReg0E_Miscellaneous2 = 0,
#else
            // .bReg0E_Miscellaneous2 = 4,
#endif
        },
        {
            {
                .bMeasurement_adc_select = 0,
                .bMeasurement_analog_test_output = 0,
                .bMeasurement_digital_test_output = 0,
            },
            // .bReg10_Measurement = 0,
        },
        {
            {
                .bVCOControl_manual_vco_range = 0,
                .bVCOControl_internal_osc_bias = 4,
                .bVCOControl_vco_measurement = 0,
            },
            // .bReg11_VCOControl = 0x40,
        },
        {
            {
                .bChargePump_current = 5,
                .bChargePump_loop_filter_c3 = 0,
                .bChargePump_loop_filter_r3 = 0,
            },
            // .bReg12_ChargePump = 0x35,
        },
        {
            {
                .bModulator1_ask_rate = 0,
                .bModulator1_low_pass_filter = 0,
#ifdef EXT_PA
                .bModulator1_aux_to_low_power = 1,
                .bModulator1_main_to_high_power = 0,
#else
                .bModulator1_aux_to_low_power = 0,
                .bModulator1_main_to_high_power = 1,
#endif
            },
#ifdef EXT_PA
            // .bReg13_Modulator1 = 0x20,
#else
            // .bReg13_Modulator1 = 0x40,
#endif
        },
        {
            {
                .bModulator2_delimiter_length = 0x1D,
#if RADON
                .bModulator2_pr_ask_enable = 0,
#else
                .bModulator2_pr_ask_enable = 1,
#endif
                .bModulator2_ook_ask_enable = 1,
            },
#if RADON
            // .bReg14_Modulator2 = 0x9D,
#else
            // .bReg14_Modulator2 = 0xDD,
#endif
        },
        {
            {
                .bModulator3_tx_power_small_adjust = 4,
                .bModulator3_tx_power_big_adjust = 2,
                .bModulator3_linear_select = 0,
                .bModulator3_rf_on_off_time = 0,
            },
            // .bReg15_Modulator3 = 0x14,
        },
        .bReg16_Modulator4_Tari = 0x8A,//0x7E,
        {
            {
                .bPLLMain1_divider_b_msb = 2,
                .bPLLMain1_PLL_divider = 5,
            },
            // .bReg17_PLLMain1 = 0x52,
        },
        {
            {
                .bPLLMain2_divider_a_msb = 0,
                .bPLLMain2_divider_b_lsb = 0x05,
            },
            // .bReg18_PLLMain2 = 0x14,
        },
        .bReg19_PLLMain3_divider_a_lsb = 0x94,
        {
            {
                .bPLLAuxiliary1_divider_b_msb = 4,
            },
            // .bReg1A_PLLAuxiliary1 = 0,
        },
        {
            {
                .bPLLAuxiliary2_divider_a_msb = 1,
                .bPLLAuxiliary2_divider_b_lsb = 0x18,
            },
            // .bReg1B_PLLAuxiliary2 = 0x61,
        },
        .bReg1C_PLLAuxiliary3_divider_a_lsb = 0x18,
        {
            {
                .bICD_IQ_Select_ICD_threshold = 0,
                .bICD_IQ_Select_IQ_threshold = 0,
            },
            // .bReg1D_ICD_IQ_Select = 0,
        },
        {
            {
                .bEmitterCoupledMixer_voltage_range = 0,
                .bEmitterCoupledMixer_sink_current_adjust = 0,
                .bEmitterCoupledMixer_dec_device_bias = 0,
            },
            // .bReg22_EmitterCoupledMixer = 0,
        },
        {
            {
                .bStatusPage_reg2Bpage = 0,
                .bStatusPage_reg2Cpage = 0,
                .bStatusPage_reg2Dpage = 0,
            },
            // .bReg29_StatusPage = 0,
        },
        {
            {
                .bIRQMask1_no_response = 1,
                .bIRQMask1_auto_ack = 1,
                .bIRQMask1_header = 1,
                .bIRQMask1_err = 1,
                .bIRQMask1_fifo = 1,
                .bIRQMask1_rx = 1,
                .bIRQMask1_tx = 1,
            },
            //.bReg35_IRQMask1 = 0xFF,
        },
        {
            {
                .bIRQMask2_err3_preamble_fifo = 0,
                .bIRQMask2_err2_rx_length = 0,
                .bIRQMask2_err1_crc = 0,
                .bIRQMask2_cmd = 0,
                .bIRQMask2_ana_rf = 0,
            },
            // .bReg36_bIRQMask2 = 0xC7,
        },
        {
            {
                .bRxLength_rx_length_msb = 0,
                .bRxLength_auto_error_code = 1,
                .bRxLength_repeat_2nd = 0,
                .bRxLength_direct_2nd = 0,
                .bRxLength_rx_without_crc = 0,
            },
            // .bReg3A_RxLength = 0x10,
        },
        {
            {
                .bTxSettings_gen2_session = 0,
                .bTxSettings_force_TRcal = 0,
                .bTxSettings_tx_crc_type = 0,
            },
            // .bReg3C_TxSettings = 0,
        },
    }
};

/**
  * @brief  This lists the registers for AS3993_CONTROL_REGISTER_DEFAULT
  * @param  None
  * @retval None
  */
const u8 AS3993_CONTROL_REGISTERS[AS3993_NUMBER_OF_CONTROL_REGISTERS]=
{
    AS3993_REG_STATUSCTRL,
    AS3993_REG_PROTOCOLCTRL,
    AS3993_REG_TXOPTIONS,
    AS3993_REG_RXOPTIONS,
    AS3993_REG_TRCALHIGH,
    AS3993_REG_TRCALLOW,
    AS3993_REG_AUTOACKTIMER,
    AS3993_REG_RXNORESPONSEWAITTIME,
    AS3993_REG_RXWAITTIME,
    AS3993_REG_RXFILTER,
    AS3993_REG_RXMIXERGAIN,
    AS3993_REG_REGULATORCONTROL,
    AS3993_REG_RFOUTPUTCONTROL,
    AS3993_REG_MISC1,
    AS3993_REG_MISC2,
    AS3993_REG_MEASUREMENTCONTROL,
    AS3993_REG_VCOCONTROL,
    AS3993_REG_CPCONTROL,
    AS3993_REG_MODULATORCONTROL1,
    AS3993_REG_MODULATORCONTROL2,
    AS3993_REG_MODULATORCONTROL3,
    AS3993_REG_MODULATORCONTROL4,
    AS3993_REG_PLLMAIN1,
    AS3993_REG_PLLMAIN2,
    AS3993_REG_PLLMAIN3,
    AS3993_REG_PLLAUX1,
    AS3993_REG_PLLAUX2,
    AS3993_REG_PLLAUX3,
    AS3993_REG_ICD,
    AS3993_REG_MIXOPTS,
    AS3993_REG_STATUSPAGE,
    AS3993_REG_IRQMASK1,
    AS3993_REG_IRQMASK2,
    AS3993_REG_RXLENGTHUP,
    AS3993_REG_TXSETTING
};

/**
  * @brief  This maps the registers' order to AS3993_CONTROL_REGISTER_DEFAULT
  * @param  None
  * @retval None
  */
const u8 AS3993_CONTROL_REGISTER_INDEXS[AS3993_NUMBER_OF_REGISTERS] =
{       0x00, // 00 STATUSCTRL
        0x01, // 01 PROTOCOLCTRL
        0x02, // 02 TXOPTIONS
        0x03, // 03 RXOPTIONS
        0x04, // 04 TRCALHIGH
        0x05, // 05 TRCALLOW
        0x06, // 06 AUTOACKTIMER
        0x07, // 07 RXNORESPONSEWAITTIME
        0x08, // 08 RXWAITTIME
        0x09, // 09 RXFILTER
        0x0A, // 0A RXMIXERGAIN
        0x0B, // 0B REGULATORCONTROL
        0x0C, // 0C RFOUTPUTCONTROL
        0x0D, // 0D MISC1
        0x0E, // 0E MISC2
        0xFF, // 0F
        0x0F, // 10 MEASUREMENTCONTROL
        0x10, // 11 VCOCONTROL
        0x11, // 12 CPCONTROL
        0x12, // 13 MODULATORCONTROL1
        0x13, // 14 MODULATORCONTROL2
        0x14, // 15 MODULATORCONTROL3
        0x15, // 16 MODULATORCONTROL4
        0x16, // 17 PLLMAIN1
        0x17, // 18 PLLMAIN2
        0x18, // 19 PLLMAIN3
        0x19, // 1A PLLAUX1
        0x1A, // 1B PLLAUX2
        0x1B, // 1C PLLAUX3
        0x1C, // 1D ICD
        0xFF, // 1E
        0xFF, // 1F
        0xFF, // 20
        0xFF, // 21
        0x1D, // 22 MIXOPTS
        0xFF, // 23 TEST1(?)
        0xFF, // 24 TEST2(?)
        0xFF, // 25 TEST3(?)
        0xFF, // 26 TEST4(?)
        0xFF, // 27 TXRESHAPE(?)
        0xFF, // 28
        0x1E, // 29 STATUSPAGE
        0xFF, // 2A AGCANDSTATUS
        0xFF, // 2B RSSILEVELS
        0xFF, // 2C AGL
        0xFF, // 2D ADC
        0xFF, // 2E COMMANDSTATUS
        0xFF, // 2F
        0xFF, // 30
        0xFF, // 31
        0xFF, // 32
        0xFF, // 33 DEVICEVERSION
        0xFF, // 34
        0x1F, // 35 IRQMASK1
        0x20, // 36 IRQMASK2
        0xFF, // 37 IRQSTATUS1
        0xFF, // 38 IRQSTATUS2
        0xFF, // 39 FIFOSTATUS
        0x21, // 3A RXLENGTHUP
        0xFF, // 3B RXLENGTHLOW
        0x22, // 3C TXSETTING
        0xFF, // 3D TXLENGTHUP
        0xFF, // 3E TXLENGTHLOW
        0xFF, // 3F FIFO
};





