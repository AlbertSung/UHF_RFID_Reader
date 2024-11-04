/**
  ******************************************************************************
  * @file    as3993.c
  * @author  Albert
  * @version V1.0.0
  * @date    03-June-2017
  * @brief   as3993 driver routine
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "as3993_registers.h"
#include "string.h"
#include <math.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile u16 g_uAS3993IRQStatus;

// Will be set to 0 if version register is > 0x60. Silicon revision 0x60 needs specific handling in some functions.
static u8 m_bChipRevisionZero = 1;


AS3993ControlRegisters_t m_tAS3993CtrlRegs;
u8 m_bReaderPowerMode = UHF_POWER_MODE_UNKNOWN;

static s8 m_cRSSIGValue = AS3993_INVALID_G_VALUE;

u8 g_bIsRFPowerOff = TRUE;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/******************************************************************************/


/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993WriteByte(u8 bAddress, u8 bValue)
{
    uhfBeginWriteAndRead();

		spiWrite(&bAddress, 1);
    spiWrite(&bValue, 1);

		uhfEndWriteAndRead();

		if(bAddress < AS3993_NUMBER_OF_REGISTERS)
    {
        bAddress = AS3993_CONTROL_REGISTER_INDEXS[bAddress];

				if(bAddress < AS3993_NUMBER_OF_CONTROL_REGISTERS)
            m_tAS3993CtrlRegs.bRegisters[bAddress] = bValue;
    }
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993WriteBytes(u8 bAddress, const u8* pbBuffer, u8 bLength)
{
    if((pbBuffer != NULL) && (bLength > 0))
    {
        uhfBeginWriteAndRead();

				spiWrite(&bAddress, 1);
        spiWrite(pbBuffer, bLength);

				uhfEndWriteAndRead();

				while((bLength > 0) && (bAddress < AS3993_NUMBER_OF_REGISTERS))
        {
            u8 bIndex;

						bIndex = AS3993_CONTROL_REGISTER_INDEXS[bAddress];

						if(bIndex < AS3993_NUMBER_OF_CONTROL_REGISTERS)
                m_tAS3993CtrlRegs.bRegisters[bIndex] = (*pbBuffer);

            bLength--;
            bAddress++;
            pbBuffer++;
        }
    }
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993WriteCmd(u8 bCommand)
{
    uhfBeginWriteAndRead();

		spiWrite(&bCommand, 1);

		uhfEndWriteAndRead();
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993WriteCmdWithBytes(u8 bCommand, u8 bAddress, const u8* pbBuffer, u8 bLength)
{
    if((pbBuffer != NULL) && (bLength > 0))
    {
        uhfBeginWriteAndRead();

				spiWrite(&bCommand, 1);
        spiWrite(&bAddress, 1);
        spiWrite(pbBuffer, bLength);

				uhfEndWriteAndRead();
    }
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u8 as3993ReadByte(u8 bAddress)
{
    u8 bValue = 0;

		uhfBeginWriteAndRead();

		bAddress |= AS3993_SPI_MODE_READ;

		spiWrite(&bAddress, 1);
    spiRead(&bValue, 1);

		uhfEndWriteAndRead();

		return bValue;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993ReadBytes(u8 bAddress, u8* pbBuffer, u8 bLength)
{
    if((pbBuffer != NULL) && (bLength > 0))
    {
        uhfBeginWriteAndRead();

				bAddress |= AS3993_SPI_MODE_READ;

				spiWrite(&bAddress, 1);
        spiRead(pbBuffer, bLength);

				uhfEndWriteAndRead();
    }
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
u8 as3993ReadChipVersion(void)
{
    u8 bVersion;

		bVersion = as3993ReadByte(AS3993_REG_DEVICEVERSION);

		if (bVersion > 0x60)
        m_bChipRevisionZero = 0;

		return bVersion;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
#if RUN_ON_AS3993 || RUN_ON_AS3980 || RUN_ON_AS3994
void as3993ReadBytesForISR(u8 bAddress, u8* pbBuffer, u8 bLength)
{
		UHF_NCS_SELECT();

		bAddress |= AS3993_SPI_MODE_READ;

		spiWrite(&bAddress, 1);
		spiRead(pbBuffer, bLength);

		UHF_NCS_DESELECT();
}
#endif

//void AS3993_IRQ_Callback(u16 IRQ_Pin)
void AS3993_IRQ_Callback(void)
{
#ifdef _SERIAL_DEBUG_
    u8 bIsShowLog;
    bIsShowLog=g_bDbgShowLog;
    g_bDbgShowLog=0;
#endif

    u8 bRegisters[2];

    if (m_bChipRevisionZero)
        delay_us(30);

    as3993ReadBytesForISR(AS3993_REG_IRQSTATUS1, bRegisters, sizeof(bRegisters));

    as3993Response |= (bRegisters[0] | (((u16)bRegisters[1]) << 8));

#ifdef RF_DATA_RECEIVE_LED
    if(AS3993_IRQ1_RX==(bRegisters[0] & (AS3993_IRQ1_RX | AS3993_IRQ1_RXERR)))
        RF_DATA_RECEIVE_LED=UM_LED_ON;
    else
        RF_DATA_RECEIVE_LED=UM_LED_OFF;
#endif

#ifdef _SERIAL_DEBUG_
    if(bIsShowLog)
        putRxTxEvent("ISR", bRegisters, sizeof(bRegisters));
    g_bDbgShowLog=bIsShowLog;
#endif
}

//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993SetAntennaPower(u8 bIsPowerOn)
{
    if(bIsPowerOn)
    {
        if((m_tAS3993CtrlRegs.s.u0.bReg00_Status & AS3993_REG00_TX_RX_MASK) != AS3993_REG00_TX_RX_ENABLE)
        {

#ifdef EXT_PA
            UHF_PAG(UHF_ON);
						// Give PA LDO time to get to maximum power
						delay_us(200);  // RFPA0133 Turn On/Off Time: 200ns
#endif
            m_tAS3993CtrlRegs.s.u0.bReg00_Status |= AS3993_REG00_TX_RX_ENABLE;
            as3993WriteByte(AS3993_REG_STATUSCTRL, m_tAS3993CtrlRegs.s.u0.bReg00_Status);

            if(m_bReaderPowerMode != UHF_POWER_MODE_UNKNOWN)
                m_bReaderPowerMode = UHF_POWER_MODE_NORMAL_RF_ON;
        }

        // according to standard we have to wait 1.5ms before issuing commands
        delay_ms(6);    // Delay even no change regsiter
    }
    else
    {
        if((m_tAS3993CtrlRegs.s.u0.bReg00_Status & AS3993_REG00_TX_RX_MASK) != AS3993_REG00_TX_RX_DISABLE)	// == AS3993_REG00_TX_RX_ENABLE)
        {
            u16 uTry;

						m_tAS3993CtrlRegs.s.u0.bReg00_Status &= (~AS3993_REG00_TX_RX_ENABLE);
            as3993WriteByte(AS3993_REG_STATUSCTRL, m_tAS3993CtrlRegs.s.u0.bReg00_Status);

						// Wait for antenna being switched off
            for(uTry = 0; uTry < 500; uTry++)
            {
                if((as3993ReadByte(AS3993_REG_AGCANDSTATUS) & AS3993_REG2A_RF_OK) != AS3993_REG2A_RF_OK)
                    break;
                delay_us(100);
            }

#ifdef EXT_PA
            UHF_PAG(UHF_OFF);
#endif
#ifdef RF_DATA_RECEIVE_LED
            RF_DATA_RECEIVE_LED = UM_LED_OFF;
#endif
            if(m_bReaderPowerMode != UHF_POWER_MODE_UNKNOWN)
                m_bReaderPowerMode = UHF_POWER_MODE_NORMAL;

            g_bIsRFPowerOff = TRUE;
        }
    }
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 as3993SetReaderPowerMode(u8 bPowerMode)
{
		s8 cResult = ERR_GEN2_PARAM;

		if((bPowerMode <= UHF_POWER_MODE_NORMAL_RF_ON) && (m_bReaderPowerMode != bPowerMode))
    {
        cResult = ERR_GEN2_OK;
				/* to enter power down mode */
        if(bPowerMode == UHF_POWER_MODE_POWER_DOWN)
        {
            int i;

			// Switch off antenna
            as3993WriteByte(AS3993_REG_STATUSCTRL, (m_tAS3993CtrlRegs.s.u0.bReg00_Status & (~AS3993_REG00_TX_RX_MASK)));

						// Wait for antenna being switched off
            i = 500;
            while(i-- && (as3993ReadByte(AS3993_REG_AGCANDSTATUS) & AS3993_REG2A_RF_OK))
                delay_ms(1);

            UHF_EN(OUT_LOW);
        }
				/* to exit power down mode */
        else if((m_bReaderPowerMode == UHF_POWER_MODE_POWER_DOWN) || (m_bReaderPowerMode == UHF_POWER_MODE_UNKNOWN))
        {
            u8 bStatus;

						UHF_EN(OUT_HIGH);
            delay_us(10);
            as3993WaitForStartup();

            if(m_bReaderPowerMode == UHF_POWER_MODE_UNKNOWN)
            {
                m_bReaderPowerMode = UHF_POWER_MODE_POWER_DOWN;

								if(as3993Initialize() != ERR_CHIP_OK)
                    m_bReaderPowerMode = UHF_POWER_MODE_UNKNOWN;
            }

            // Do not switch on antenna before PLL is locked.
            m_tAS3993CtrlRegs.s.u33.bReg3A_RxLength &= AS3993_REG3A_RX_INT_SETTINGS_MASK;

            bStatus = m_tAS3993CtrlRegs.s.u0.bReg00_Status;
            m_tAS3993CtrlRegs.s.u0.bReg00_Status &= (~AS3993_REG00_TX_RX_MASK);
            as3993RestoreRegisters();

            m_tAS3993CtrlRegs.s.u0.bReg00_Status = bStatus;

            delay_us(300);
            cResult = as3993WaitingLockPLL();

            as3993WriteByte(AS3993_REG_STATUSCTRL, bStatus);

            WaitForAS3980();
        }

				/* to enter other modes */
        if(bPowerMode != m_bReaderPowerMode)
        {
            u8 bStatus;

						switch(bPowerMode)
            {
            case UHF_POWER_MODE_POWER_DOWN:
                break;
            case UHF_POWER_MODE_STANDBY:
                bStatus = ((m_tAS3993CtrlRegs.s.u0.bReg00_Status & AS3993_REG00_AGC_ENABLE) | AS3993_REG00_STANDBY);
                as3993WriteByte(AS3993_REG_STATUSCTRL, bStatus);
                break;
            case UHF_POWER_MODE_NORMAL_REC_ON:
                bStatus = ((m_tAS3993CtrlRegs.s.u0.bReg00_Status & AS3993_REG00_AGC_ENABLE) | AS3993_REG00_RX_ENABLE);
                as3993WriteByte(AS3993_REG_STATUSCTRL, bStatus);

								if(m_bReaderPowerMode <= UHF_POWER_MODE_NORMAL)
                    delay_ms(1);
                break;
            case UHF_POWER_MODE_NORMAL_RF_ON:
                as3993SetAntennaPower(UHF_ON);
                break;
            default:    //case UHF_POWER_MODE_NORMAL:
                as3993SetAntennaPower(UHF_OFF);
                break;
            }
            if(bPowerMode != UHF_POWER_MODE_NORMAL_RF_ON)
                g_bIsRFPowerOff = TRUE;
        }

        if(m_bReaderPowerMode != UHF_POWER_MODE_UNKNOWN)
            m_bReaderPowerMode = bPowerMode;

#ifdef ALB_DEBUG
        printf("%09lu New Power Mode: %d, IsRFPowerOff: %d \r\n", getSysMilliseconds(), m_bReaderPowerMode, g_bIsRFPowerOff);
#endif
    }
		
    return cResult;
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
static s8 as3993WaitingLockPLL(void)
{
    u8 i, bVoltage, bStatus;
    s8 cResult;

    bStatus = m_tAS3993CtrlRegs.s.u30.bReg29_StatusPage;
    // have vco_ri in aglstatus
    bStatus = ((bStatus & (~AS3993_REG29_SEL_REG2C_MASK)) | AS3993_REG29_SEL_REG2C_VCO);
    as3993WriteByte(AS3993_REG_STATUSPAGE, bStatus);

    bStatus = m_tAS3993CtrlRegs.s.uu16.bReg11_VCOControl;
    // set mvco bit
    bStatus |= AS3993_REG11_VCO_MEAS_ENABLE;
    as3993WriteByte(AS3993_REG_VCOCONTROL, bStatus);
    // give PLL some settling time, should be around 500us
    delay_ms(1);

    bVoltage = (as3993ReadByte(AS3993_REG_AGL) & AS3993_REG2C_VCO_VOL_RESULT_MASK);

    // clear mvco bit
    bStatus &= (~AS3993_REG11_VCO_MEAS_ENABLE);
    as3993WriteByte(AS3993_REG_VCOCONTROL, bStatus);

    cResult = ERR_CHIP_OK;
    if((bVoltage <= 1) || (bVoltage >= 6))
    {
        cResult = ERR_CHIP_CRYSTAL;
        // wait for PLL to be locked and give a few attempts
        for(i = 0 ; i < 3 ; i++)
        {
            as3993WriteCmd(AS3993_CMD_VCO_AUTO_RANGE);
            // Please keep in mind, that the Auto Bit procedure will take app. 6 ms whereby the locktime of PLL is just 400us
            delay_ms(10);
            bStatus = as3993ReadByte(AS3993_REG_AGCANDSTATUS);

            if((bStatus & AS3993_REG2A_PLL_OK) != 0)
            {
                cResult = ERR_CHIP_OK;
                break;
            }
        }
    }
		
#ifdef ALB_DEBUG
		printf("%09lu Wait PLL lock result %+d with step %d \r\n", getSysMilliseconds(), cResult, bVoltage);
#endif
		
    return cResult;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void as3993WaitForStartup(void)
{
    u8 bOSC_OK, bVersion;
    u8 bBuffer[2];
    u16 uCount;

    for(uCount = 0; uCount < 250; uCount++)
    {
        bVersion = as3993ReadChipVersion();
        bOSC_OK = as3993ReadByte(AS3993_REG_AGCANDSTATUS);
        // wait for startup
        if(((bVersion & 0x60) == 0x60) && (bOSC_OK & 0x01))
            break;
    }

    delay_ms(3);

    as3993ReadBytes(AS3993_REG_IRQSTATUS1, &bBuffer[0], 2);    // ensure that IRQ bits are reset
    as3993ClrResponse();
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void as3993ResetWithoutSaveRegisters(void)
{
    UHF_EN(OUT_LOW);

    delay_ms(10);

    UHF_EN(OUT_HIGH);

    delay_ms(10);

    as3993WaitForStartup();
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
static void as3993RestoreRegisters(void)
{
    int i;

		for(i = 0; i < sizeof(AS3993_CONTROL_REGISTERS); i++)
        as3993WriteByte(AS3993_CONTROL_REGISTERS[i], m_tAS3993CtrlRegs.bRegisters[i]);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
static void as3993Reset(void)
{
		as3993SetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);

		delay_ms(1);

		as3993SetReaderPowerMode(UHF_POWER_MODE_NORMAL);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993WaitForResponse(u16 uWaitMask)
{
    u32 ulCounter;
    ulCounter = WAIT_FOR_RESPONSE_COUNT;

		while(((as3993Response & uWaitMask) == 0) && (ulCounter > 0))
    {
        ulCounter--;
        delay_us(WAIT_FOR_RESPONSE_DELAY);
    }

    if(ulCounter == 0)
    {
#ifdef _SERIAL_DEBUG_
        printf("%09lu as3993WaitForResponse: Timeout\r\n", getSysMilliseconds());
        dbg_waitForPrint();
#endif
				as3993Reset();

				as3993Response = RESP_NORESINTERRUPT;
    }
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993WaitForResponseTimed(u16 uWaitMask, u32 ulCounter)
{
    while(((as3993Response & uWaitMask) == 0) && (ulCounter > 0))
    {
        ulCounter--;
        delay_us(WAIT_FOR_RESPONSE_DELAY);
    }

    if(ulCounter == 0)
    {
#ifdef _SERIAL_DEBUG_
        printf("%09lu as3993WaitForResponseTimed: Timeout\r\n", getSysMilliseconds());
        dbg_waitForPrint();
#endif
        as3993Reset();

				as3993Response = RESP_NORESINTERRUPT;
    }
}

// ADC Values are in sign magnitude representation -> convert
#define CONVERT_ADC_TO_NAT(ADCValue)   (((ADCValue) & 0x80) ? ((ADCValue) & 0x7F) : (0 - ((ADCValue) & 0x7F)))

/**
  * @brief  
  * @param  
  * @retval 
  */
static s8 as3993GetADC(u8 bADCSelect)
{
    s8 cValue = 0;
    u8 i;

		// Set 2Dh register for ADC result
		as3993WriteByte(AS3993_REG_STATUSPAGE, ((m_tAS3993CtrlRegs.s.u30.bReg29_StatusPage & (~AS3993_REG29_SEL_REG2D_MASK)) | AS3993_REG29_SEL_REG2D_ADC));
		// Select ADC measurement input
    as3993WriteByte(AS3993_REG_MEASUREMENTCONTROL, bADCSelect);

		delay_us(300); // wait for settling time

		as3993WriteCmd(AS3993_CMD_TRIGGERADCCON);

		delay_us(20); // according to spec

		for(i = 0; i < 3; i++)
    {
        if((as3993ReadByte(AS3993_REG_COMMANDSTATUS) & AS3993_REG2E_ADC_CONV_DONE) == AS3993_REG2E_ADC_CONV_DONE)
        {
            cValue = (s8)as3993ReadByte(AS3993_REG_ADC);
            break;
        }
        delay_us(10);
    }
		
    return CONVERT_ADC_TO_NAT(cValue);
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
IQValue_t as3993GetMixerIQLevel(void)
{
    IQValue_t tIQValue;
    u8 bRegMeas;

		bRegMeas = m_tAS3993CtrlRegs.s.u15.bReg10_Measurement;

		// disable the OAD pin outputs
		as3993WriteByte(AS3993_REG_MEASUREMENTCONTROL, bRegMeas & (~AS3993_REG10_DTO_OAD_MASK));

    // Reset the receiver - otherwise the I values seem to oscillate
    as3993WriteCmd(AS3993_CMD_BLOCKRX);
    as3993WriteCmd(AS3993_CMD_ENABLERX);

		tIQValue.st1.cIValue = as3993GetADC(AS3993_REG10_ADC_MEAS_MIXER_DC_I);
    tIQValue.st1.cQValue = as3993GetADC(AS3993_REG10_ADC_MEAS_MIXER_DC_Q);

		as3993WriteCmd(AS3993_CMD_BLOCKRX);

    as3993WriteByte(AS3993_REG_MEASUREMENTCONTROL, bRegMeas);

    return tIQValue;
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
static u8 as3993GetRawRSSI(void)
{
		u8 bRSSIValue;

		as3993WriteCmd(AS3993_CMD_BLOCKRX);
    as3993WriteCmd(AS3993_CMD_ENABLERX);

		// According to architecture note we have to wait here at least 100us however experiments show 350us to be necessary on AS3992
    delay_us(500);

		bRSSIValue = as3993ReadByte(AS3993_REG_RSSILEVELS);

		as3993WriteCmd(AS3993_CMD_BLOCKRX);

		return bRSSIValue;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 as3993GetMaxRSSIEx(u16 uCountWithMilliseconds)
{
    s16 iMax_dBm;
    IQValue_t tIQValue;
    u8 bRegStatus;//, bRegFilter;

    uCountWithMilliseconds = (uCountWithMilliseconds << 1);

		if(uCountWithMilliseconds == 0)
        uCountWithMilliseconds++;

    bRegStatus = m_tAS3993CtrlRegs.s.u0.bReg00_Status;

    // Receiver on and transmitter are off
    as3993WriteByte(AS3993_REG_STATUSCTRL, AS3993_REG00_RX_ENABLE);
    as3993WriteByte(AS3993_REG_RXFILTER, 0x24); // 0xFF /* Optimal filter settings */

    if(!(bRegStatus & AS3993_REG00_RX_ENABLE))
    {   // rec_on needs about 6ms settling time, to be sure wait 10 ms
        delay_ms(10);
    }

    tIQValue.uIQValue = 0;
    iMax_dBm = (-32768);

    while(uCountWithMilliseconds)
    {
        s16 iTemp;
        IQValue_t tIQTemp;

				uCountWithMilliseconds--;

				tIQTemp = as3993GetMixerIQLevel();
        iTemp = (tIQTemp.st1.cIValue + tIQTemp.st1.cQValue);

				if(iMax_dBm < iTemp)
        {
            iMax_dBm = iTemp;
            tIQValue.uIQValue = tIQTemp.uIQValue;
        }
    }

    iMax_dBm = sqrt((tIQValue.st1.cIValue * tIQValue.st1.cIValue) + (tIQValue.st1.cQValue * tIQValue.st1.cQValue));

    if(m_cRSSIGValue == AS3993_INVALID_G_VALUE)
        m_cRSSIGValue = as3993GetGainEffectRSSIValue();

    iMax_dBm += m_cRSSIGValue;

    return ((s8)iMax_dBm);
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
s8 as3993GetRxMixerGainValue(void)
{
    s8 cGValue;

		// Gain in/de-crease
    cGValue = ((m_tAS3993CtrlRegs.s.u10.bReg0A_RxMixerGain & AS3993_REG0A_BASEGAIN_MASK) >> 6);
    cGValue = (cGValue + (cGValue << 1)); // cGValue=(cGValue*3);

		if((m_tAS3993CtrlRegs.s.u10.bReg0A_RxMixerGain & AS3993_REG0A_INC_BASEGAIN) == AS3993_REG0A_INC_BASEGAIN)
        cGValue = (0 - cGValue);	// ?

		if(m_tAS3993CtrlRegs.s.u13.bReg0D_Miscellaneous1 & AS3993_REG0D_SINGLE_ENDED)
    {
				// single-ended input mixer
        switch(m_tAS3993CtrlRegs.s.u10.bReg0A_RxMixerGain & AS3993_REG0A_MIX_RX_MIXER_MASK)
        {
        case AS3993_REG0A_MIX_SE_DEC_6DBM:
            cGValue += 6;
            break;
        case AS3993_REG0A_MIX_SE_INC_6DBM:
            cGValue -= 6;
            break;
        }
    }
    else
    {
				// differential input mixer
        switch(m_tAS3993CtrlRegs.s.u10.bReg0A_RxMixerGain & AS3993_REG0A_MIX_RX_MIXER_MASK)
        {
        case AS3993_REG0A_MIX_DIFF_DEC_8DBM:
            cGValue += 8;
            break;
        case AS3993_REG0A_MIX_DIFF_INC_10DBM:
            cGValue -= 10;
            break;
        }
    }

    return cGValue;
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
s8 as3993GetGainEffectRSSIValue(void)
{
    s8 cGRSSIValue;

		if(m_tAS3993CtrlRegs.s.u13.bReg0D_Miscellaneous1 & AS3993_REG0D_SINGLE_ENDED)
    {
				// single-ended input mixer
        cGRSSIValue = AS3993_SINGLE_ENDED_G_VALUE;
    }
    else
    {
				// differential input mixer
        cGRSSIValue = AS3993_DIFFERENTIAL_G_VALUE;
    }

    cGRSSIValue += as3993GetRxMixerGainValue();

    return cGRSSIValue;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993GetAgcRssi(s8* pcAGC, s8* pcRSSI, u8* pbIQValue)
{
    u8 bBuffer[2];
    s8 cAGC, cRSSI;

		as3993ReadBytes(AS3993_REG_AGCANDSTATUS, bBuffer, sizeof(bBuffer));

		cAGC = ((bBuffer[0] & AS3993_REG2A_AGC_MASK) >> 4);
    cRSSI = (((bBuffer[1] >> 4) & 0x0F) + (bBuffer[1] & 0x0F));

		*pbIQValue = cRSSI;		// s8 -> u8 ?

		if(m_cRSSIGValue == AS3993_INVALID_G_VALUE)
        m_cRSSIGValue = as3993GetGainEffectRSSIValue();

    if(pcRSSI != NULL)
        (*pcRSSI) = (((2.1 * cRSSI) / 2) + m_cRSSIGValue);

    if(pcAGC != NULL)
    {
        if(cAGC > 4)
            (*pcAGC) = (m_cRSSIGValue + (cAGC + (cAGC<<1)));        //* TODO Kevin 1001 need to modify. 
        else
            (*pcAGC) = (m_cRSSIGValue + cAGC);
    }
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 as39932GetRssi(u8 bRSSISelect, u8* pbIQValue)
{
    u8 bBuffer[2];
    s8 cAGC, cRSSI;

		bRSSISelect = (bRSSISelect & AS3993_REG29_SEL_REG2B_RSSI_MASK);

		if(bRSSISelect != m_tAS3993CtrlRegs.s.u30.s30.bStatusPage_reg2Bpage)
    {
        bRSSISelect |= (m_tAS3993CtrlRegs.s.u30.bReg29_StatusPage & (~AS3993_REG29_SEL_REG2B_RSSI_MASK));
        as3993WriteByte(AS3993_REG_STATUSPAGE, bRSSISelect);
    }

    as3993ReadBytes(AS3993_REG_AGCANDSTATUS, bBuffer, sizeof(bBuffer));

    cAGC = ((bBuffer[0] & AS3993_REG2A_AGC_MASK) >> 4);
    cRSSI = (((bBuffer[1] >> 4) & 0x0F)+(bBuffer[1] & 0x0F));
		
		*pbIQValue = bBuffer[1];

    if(m_cRSSIGValue == AS3993_INVALID_G_VALUE)
        m_cRSSIGValue = as3993GetGainEffectRSSIValue();

    // In the document the G value is positive
    // If AGC>4 RSSI=G-3db*(AGC-4), If AGC<=4 RSSI=G
    // (*pbAGC)=(m_cRSSIGValue+(cAGC*3));

    cAGC -= 4;

    if(cAGC > 0)
        cAGC = (m_cRSSIGValue+(cAGC+(cAGC<<1)));
    else
        cAGC = m_cRSSIGValue;

    // In the document the G value is positive
    // meanRSSI=(RSSI(I)+RSSI(Q))/2
    // Pin(dBm)=2.1*meanRSSI-G
    // G is a constant depending on the register settings of the RX Filter Settings Register and the RX Mixer and Gain Register.

    return (((2.1 * cRSSI) / 2) + cAGC);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 as3993GetMaxRSSI(u16 uCountWithMilliseconds, u8* pbIQRAWValue)
{
    u8 bIQValue, bValue;
    s8 cMax_dBm;
    u8 bRegStatus, bRegProtocol, bRegFilter;

    uCountWithMilliseconds = (uCountWithMilliseconds << 1);

		if(uCountWithMilliseconds == 0)
        uCountWithMilliseconds++;

    bRegStatus = m_tAS3993CtrlRegs.s.u0.bReg00_Status;

    // Receiver on and transmitter are off
    as3993WriteByte(AS3993_REG_STATUSCTRL, AS3993_REG00_RX_ENABLE);

    bRegProtocol = m_tAS3993CtrlRegs.s.u1.bReg01_Protocol;
    as3993WriteByte(AS3993_REG_PROTOCOLCTRL, (bRegProtocol | AS3993_REG01_DECODER_DISABLE));

    as3993WriteByte(AS3993_REG_IRQMASK1, (m_tAS3993CtrlRegs.s.u31.bReg35_IRQMask1 & (~AS3993_REG35_IRQ_NORESP_ENABLE)));

    bValue = m_tAS3993CtrlRegs.s.u30.bReg29_StatusPage;
    // set real time rssi in rssi register
    bValue = ((bValue & (AS3993_REG29_SEL_REG2D_MASK | AS3993_REG29_SEL_REG2C_MASK)) | AS3993_REG29_SEL_REG2B_REAL_TIME);
    as3993WriteByte(AS3993_REG_STATUSPAGE, bValue);

    bRegFilter = m_tAS3993CtrlRegs.s.u9.bReg09_RxFilterSettings;
    as3993WriteByte(AS3993_REG_RXFILTER, 0x24); // 0xFF /* Optimal filter settings */

		if(!(bRegStatus & AS3993_REG00_RX_ENABLE))
    {   // rec_on needs about 6ms settling time, to be sure wait 10 ms
        delay_ms(10);
    }

    cMax_dBm = 0;
    bIQValue = 0;

    while(uCountWithMilliseconds)
    {
        s8 cTemp;

				uCountWithMilliseconds--;

				bValue = as3993GetRawRSSI();

				cTemp = (((bValue >> 4) & 0x0F) + (bValue & 0x0F));

				if(cMax_dBm < cTemp)
        {
            cMax_dBm = cTemp;
            bIQValue = bValue;
        }
    }

    if(cMax_dBm != 0)
    {
        // meanRSSI=(RSSI(I)+RSSI(Q))/2
        // Pin(dBm)=2.1*meanRSSI-G
        // G is a constant depending on the register settings of the RX Filter Settings Register and the RX Mixer and Gain Register.

				cMax_dBm = ((2.1 * cMax_dBm) / 2);

				if(m_cRSSIGValue == AS3993_INVALID_G_VALUE)
            m_cRSSIGValue = as3993GetGainEffectRSSIValue();

        cMax_dBm += m_cRSSIGValue;
    }
    else
    {
				// short exit, below formula does not work for 0 value
        cMax_dBm = (-128);
        bIQValue = 0;
    }

    if(pbIQRAWValue != NULL)
        (*pbIQRAWValue) = bIQValue;

    as3993WriteByte(AS3993_REG_RXFILTER, bRegFilter); // Restore filter
    as3993WriteByte(AS3993_REG_IRQMASK1, (m_tAS3993CtrlRegs.s.u31.bReg35_IRQMask1 | AS3993_REG35_IRQ_NORESP_ENABLE));
    as3993WriteByte(AS3993_REG_PROTOCOLCTRL, bRegProtocol);
    as3993WriteByte(AS3993_REG_STATUSCTRL, bRegStatus);

    if(bRegStatus & AS3993_REG00_TX_ENABLE)
    {
				// according to standard we have to wait 1.5ms before issuing commands
        delay_ms(2);
    }

    return cMax_dBm;
}

const PLL_DIVIDER_T m_tPLLDividers[] =
{
    {(u8)AS3993_REG17_PLL_DIVIDER_125K, (u8)125},
    {(u8)AS3993_REG17_PLL_DIVIDER_100K, (u8)100},
    {(u8)AS3993_REG17_PLL_DIVIDER_50K, (u8)50},
    {(u8)AS3993_REG17_PLL_DIVIDER_25K, (u8)25}
};

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993SetBaseFrequency(u8 bPLLRegister, u32 ulBaseFrequency, u8 bPLLDivUser)
{
    u8 bBuffer[3];
    u8 bStatusReg, bPLLDivisor;
    s32 lDivisor;
    u16 uValueA, uValueB;
    u8 i;

    ulBaseFrequency = ulBaseFrequency - DEVIATION_FREQUENCY;

    if(ulBaseFrequency < AS3993_MIN_TARGET_FREQUENCY)
        ulBaseFrequency = AS3993_MIN_TARGET_FREQUENCY;
    else if(ulBaseFrequency > AS3993_MAX_TARGET_FREQUENCY)
        ulBaseFrequency = AS3993_MAX_TARGET_FREQUENCY;

		// only for PLL main or auxiliary divider
    if(bPLLRegister != AS3993_REG_PLLAUX1)
        bPLLRegister = AS3993_REG_PLLMAIN1;

    bStatusReg = m_tAS3993CtrlRegs.s.u0.bReg00_Status;
    as3993WriteByte(AS3993_REG_STATUSCTRL, (bStatusReg & (~AS3993_REG00_TX_ENABLE)));

    bBuffer[0] = m_tAS3993CtrlRegs.s.u22.bReg17_PLLMain1;
    bPLLDivisor = (bBuffer[0] & AS3993_REG17_PLL_DIVIDER_MASK);

    if((bPLLDivisor < AS3993_REG17_PLL_DIVIDER_125K) || (bPLLDivisor > AS3993_REG17_PLL_DIVIDER_25K))
        bPLLDivisor = AS3993_REG17_PLL_DIVIDER_25K;

		// read original setting if it exist
    bPLLDivisor = m_tPLLDividers[(bPLLDivisor - AS3993_REG17_PLL_DIVIDER_125K) >> AS3993_REG17_PLL_DIVIDER_POS].bDividerKHz;

		// change PLL reference divider if user defines this
		if(bPLLDivUser != NULL)
				bPLLDivisor = bPLLDivUser;
		
		lDivisor = (ulBaseFrequency / bPLLDivisor);

		uValueA = (lDivisor / (33 + 32));
    lDivisor = (lDivisor - (uValueA * (33+32)));
    uValueB = uValueA;

		if(lDivisor >= 33)
    {
        uValueA++;
        lDivisor -= 33;
    }

    if(lDivisor >= 32)
    {
        uValueB++;
        lDivisor -= 32;
    }

    uValueA += lDivisor;
    uValueB -= lDivisor;

    bBuffer[0] = 0x00;    // No change when register is AS3993_REG_PLLAUX1

    if(bPLLRegister == AS3993_REG_PLLMAIN1)
    {
        for(i = 0; i < (sizeof(m_tPLLDividers) / sizeof(PLL_DIVIDER_T)); i++)
        {
            if(bPLLDivisor == m_tPLLDividers[i].bDividerKHz)
            {
                bBuffer[0] = m_tPLLDividers[i].bRegValue;
                break;
            }
        }
    }

    bBuffer[0] |= ((uValueB >> 6) & 0x0Fu);
    bBuffer[1] = ((uValueB << 2) & 0xFCu) | ((uValueA >> 8) & 0x03u);
    bBuffer[2] = (uValueA & 0xFF);

    as3993WriteBytes(bPLLRegister, bBuffer, 3);

    as3993WaitingLockPLL();

    as3993WriteByte(AS3993_REG_STATUSCTRL, bStatusReg);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 as3993SetSensitivity(s8 cSensitivity)
{
    u8 bGainSettings;
    u8 bBaseband;

		cSensitivity -= AS3993_NOMINAL_SENSITIVITY;

    bGainSettings = m_tAS3993CtrlRegs.s.u10.bReg0A_RxMixerGain;
		bGainSettings &= AS3993_REG0A_HYSTERESIS_INC_MASK;

    if(m_tAS3993CtrlRegs.s.u13.bReg0D_Miscellaneous1 & AS3993_REG0D_SINGLE_ENDED)
    {
				// single ended input mixer
        if(cSensitivity >= 6)
        {
            cSensitivity -= 6;	// 6 dB gain decrease
            bGainSettings |= AS3993_REG0A_MIX_SE_DEC_6DBM;
        }
        else if(cSensitivity <= (-6))
        {
            cSensitivity += 6;	// 6 dB gain increase
            bGainSettings |= AS3993_REG0A_MIX_SE_INC_6DBM;
        }
        else
            bGainSettings |= AS3993_REG0A_MIX_SE_NOMINAL;
    }
    else
    {
				// differential input mixer
        if(cSensitivity >= 8)
        {
            cSensitivity -= 8;	// 8 dB gain attenuation
            bGainSettings |= AS3993_REG0A_MIX_DIFF_DEC_8DBM;
        }
        else if(cSensitivity <= (-10))
        {
            cSensitivity += 10;	// 10 dB gain increase
            bGainSettings |= AS3993_REG0A_MIX_DIFF_INC_10DBM;
        }
    }

    if(cSensitivity > 0)
    {
        // RX Gain direction: decrease
        bGainSettings &= (~AS3993_REG0A_INC_BASEGAIN);

				bBaseband = (cSensitivity / 3);

				if(bBaseband > 3)
            bBaseband = 3;

        cSensitivity -= (bBaseband * 3);
    }
    else
    {
        // RX Gain direction: increase
        bGainSettings |= AS3993_REG0A_INC_BASEGAIN;

				bBaseband = ((0 - cSensitivity) / 3);		// ((0 - cSensitivity + 2) / 3) ?

				if(bBaseband > 3)
            bBaseband = 3;

        cSensitivity += (bBaseband * 3);
    }

    bGainSettings |= (bBaseband << AS3993_REG0A_BASEGAIN_POS);

    as3993WriteByte(AS3993_REG_RXMIXERGAIN, bGainSettings);

		m_cRSSIGValue = as3993GetGainEffectRSSIValue();		// ?

#ifdef ALB_DEBUG
		printf("%09lu Set sensitivity with %+d \r\n", getSysMilliseconds(), cSensitivity);
#endif
		
    return cSensitivity;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 as3993SetRFTxPower(s8 cPower)
{
    u8 bRegister;

    bRegister = m_tAS3993CtrlRegs.s.u20.bReg15_Modulator3;
    bRegister &= (~AS3993_REG15_RF_TX_POWER_MASK);

    if(cPower <= (-12))
    {
        bRegister |= AS3993_REG15_RF_TX_POWER_DEC_12DB;
        cPower -= (-12);
    }
    else if(cPower <= (-8))
    {
        bRegister |= AS3993_REG15_RF_TX_POWER_DEC_8DB;
        cPower -= (-8);
    }

    if(cPower >= (-7))
    {
        bRegister |= ((0 - cPower) & AS3993_REG15_RF_TX_POWER_1DB_MASK);
        cPower = 0;
    }
    else
    {
        bRegister |= AS3993_REG15_RF_TX_POWER_DEC_7DB;
        cPower += (-7);		// ?
    }

    as3993WriteByte(AS3993_REG_MODULATORCONTROL3, bRegister);

    return cPower;
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
u8 as3993GetReaderPowerMode(void)
{
    return m_bReaderPowerMode;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993SetAutoAckMode(u8 uAutoAckMode)
{
    u8 bRegister;

		bRegister = (m_tAS3993CtrlRegs.s.u1.bReg01_Protocol & (~AS3993_REG01_AUTO_ACK_MASK));

		if(uAutoAckMode == UHF_AUTO_MODE_ACK)
        bRegister |= AS3993_REG01_AUTO_ACK_ENABLE;
    else if(uAutoAckMode == UHF_AUTO_MODE_ACK_REQ_RN)
        bRegister |= AS3993_REG01_AUTO_ACK_WITH_REQRN;

    as3993WriteByte(AS3993_REG_PROTOCOLCTRL, bRegister);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993SetGen2Session(u8 bSession)
{
    u8 bRegister;

		bRegister = m_tAS3993CtrlRegs.s.u34.bReg3C_TxSettings;

		if(bSession > AS3993_REG3C_GEN2_SESSION_S1S0_3)
        bSession = AS3993_REG3C_GEN2_SESSION_S1S0_0;

    bRegister = ((bRegister & (~AS3993_REG3C_GEN2_SESSION_MASK)) | bSession);

    as3993WriteByte(AS3993_REG_TXSETTING, bRegister);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void as3993SetGen2Protocol(void)
{
    u8 bRegister;

		bRegister = m_tAS3993CtrlRegs.s.u1.bReg01_Protocol;

		bRegister = ((bRegister & (~AS3993_REG01_PROTOCOL_MASK)) | AS3993_REG01_PROTOCOL_GEN2);

		as3993WriteByte(AS3993_REG_PROTOCOLCTRL, bRegister);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 as3993IsValidFrequency(u32 ulFrequency)  // EQU is 0, > is 1, < is -1
{
    s8 cResult = 0;

		if(ulFrequency < AS3993_MIN_TARGET_FREQUENCY)
        cResult = -1;
    else if(ulFrequency > AS3993_MAX_TARGET_FREQUENCY)
        cResult = 1;

    return cResult;
}

//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  
  * @retval None
  */
void as3993DirectWriteByte(u8 bAddress, u8 bValue)
{
    uhfBeginWriteAndRead();

		spiWrite(&bAddress, 1);

		spiWrite(&bValue, 1);

		uhfEndWriteAndRead();
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
static u8 as3993EnterMeasureRSSIMode(void)
{
    u8 bPowerMode;

		bPowerMode = m_bReaderPowerMode;

    as3993SetReaderPowerMode(UHF_POWER_MODE_NORMAL);

    // Turn off low power output because UM800L using RF source from low power output
    // And turn on internal PA but internal PA is not connect to output, so it's let no RF power output
    // Choice 1: (UM800L) Pins RFOPX and RFONX should be connected to VDD_B via two 100 O resistors.
    // Choice 2: The TX ports of the internal PA can be left unconnected but a 100nF capacitor is required at the VDD_PA pin.
    as3993DirectWriteByte(AS3993_REG_RFOUTPUTCONTROL, (AS3993_REG0C_LO_SOURCE_INTERNAL_PA | AS3993_REG0C_MAIN_PA_7MA));

    // In the Protocol Selection Register set the two bits rf_on and rec_on to 1 (Receiver and transmitter on)
		as3993DirectWriteByte(AS3993_REG_STATUSCTRL, (m_tAS3993CtrlRegs.s.u0.bReg00_Status | AS3993_REG00_TX_RX_ENABLE));

		// Set the dir_mode bit in the Protocol Selection Register to 1
    as3993DirectWriteByte(AS3993_REG_PROTOCOLCTRL, (m_tAS3993CtrlRegs.s.u1.bReg01_Protocol | AS3993_REG01_DECODER_DISABLE));

    // Disable the no response interrupt in the Enable Interrupt Register 1 by setting the e_irq_noresp bit to 0.
    as3993DirectWriteByte(AS3993_REG_IRQMASK1, (m_tAS3993CtrlRegs.s.u31.bReg35_IRQMask1 & (~AS3993_REG35_IRQ_NORESP_ENABLE)));

    // The RSSI Display Register needs to be configured to show the Real time RSSI information through the Status Readout Page Setting Register (register 0x29)
    as3993WriteByte(AS3993_REG_STATUSPAGE, ((m_tAS3993CtrlRegs.s.u30.bReg29_StatusPage & (AS3993_REG29_SEL_REG2D_MASK | AS3993_REG29_SEL_REG2C_MASK)) | AS3993_REG29_SEL_REG2B_REAL_TIME));

    return bPowerMode;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
static void as3993ExitMeasureRSSIMode(u8 bPowerMode)
{
	  as3993DirectWriteByte(AS3993_REG_IRQMASK1, m_tAS3993CtrlRegs.s.u31.bReg35_IRQMask1);

		as3993DirectWriteByte(AS3993_REG_PROTOCOLCTRL, m_tAS3993CtrlRegs.s.u1.bReg01_Protocol);

		as3993DirectWriteByte(AS3993_REG_STATUSCTRL, m_tAS3993CtrlRegs.s.u0.bReg00_Status);

		as3993DirectWriteByte(AS3993_REG_RFOUTPUTCONTROL, m_tAS3993CtrlRegs.s.u12.bReg0C_RFOutputAndLO);

		as3993SetReaderPowerMode(bPowerMode);
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
static u8 as3993MeasureIQLevels(void)
{
    u8 bIQLevels;

		as3993WriteCmd(AS3993_CMD_ENABLERX);

		delay_us(500);

		bIQLevels = as3993ReadByte(AS3993_REG_RSSILEVELS);

		as3993WriteCmd(AS3993_CMD_BLOCKRX);

		return bIQLevels;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
s8 as3993MeasureMaxRSSI(u16 uMeasureTimeMilliseconds, s8 cCenterRSSI)
{
    s8 cSaveSensitivity;
    u8 bPowerMode, bIQLevels;
    s8 cMaxRSSI, cRSSI;

    if(uMeasureTimeMilliseconds == 0)
        uMeasureTimeMilliseconds++;

    bPowerMode = as3993EnterMeasureRSSIMode();

    cSaveSensitivity = as3993GetSensitivity();

    // meanRSSI=(RSSI(I)+RSSI(Q))/2
    // Pin(dBm)=2.1*meanRSSI-G
    // Max I and Q value is 15
    // Pin(dBm)=(2.1*((15+15)/2))-G = 31.5-G
    // Measure RSSI Range: 0~31.5 - G
    // G = ~Sensitivity, So have to set Sensitivity to cCenterRSSI - ((2.1*((15+15)/2))/2)

    cCenterRSSI -= ((2.1 * ((15 + 15) / 2)) / 2);

    as3993SetSensitivity(cCenterRSSI);

    cMaxRSSI = AS3993_INVALID_RSSI;

    while(uMeasureTimeMilliseconds)
    {
        uMeasureTimeMilliseconds--;
        
				bIQLevels = as3993MeasureIQLevels();

				cRSSI = (((bIQLevels >> 4) & 0x0F) + (bIQLevels & 0x0F));

				if(cMaxRSSI < cRSSI)
            cMaxRSSI = cRSSI;
    }

    // meanRSSI=(RSSI(I)+RSSI(Q))/2
    // Pin(dBm)=2.1*meanRSSI-G
    // G is a constant depending on the register settings of the RX Filter Settings Register and the RX Mixer and Gain Register.

    cMaxRSSI = ((2.1 * cMaxRSSI) / 2);

    cMaxRSSI += m_cRSSIGValue;

    as3993SetSensitivity(cSaveSensitivity);

    as3993ExitMeasureRSSIMode(bPowerMode);

    return cMaxRSSI;
}



//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  None
  * @retval 
  */
s8 as3993Initialize(void)
{
    const u8 AS3993_INIT_TEST_DATAS[4] = {0x55, 0xAA, 0x00, 0xFF};
    u8 bBuffer[sizeof(AS3993_INIT_TEST_DATAS)];
    u8 i;

		UHF_NCS_SELECT();

#ifdef ALB_DEBUG
    printf("AS3993 Begin Initialize \r\n");
#endif

    g_bIsRFPowerOff = TRUE;

		as3993ResetWithoutSaveRegisters();		// remarked only for stop mode rebooting issue

    // check SPI communication
    as3993WriteBytes(AS3993_REG_MODULATORCONTROL1, AS3993_INIT_TEST_DATAS, sizeof(AS3993_INIT_TEST_DATAS));
    memset(bBuffer, 0x33, sizeof(bBuffer));
    as3993ReadBytes(AS3993_REG_MODULATORCONTROL1, bBuffer, sizeof(bBuffer));

		if(memcmp(bBuffer, AS3993_INIT_TEST_DATAS, sizeof(AS3993_INIT_TEST_DATAS)) != 0)
    {
#ifdef ALB_DEBUG
        printf("%09lu AS3993 Init Fail: data bus interface pins not working \r\n", getSysMilliseconds());
#endif
        return ERR_CHIP_BUS_PINS; // data bus interface pins not working
    }

    // check EN pin + SPI communication
    as3993ResetWithoutSaveRegisters();

    as3993ReadBytes(AS3993_REG_MODULATORCONTROL1, bBuffer, sizeof(bBuffer));

    if(memcmp(bBuffer, AS3993_INIT_TEST_DATAS, sizeof(AS3993_INIT_TEST_DATAS)) == 0)
    {
#ifdef ALB_DEBUG
        printf("%09lu AS3993 Init Fail: enable pin not working \r\n", getSysMilliseconds());
#endif
        return ERR_CHIP_ENABLE_PIN; // enable pin not working
    }

    // check IRQ line
    delay_ms(1);
    WaitForAS3980();

    as3993WriteByte(AS3993_REG_IRQMASK1, AS3993_REG35_IRQ_FIFO_ENABLE);
    // set up 48Byte transmission, but we supply less, therefore a fifo underflow IRQ is produced
    as3993WriteByte(AS3993_REG_TXLENGTHUP, 0x03);
    as3993WriteCmd(AS3993_CMD_TRANSMCRC);

		for(i = 0; i < 6; i++)
        as3993WriteBytes(AS3993_REG_FIFO, bBuffer, sizeof(bBuffer));

    as3993WaitForResponse(RESP_FIFO);

    if(!(as3993GetResponse() & RESP_FIFO))
    {
#ifdef ALB_DEBUG
        printf("%09lu AS3993 Init Fail: AS3993_ERR_FIFO \r\n", getSysMilliseconds());
#endif
        return ERR_CHIP_FIFO;
    }

    as3993ClrResponse();

    as3993ResetWithoutSaveRegisters();

    as3993WriteCmd(AS3993_CMD_HOP_TO_MAIN_FREQUENCY);

    // Have to copy from Default again, becase test AC3993 chip to change the m_tAS3993CtrlRegs values
    memcpy(&m_tAS3993CtrlRegs, &AS3993_CONTROL_REGISTER_DEFAULT, sizeof(m_tAS3993CtrlRegs));

#if RUN_ON_AS3980
    // AS3980 is single ended only
    m_tAS3993CtrlRegs.bMiscellaneous1_single_ended = UHF_INPUT_MIXER_SINGLE_ENDED;
#endif

    as3993RestoreRegisters();

		// Now that the chip is configured with correct ref frequency the PLL should lock
    delay_ms(20);
    as3993WaitingLockPLL();

		bBuffer[0] = as3993ReadByte(AS3993_REG_AGCANDSTATUS);

		if(!(bBuffer[0] & (AS3993_REG2A_OSC_OK | AS3993_REG2A_PLL_OK)))
    {
#ifdef ALB_DEBUG
        printf("%09lu AS3993 Init Fail: Crystal not stable \r\n", getSysMilliseconds());
#endif
        return ERR_CHIP_CRYSTAL; // Crystal not stable
    }
    if (!(bBuffer[0] & AS3993_REG2A_PLL_OK))
    {
#ifdef ALB_DEBUG
        printf("%09lu AS3993 Init Fail: PLL not locked \r\n", getSysMilliseconds());
#endif
       return ERR_CHIP_LOCK_PLL; // PLL not locked
    }

		UHF_NCS_DESELECT();

    if((m_tAS3993CtrlRegs.s.u0.bReg00_Status & AS3993_REG00_STANDBY) != 0)
        m_bReaderPowerMode = UHF_POWER_MODE_STANDBY;
    else if((m_tAS3993CtrlRegs.s.u0.bReg00_Status & AS3993_REG00_TX_ENABLE) != 0)
        m_bReaderPowerMode = UHF_POWER_MODE_NORMAL_RF_ON;
    else if((m_tAS3993CtrlRegs.s.u0.bReg00_Status & AS3993_REG00_RX_ENABLE) != 0)
        m_bReaderPowerMode = UHF_POWER_MODE_NORMAL_REC_ON;
    else
        m_bReaderPowerMode = UHF_POWER_MODE_NORMAL;

#ifdef ALB_DEBUG
    printf("%09lu AS3993 Init OK \r\n", getSysMilliseconds());
#endif

    return ERR_CHIP_OK;
}



