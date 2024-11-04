/**
  ******************************************************************************
  * @file    gen2.c
  * @author  Albert
  * @version V1.0.0
  * @date    14-June-2017
  * @brief   gen2 protocol sub-routine
  ******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "gen2.h"
#include "as3993_registers.h"
#include "gen2_config.h"
#include "string.h"

#include "math.h"				// for Q algorithm


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u8 m_bLastGen2Command = 0x00;
static u8 m_bIsWaitTxIrq = 1;
static u8 m_bIsUseAutoACK = GEN2_USE_AUTO_ACK_DEFAULT;
static u8 m_bTimeout = GEN2_TIMEOUT;

volatile static u32 m_ulSendingMSTicks;
u8 g_bIsHoppingFrequency = TRUE;

u8 m_bIsAccessTag = 0;

static u8 m_bFrequencyRand;
static u8 m_bCurrentFrequencyInx;
static u8 m_bCurrentFrequencyRandInx;

static u8 bIsHoppingDirection = HOPPING_OFFSET_DIR_TO_LOW;


u8 m_bRSSIMode = RSSI_MODE_2NDBYTE;    //RSSI_MODE_REALTIME;   // RSSI_MODE_REALTIME is not work;

u16 m_uFoundTagsNum = 0;

s8 m_cRxSensitivity;

u32 g_ulFrequency;

AccessTagInfo_t g_tCurrentTag;
gen2QueryParams_t g_tQueryParams;
QueryInfo_t m_tQueryInfo;

// for testing
u8 bQueryPlusOnes = 0;
u8 bQueryOriginalQ, bQueryPreviousQ;

u16 g_bEndInventoryNum = 0;
u8 g_bEndInventoryRound = FALSE;

u8 bIsCollision, bNoResponse;
double dAdjustQ, dTempQ;

LockTagActions_t g_tLockActions;
UntraceAction_t g_tUntraceAction;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/******************************************************************************/

/**
  * @brief  
  * @param  
  * @retval 
  */
u16 insertBytesToBytes(u8* pbDestBuffer, u16 uDestBitLen, u16 uBitOffset, u16 uLen, const u8* pbSrcBuffer, u16 uSrcBitLen)
{
    u8 bMask;
    u8 bBit;

		if(uBitOffset < uDestBitLen)
    {
        uDestBitLen -= uBitOffset;

				if(uLen > uDestBitLen)
            uLen = uDestBitLen;

        bBit = (uBitOffset & 0x07u);
        uBitOffset = (uBitOffset >> 3);
        bMask = (0xFF >> bBit);

        while(uLen > 0 && uSrcBitLen > 0)
        {
            if(uLen < 8)
            {
                bMask = (0xFFu >> bBit);

								if((bBit + uLen) > 8)
                {
                    pbDestBuffer[uBitOffset] = ((pbDestBuffer[uBitOffset] & (~bMask)) | ((*pbSrcBuffer) >> (uLen - (8 - bBit))));
                    uBitOffset++;
                    uLen -= (8 - bBit);
                    bBit = 0;
                    bMask = (0xFFu << (8 - uLen));
                    pbDestBuffer[uBitOffset] = ((pbDestBuffer[uBitOffset] & (~bMask)) | ((*pbSrcBuffer) << (8 - uLen)));
                    bBit += ((u8)uLen);
                }
                else
                {
                    bMask = ((bMask) & (~(0xFFu >> (bBit + uLen))));
                    pbDestBuffer[uBitOffset] = ((pbDestBuffer[uBitOffset] & (~bMask)) | ((((*pbSrcBuffer) << (8 - bBit - uLen))) & bMask));
                    bBit += uLen;
                }

                if(uSrcBitLen >= uLen)
                    uSrcBitLen -= uLen;
                else
                    uSrcBitLen = 0;

                uLen = 0;
            }
            else
            {
                uLen -= 8;

								if(uSrcBitLen >= 8)
                    uSrcBitLen -= 8;
                else
                    uSrcBitLen = 0;

                if(bBit == 0)
                {
                    pbDestBuffer[uBitOffset] = (*pbSrcBuffer);
                    uBitOffset++;
                }
                else
                {
                    pbDestBuffer[uBitOffset] = ((pbDestBuffer[uBitOffset] & (~bMask)) | ((*pbSrcBuffer) >> bBit));
                    uBitOffset++;
                    pbDestBuffer[uBitOffset] = ((pbDestBuffer[uBitOffset] & bMask) | ((*pbSrcBuffer) << (8 - bBit)));
                }
            }

            pbSrcBuffer++;
        }

        uBitOffset = ((uBitOffset << 3) + bBit);
    }

    return uBitOffset;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u16 insertByteToBytes(u8* pbDestBuffer, u16 uDestBitLen, u16 uOffset, u16 uLen, u8 bValue)
{
    return insertBytesToBytes(pbDestBuffer, uDestBitLen, uOffset, uLen, &bValue, 8);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
u16 instrU32EBVToBytes(u8* pbDestBuffer, u16 uDestBitLen, u16 uOffset, u16 uLen, u32 ulValue)
{
    u8 bValue;
    signed char i;

		i = 28;

		while(i > 0 && (ulValue >> i) == 0)
        i -= 7;

    do
    {
        bValue = ((ulValue >> i) & 0x7Fu);

				if(i > 0)
            bValue |= 0x80u;

        i -= 7;

        uOffset = insertBytesToBytes(pbDestBuffer, uDestBitLen, uOffset, uLen, &bValue, 8);

        if(uLen < 7)
        {
            uLen = 0;
            break;
        }

        uLen -= 7;
    }
    while(i >= 0);

    return uOffset;
}


//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  None
  * @retval None
  */
void uhf2Nak(void)
{
    uhfWriteCmd(AS3993_CMD_NAK);
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 uhfTxRxGen2Bytes(u8 bCommand, const u8* pbTxBuf, u16 uTxBitLen, u8* pbRxBuf, u16* puRxBitLen, u8 bTimeout, u8 bFollowCmd)
{
    s8 cResult;
    u16 uUHFResponse, uTotalByteLen, uRxByteLen;
    u8 bByteLen;
    u8 bBuffer[2];

		cResult = ERR_GEN2_OK;

    uRxByteLen = 0;

		// check parameters' validity
		if(puRxBitLen != NULL)
    {
        if(pbRxBuf != NULL)
            uRxByteLen = (*puRxBitLen);
    }

		// configure RX
    if(uRxByteLen > 0)
    {
				uhfWriteByte(AS3993_REG_RXNORESPONSEWAITTIME, bTimeout);		// ?

				bBuffer[0] = (((uRxByteLen >> 8) & AS3993_REG3A_RX_LENGTH_MSB_MASK) | AS3993_REG3A_AUTO_ERR_LEN); // Set auto_errcode_rx1
        bBuffer[1] = (uRxByteLen & 0xFFu);

				uhfWriteBytes(AS3993_REG_RXLENGTHUP, bBuffer, 2);

				uRxByteLen = ((uRxByteLen + 7) >> 3);

				(*puRxBitLen) = 0;
    }

		// configure TX then do TX
    if(bCommand != 0)
    {
        m_bLastGen2Command = bCommand;

				if(uTxBitLen)
        {
            uTotalByteLen = ((uTxBitLen + 7) >> 3);
            bByteLen = uTotalByteLen;

						if(bByteLen > CHIP_FIFO_BUFFER_SIZE)
                bByteLen = CHIP_FIFO_BUFFER_SIZE;

            bBuffer[0] = (uTxBitLen >> 7);		// (>> 3 and >> 4)
            bBuffer[1] = ((uTxBitLen << 1) & 0xFFu);	// (>> 3 and << 4)

            uhfWriteBytes(AS3993_REG_TXLENGTHUP, bBuffer, 2);

            uhfClrResponse();

            uhfWriteCmdWithBytes(bCommand, AS3993_REG_FIFO, pbTxBuf, bByteLen);

            uTotalByteLen -= bByteLen;
            pbTxBuf += bByteLen;

            while(uTotalByteLen > 0)
            {
                bByteLen = uTotalByteLen;

								if(bByteLen > IRQ_FIFO_BUFFER_SIZE)
                    bByteLen = IRQ_FIFO_BUFFER_SIZE;

                uhfWaitForResponse(RESP_FIFO);
                uhfClrResponseMask(RESP_FIFO);

                uhfWriteBytes(AS3993_REG_FIFO, pbTxBuf, bByteLen);

                uTotalByteLen -= bByteLen;
                pbTxBuf += bByteLen;
            }
        }
        else
        {
            uhfClrResponse();

						uhfWriteCmd(bCommand);
        }
    }

		// waiting for TXIRQ
    if(m_bLastGen2Command && m_bIsWaitTxIrq)
    {
        uhfWaitForResponse(RESP_TXIRQ);
        uhfClrResponseMask(RESP_TXIRQ | RESP_FIFO);
    }

		// do RX
    if(uRxByteLen > 0)
    {
        memset(pbRxBuf, 0x00u, uRxByteLen);

				if(0xFF == bTimeout)
            uhfWaitForResponseTimed((RESP_RXDONE_OR_ERROR | RESP_FIFO), WAIT_FOR_LONG_RESPONSE_COUNT);	// WAIT_FOR_LONG_RESPONSE_COUNT ?
        else
            uhfWaitForResponse(RESP_RXDONE_OR_ERROR | RESP_FIFO);

        while ((uhfGetResponse() & (RESP_FIFO | RESP_RXIRQ)) == RESP_FIFO)
        {
            uhfClrResponseMask(RESP_FIFO | RESP_RXIRQ);

						bByteLen = IRQ_FIFO_BUFFER_SIZE;

						if(bByteLen > uRxByteLen)
            {
                u8 bTemps[IRQ_FIFO_BUFFER_SIZE];

								if(uRxByteLen > 0)
                    ufhReadFifoBytes(pbRxBuf, uRxByteLen);

                ufhReadFifoBytes(bTemps, (bByteLen - uRxByteLen));

                bByteLen = uRxByteLen;
            }
            else
                ufhReadFifoBytes(pbRxBuf, bByteLen);

            if(bByteLen > 0)
            {
                (*puRxBitLen) += bByteLen;
                pbRxBuf += bByteLen;
                uRxByteLen -= bByteLen;
            }
						
            uhfWaitForResponse(RESP_RXDONE_OR_ERROR | RESP_FIFO);
        }

        uhfWaitForResponse(RESP_RXDONE_OR_ERROR);

        uUHFResponse = uhfGetResponse();

        cResult = (uUHFResponse & (~(RESP_FIFO | RESP_AUTOACK)));

#ifdef _SERIAL_DEBUG_
        m_uLastUHFResponse=uUHFResponse;
#endif

        uhfClrResponse();

#if RUN_ON_AS3980
        // on AS3980 header IRQ is actually 2nd-byte IRQ -> ignore if no command which expects header bit was sent
        if(m_bLastGen2Command!=AS3993_CMD_TRANSMCRCEHEAD)
            cResult&=(~RESP_HEADERBIT);
#endif
        if((uUHFResponse & (RESP_RXIRQ | RESP_NORESINTERRUPT | RESP_RXERR)) == RESP_RXIRQ)
        {
            if(bFollowCmd != 0)
            {
								uhfWriteCmd(bFollowCmd);

								m_bLastGen2Command = bFollowCmd;
            }

            cResult = (cResult & (~RESP_RXIRQ));
        }
        else if((uUHFResponse & RESP_AUTOACK) == 0)
        {
            switch(m_bLastGen2Command)
            {
            case AS3993_CMD_QUERY:
            case AS3993_CMD_QUERYREP:
            case AS3993_CMD_QUERYADJUSTUP:
            case AS3993_CMD_QUERYADJUSTNIC:
            case AS3993_CMD_QUERYADJUSTDOWN:
                break;
            }
        }

				// get the number of bytes
        bByteLen = (uhfReadByte(AS3993_REG_FIFOSTATUS) & 0x1F);

        if((uUHFResponse & RESP_HEADERBIT) && (bCommand == AS3993_CMD_TRANSMCRCEHEAD) && (bByteLen == 3))
        {
            u8 bBuffer[3];

						ufhReadFifoBytes(bBuffer, bByteLen);

						cResult = (bBuffer[0] & 0x0F);

						if((bBuffer[0] & 0xF0) == SL900A_ERROR_HEADER)
								cResult |= ERR_SL900A_TAG_ERROR;

						cResult = ERR_TAG_FIRST_ERROR - cResult;
        }
        else
        {
						// receive remaining data
            if(bByteLen > 0)
            {
                if(bByteLen > uRxByteLen)
                {
                    u8 bTemps[IRQ_FIFO_BUFFER_SIZE];

										if(uRxByteLen > 0)
                        ufhReadFifoBytes(pbRxBuf, uRxByteLen);

                    ufhReadFifoBytes(bTemps, (bByteLen - uRxByteLen));

                    bByteLen = uRxByteLen;
                }
                else
                    ufhReadFifoBytes(pbRxBuf, bByteLen);

                (*puRxBitLen) += bByteLen;
                pbRxBuf += bByteLen;
                uRxByteLen -= bByteLen;
            }

						// check other errors
            if(uUHFResponse & RESP_NORESINTERRUPT)
                cResult = ERR_CHIP_NORESP;
            else if(uUHFResponse & RESP_RXCOUNTERROR)
            {
                if(!(uUHFResponse & RESP_RXERR))
                {
                    //as3980 produces irq_err2 (without irq_rxerr) if new epc is read 500ms after last one.
                    WaitForAS3980();        // Wait for tx to be re-enabled.
                }
								
                cResult = ERR_CHIP_RXCOUNT;
            }
            else if(uUHFResponse & RESP_PREAMBLEERROR)
                cResult = ERR_CHIP_PREAMBLE;
            else if(uUHFResponse & RESP_CRCERROR)
                cResult = ERR_CHIP_CRC;

        }

        (*puRxBitLen) = ((*puRxBitLen)*8);
    }
    else
    {
        // we do not want to receive any data
        uhfClrResponse();

				uhfWriteCmd(AS3993_CMD_BLOCKRX);

				bBuffer[0] = 0;
        bBuffer[1] = 0;

				uhfWriteBytes(AS3993_REG_RXLENGTHUP, bBuffer, 2);

				uhfWaitForResponse(RESP_TXIRQ);

				uhfClrResponse();

				cResult = ERR_GEN2_OK;
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
static u8 getRandOrder(u8 bIndex, u8 bRand, u8 bMaxNumbers)
{
    u8 bMask, bPoint;
    u8 i;

		bPoint = 0x80;

		while(bPoint > bMaxNumbers)
        bPoint >>= 1;

    bMask = (bPoint - 1);

    for(i = 0; i < 3; i++)
    {
        if(bIndex <= bMask)
        {
            if(bPoint > 2)
                bIndex = ((bIndex & 0xFC) | ((bIndex & 0x01) << 1)  | ((bIndex & 0x02) >> 1));
            if(bPoint > 4)
                bIndex = ((bIndex & 0xF0) | ((bIndex & 0x03) << 2)  | ((bIndex & 0x0C) >> 2));

            bIndex = (bIndex ^ (bRand & bMask));
        }

        bIndex = ((((u16)bIndex) + bRand + i + 5) % ((u16)bMaxNumbers));
    }

    return bIndex;
}


//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  None
  * @retval None
  */
void gen2Initialize(void)
{
    memset(&m_tQueryInfo, 0, sizeof(m_tQueryInfo));
		m_tQueryInfo.bCurrentQ = g_tRegisters.s2.bGen2QBegin;

    memset(&g_tQueryParams, 0, sizeof(g_tQueryParams));
    g_tQueryParams.bTRcalDivideRatio = 0;
    g_tQueryParams.bMiller = g_tRegisters.s2.bRxDecode;
    g_tQueryParams.bTRext = g_tRegisters.s2.bGen2Trext;
    g_tQueryParams.bSelect = GEN2_QRY_SEL_ALL;
    g_tQueryParams.bSession = g_tRegisters.s2.bGen2Session;
    g_tQueryParams.bTarget = g_tRegisters.s2.bGen2Target & GEN2_QRY_TARGET_MASK;//GEN2_QRY_TARGET_A;
    g_tQueryParams.bQBegin = m_tQueryInfo.bCurrentQ;

    uhfInitialize();

		g_ulFrequency = getFrequency(0);
		uhfSetBaseFrequency(g_ulFrequency);
    uhfSetSensitivity(g_tRegisters.s2.cRFRxSensitivity);
    uhfSetRFTxPower(g_tRegisters.s2.cRFTxPower);

    m_bCurrentFrequencyInx = (u8)UHF_NEVER_HOP;
    m_bCurrentFrequencyRandInx = (u8)UHF_NEVER_HOP;

    uhfSetReaderPowerMode(UHF_POWER_MODE_POWER_DOWN);

		m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;
}

/**
  * @brief  
  * @param  None
  * @retval Error code
  */
static s8 uhfHopFrequencies(void)
{
    volatile s8 cResult;

    cResult = ERR_GEN2_CHANNEL_TIMEOUT;

		if(uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON)
    {
        if(gen2IsHoppingFrequency() == FALSE)
        {
            cResult = ERR_GEN2_OK;
        }
        else if((m_bCurrentFrequencyInx != ((u8)UHF_NEVER_HOP)))
        {
            if(IsTimeout_ms(m_ulSendingMSTicks, g_tRegisters.s2.tFrequencyList.uAllocationTime) == FALSE)
                cResult = ERR_GEN2_OK;
        }
    }

#ifdef ALB_DEBUG
		printf("%09lu UHF Hop freq result1: %+d \r\n", getSysMilliseconds(), cResult);
#endif

    if(cResult == ERR_GEN2_CHANNEL_TIMEOUT)
    {
        uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_REC_ON);

				delay_ms(g_tRegisters.s2.tFrequencyList.uIdleTime);

				if(g_tRegisters.s2.tFrequencyList.bNumberOfFrequency > 0)
        {
            s8 cRSSI_0, cRSSI_1;
            u8 i;
						u32 ulOffsetFrequency;
            u8 count = 0;

						cRSSI_0 = 0;
            cRSSI_1 = 0;

            for(i = 0; i < g_tRegisters.s2.tFrequencyList.bNumberOfFrequency; i++)
            {
                m_bCurrentFrequencyInx++;

								if(m_bCurrentFrequencyInx >= g_tRegisters.s2.tFrequencyList.bNumberOfFrequency)
                    m_bCurrentFrequencyInx = 0;

                m_bCurrentFrequencyRandInx = getRandOrder(m_bCurrentFrequencyInx, m_bFrequencyRand, g_tRegisters.s2.tFrequencyList.bNumberOfFrequency);

                g_ulFrequency = getFrequency(m_bCurrentFrequencyRandInx);

                while(count < 2)
                {
                    if(bIsHoppingDirection == HOPPING_OFFSET_DIR_TO_LOW)
                    {
                        bIsHoppingDirection = HOPPING_OFFSET_DIR_TO_HIGH;

												ulOffsetFrequency = (g_ulFrequency - GEN2_RX_LOW_OFFSET_FREQUENCY_KHZ);

												if(ulOffsetFrequency < AS3993_MIN_TARGET_FREQUENCY)
                            ulOffsetFrequency = (AS3993_MIN_TARGET_FREQUENCY + GEN2_RX_HIGH_OFFSET_FREQUENCY_KHZ);
                    }
                    else
                    {
                        bIsHoppingDirection = HOPPING_OFFSET_DIR_TO_LOW;

												ulOffsetFrequency = (g_ulFrequency + GEN2_RX_HIGH_OFFSET_FREQUENCY_KHZ);

												if(ulOffsetFrequency > AS3993_MAX_TARGET_FREQUENCY)
														ulOffsetFrequency = (AS3993_MAX_TARGET_FREQUENCY - GEN2_RX_LOW_OFFSET_FREQUENCY_KHZ);
                    }

                    as3993SetBaseFrequency(AS3993_REG_PLLMAIN1, ulOffsetFrequency, 100);

                    m_tQueryInfo.cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;

                    uhfSetSensitivity(m_tQueryInfo.cRxSensitivity);

                    if(count == 0)
                    {
												cRSSI_0 = uhfMeasureMaxRSSI(g_tRegisters.s2.tFrequencyList.uListeningTime, g_tRegisters.s2.tFrequencyList.cRSSIThreshold);
                    }
                    else
                    {
												cRSSI_1 = uhfMeasureMaxRSSI(g_tRegisters.s2.tFrequencyList.uListeningTime, g_tRegisters.s2.tFrequencyList.cRSSIThreshold);
                    }

                    count++;							
                }

								if((cRSSI_0 <= g_tRegisters.s2.tFrequencyList.cRSSIThreshold) && (cRSSI_1 <= g_tRegisters.s2.tFrequencyList.cRSSIThreshold))
                {
										as3993SetBaseFrequency(AS3993_REG_PLLMAIN1, g_ulFrequency, 50);

#ifdef _SERIAL_DEBUG_
                    dbg_printf("uhfHopFrequencies: Free %lu, RSSI: %d<=%d\r\n", uhfGetFrequency(m_bCurrentFrequencyRandInx), cRSSI, g_tRegisters.tFrequencyList.cRSSIThreshold);
#endif
                    cResult = ERR_GEN2_OK;

										break; // Found free frequency, now we can return
                }
            }
#ifdef ALB_DEBUG
				printf("%09lu UHF Hop freq result2: %+d with FreqNum=%d,FreqRandInx=%d,OffsetFreq=%d \r\n",
						getSysMilliseconds(), cResult, g_tRegisters.s2.tFrequencyList.bNumberOfFrequency,
						m_bCurrentFrequencyRandInx, ulOffsetFrequency);
#endif
        }
    }

    if(cResult == ERR_GEN2_OK)
    {
        if(uhfGetReaderPowerMode() != UHF_POWER_MODE_NORMAL_RF_ON)
        {
            m_ulSendingMSTicks = getSysTicks();

						uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL_RF_ON);
        }
    }
    else
    {
        uhfSetReaderPowerMode(UHF_POWER_MODE_NORMAL);
    }

    return cResult;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void checkHoppingFrequencies(void)
{
    if(uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON)
        uhfHopFrequencies();
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void gen2ResetInventoryRound(void)
{
    if((m_tQueryInfo.uCollisions == 0) && (m_tQueryInfo.bCurrentQ > GEN2_QUERY_MIN_Q))
        m_tQueryInfo.bCurrentQ--;

    m_tQueryInfo.uQueryCount = 0;
    m_tQueryInfo.bQNoChangeCount = 0;

    m_tQueryInfo.uCollisions = 0;
		m_tQueryInfo.uAckCount = 0;
		m_tQueryInfo.uNorespCount = 0;

		bIsCollision = FALSE;
		bNoResponse = FALSE;
		dAdjustQ = 0;
		dTempQ = 0;

		m_uFoundTagsNum = 0;

    m_bFrequencyRand = ((u8)(getSysTicks() & 0xFF));
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void gen2Configure(gen2QueryParams_t* ptQueryParams)
{
    // depending on link frequency setting adjust
    // registers 01, 02, 03, 04, 05, 06, 07, 08 and 09
    u8 bRegisters[9];

    ptQueryParams->bTRcalDivideRatio = 1;

		if(ptQueryParams->bSession > GEN2_IINV_S3)
        ptQueryParams->bSession = GEN2_IINV_S0; // limit SL and invalid settings

    if((ptQueryParams->bMiller == GEN2_COD_FM0) || (ptQueryParams->bMiller == GEN2_COD_MILLER2))
        ptQueryParams->bTRext = TREXT_ON;

    switch(g_tRegisters.s2.bGen2LinkFrequency)
    {
    case GEN2_LF_640:  /* 640kHz */

				bRegisters[0] = 0x20; /* AS3993_REG_TXOPTIONS            */
        bRegisters[1] = 0xF0; /* AS3993_REG_RXOPTIONS            */
        bRegisters[2] = 0x01; /* AS3993_REG_TRCALHIGH            */
        bRegisters[3] = 0x4D; /* AS3993_REG_TRCALLOW             */
        bRegisters[4] = 0x03; /* AS3993_REG_AUTOACKTIMER         */
        bRegisters[5] = 0x02; /* AS3993_REG_RXNORESPONSEWAITTIME */
        bRegisters[6] = 0x01; /* AS3993_REG_RXWAITTIME           */

				if(ptQueryParams->bMiller > GEN2_COD_MILLER2)
            bRegisters[7] = 0x04; /* AS3993_REG_RXFILTER         */
        else
            bRegisters[7] = 0x07; /* AS3993_REG_RXFILTER         */
        break;

    case GEN2_LF_320: /* 320kHz */

				bRegisters[0] = 0x20;
        bRegisters[1] = 0xC0;

				if(g_tRegisters.s2.bGen2Tari == TARI_6_25)    /* if Tari = 6.25us */
        {
            /* TRcal = 25us */
            ptQueryParams->bTRcalDivideRatio = 0;
            bRegisters[2] = 0x00;
            bRegisters[3] = 0xFA;
        }
        else
        {
            /* TRcal = 66.7us */
            bRegisters[2] = 0x02;
            bRegisters[3] = 0x9B;
        }

        bRegisters[4] = 0x04;
        bRegisters[5] = 0x03;
        bRegisters[6] = 0x02;

        if(ptQueryParams->bMiller > GEN2_COD_MILLER2)
            bRegisters[7] = 0x24; /* AS3993_REG_RXFILTER         */
        else
            bRegisters[7] = 0x27; /* AS3993_REG_RXFILTER         */
        break;

    case GEN2_LF_256: /* 256kHz */

				bRegisters[0] = 0x10;			// for testing (00:1.50*Tari, 10:1.66*Tari, 20:1.83*Tari, 30:2.00*Tari)
        bRegisters[1] = 0x90;

				if(g_tRegisters.s2.bGen2Tari == TARI_6_25)    /* Tari = 6.25us */
        {
            /* TRcal = 31.3us */
            ptQueryParams->bTRcalDivideRatio = 0;

						bRegisters[2] = 0x01;
            bRegisters[3] = 0x39;
        }
        else                                /* Tari = 25us or 12.5us */
        {
            /* TRcal = 83.3us */
            bRegisters[2] = 0x03;
            bRegisters[3] = 0x41;
        }

        bRegisters[4] = 0x00;			// for testing (T2: 3*Tpri = 11.1us ~ 20*Tpri = 78.1us)
				bRegisters[5] = 0x0F;			// for testing (T1: 70.93*0.9-2 = 61.8us ~ 70.93*1.1+2 = 80.0us)
				bRegisters[6] = 0x08;			// for testing (T1: 70.93*0.9-2 = 61.8us ~ 70.93*1.1+2 = 80.0us)

        if(ptQueryParams->bMiller > GEN2_COD_MILLER2)
            bRegisters[7] = 0x34; /* AS3993_REG_RXFILTER         */
        else
            bRegisters[7] = 0x37; /* AS3993_REG_RXFILTER         */
        break;

    case GEN2_LF_213: /* 213.3kHz */

				bRegisters[0] = 0x20;
        bRegisters[1] = 0x80;

			if(g_tRegisters.s2.bGen2Tari == TARI_6_25)    /* Tari = 6.25us */
        {
            /* TRcal = 37.5us */
            ptQueryParams->bTRcalDivideRatio = 0;

						bRegisters[2] = 0x01;
            bRegisters[3] = 0x77;
        }
        else
        {
            /* TRcal = 100us */
            bRegisters[2] = 0x03;
            bRegisters[3] = 0xE8;
        }

        bRegisters[4] = 0x06;
        bRegisters[5] = 0x05;
        bRegisters[6] = 0x05;

        if(ptQueryParams->bMiller > GEN2_COD_MILLER2)
            bRegisters[7] = 0x34; /* AS3993_REG_RXFILTER         */
        else
            bRegisters[7] = 0x37; /* AS3993_REG_RXFILTER         */
        break;

    case GEN2_LF_160: /* 160kHz */

				bRegisters[0] = 0x20;
        bRegisters[1] = 0x60;

				if(g_tRegisters.s2.bGen2Tari == TARI_12_5)    /* Tari = 12.5us */
        {
            /* TRcal = 50us */
            ptQueryParams->bTRcalDivideRatio = 0;

						bRegisters[2] = 0x01;
            bRegisters[3] = 0xF4;
        }
        else
        {
            /* TRcal = 1333.3us */
            bRegisters[2] = 0x05;
            bRegisters[3] = 0x35;
        }

        bRegisters[4] = 0x08;//0x0A;
        bRegisters[5] = 0x05;
        bRegisters[6] = 0x08;

        if(ptQueryParams->bMiller > GEN2_COD_FM0)
            bRegisters[7] = 0x3F; /* AS3993_REG_RXFILTER         */
        else
            bRegisters[7] = 0xBF; /* AS3993_REG_RXFILTER         */
        break;

    case GEN2_LF_40: /* 40kHz */

				bRegisters[0] = 0x30; /* AS3993_REG_TXOPTIONS            */
        bRegisters[1] = 0x00; /* AS3993_REG_RXOPTIONS            */
        bRegisters[2] = 0x07; /* AS3993_REG_TRCALHIGH            */
        bRegisters[3] = 0xD0; /* AS3993_REG_TRCALLOW             */
        bRegisters[4] = 0x20;//0x3F; /* AS3993_REG_AUTOACKTIMER         */
        bRegisters[5] = 0x0C; /* AS3993_REG_RXNORESPONSEWAITTIME */
        bRegisters[6] = 0x24; /* AS3993_REG_RXWAITTIME           */
        bRegisters[7] = 0xFF; /* AS3993_REG_RXFILTER         */

				ptQueryParams->bTRcalDivideRatio = 0;
        break;

    default:
        return; /* use preset settings */
    }

    bRegisters[0] |= g_tRegisters.s2.bGen2Tari;
    bRegisters[1] = (bRegisters[1] & ~0x0F)
                    | (ptQueryParams->bMiller)
                    | (ptQueryParams->bTRext << 3);

    m_bTimeout = bRegisters[5];

    /* Modify only the gen2 relevant settings */
    uhfWriteBytes(AS3993_REG_TXOPTIONS, bRegisters, 8);

    uhfSetGen2Session(ptQueryParams->bSession);
    uhfSetGen2Protocol();
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2Select(gen2SelectParams_t* ptSelectParams)
{
    u32 ulBitCount;
    u16 uLen;
    u8 bBuffer[sizeof(gen2SelectParams_t)+1];
		s8 cResult = ERR_GEN2_PARAM;

		if(ptSelectParams != NULL)
    {
        memset(bBuffer, 0x00, sizeof(bBuffer));

				ulBitCount = 0;
        ulBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer)*8), ulBitCount, 4, EPC_SELECT);
        ulBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer)*8), ulBitCount, 3, ptSelectParams->bTarget);
        ulBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer)*8), ulBitCount, 3, ptSelectParams->bAction);
        ulBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer)*8), ulBitCount, 2, ptSelectParams->bMemBank);
        ulBitCount = instrU32EBVToBytes(bBuffer, (sizeof(bBuffer)*8), ulBitCount, 32, ptSelectParams->ulPointer);
        ulBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer)*8), ulBitCount, 8, ptSelectParams->bLength);
        
				if(ptSelectParams->bLength > 0)
            ulBitCount = insertBytesToBytes(bBuffer, (sizeof(bBuffer)*8), ulBitCount, ptSelectParams->bLength, ptSelectParams->bMasks, (sizeof(ptSelectParams->bMasks)*8));
        
				ulBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer)*8), ulBitCount, 1, ptSelectParams->bTruncation);
        
				uLen = 1;
        cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRC,
                                 bBuffer,
                                 ulBitCount,
                                 bBuffer,
                                 &uLen,
                                 1,
                                 0x00u);

#ifdef _SERIAL_DEBUG_
        printf("%09lu Select CMD(%lu): target=%d, action=%d, mem_bank=%d, point=%lu, len=%d: ", getSysMilliseconds(), ulBitCount, ptSelectParams->bTarget, ptSelectParams->bAction, ptSelectParams->bMemBank, ptSelectParams->ulPointer, ptSelectParams->bLength);
        if(ptSelectParams->bLength>0)
        {
            dbg_printHex(ptSelectParams->bMasks, (ptSelectParams->bLength/8));
            printf(", ");
        }
        dbg_printHex(bBuffer, ((ulBitCount+7)>>3));
        dbg_printCrLf();
        dbg_waitForPrint();
#endif

        delay_ms(1);

        uhfReadBytes(AS3993_REG_IRQSTATUS1, bBuffer, 2);    // ensure that IRQ bits are reset
        uhfClrResponse();
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2QueryCmd(u8 bCommand, gen2QueryParams_t* ptQueryParams, AccessTagInfo_t* ptTag, u8 bIsTagAccess)
{
    u8 bBuffer[(CRC_LENGTH + PC_LENGTH + 128)];
    u16 uRN16;
    u16 uLen;
		s8 cResult = ERR_GEN2_PARAM;

		m_bIsWaitTxIrq = (!m_bIsUseAutoACK);

#ifdef _SERIAL_DEBUG_
    if(bCommand==AS3993_CMD_QUERY)
    {
        printf("%09lu Query: DR=%d, miller=%d, trext=%d, sel=%d, session=%d, target=%c, q=%d: ", getSysMilliseconds(), ptQueryParams->bTRcalDivideRatio, ptQueryParams->bMiller, ptQueryParams->bTRext, ptQueryParams->bSelect, ptQueryParams->bSession, ('A'+ptQueryParams->bTarget), ptQueryParams->bQBegin);
        dbg_printHex((u8*)ptQueryParams, (sizeof(ptQueryParams)));
        dbg_printCrLf();
        dbg_waitForPrint();
    }
#endif

    if(ptTag != NULL)
    {
        memset(ptTag, 0x00, sizeof(AccessTagInfo_t));

				if(m_bIsUseAutoACK)
        {
            uLen = (sizeof(bBuffer) * 8);

						cResult = uhfTxRxGen2Bytes(bCommand,
																			(u8*)ptQueryParams,
																			(ptQueryParams != NULL ? (sizeof(gen2QueryParams_t) * 8) : 0x00),
																			bBuffer,
																			&uLen,
																			m_bTimeout,
																			0);
        }
        else
        {
            uLen = (sizeof(uRN16) * 8);

						cResult = uhfTxRxGen2Bytes(bCommand,
																			(u8*)ptQueryParams,
																			(ptQueryParams != NULL ? (sizeof(gen2QueryParams_t) * 8) : 0x00),
																			(u8*)&uRN16,
																			&uLen,
																			m_bTimeout,
																			AS3993_CMD_ACK);

            if(cResult == ERR_GEN2_OK)
            {
                // Get ACK's reply
                uLen = (sizeof(bBuffer) * 8);

								cResult = uhfTxRxGen2Bytes(0x00,
																					NULL,
																					0x00,
																					bBuffer,
																					&uLen,
																					m_bTimeout,
																					(bIsTagAccess ? AS3993_CMD_REQRN : 0x00));

            }

				}

        if(cResult == ERR_GEN2_OK)
        {
            cResult = ERR_CHIP_RXCOUNT;

						uLen = ((uLen + 7) >> 3);

						if((uLen > 0) && (uLen <= sizeof(ptTag->tTag.bFullEPCs)))
            {
                cResult = ERR_GEN2_PC_LENGTH_ERROR;

								memcpy(ptTag->tTag.bFullEPCs, bBuffer, uLen);

#if RUN_ON_AS3980
                if((CRC_LENGTH + PC_LENGTH + ((bBuffer[0] & 0xF8u) >> 2)) == uLen)
#else
                if((PC_LENGTH + ((bBuffer[0] & 0xF8u) >> 2)) == uLen)
#endif //#if RUN_ON_AS3980
                {
                    cResult = ERR_GEN2_OK;

										ptTag->tTag.bEPCLen = uLen;

										if(bIsTagAccess && (uLen > 0))
                    {
                        uLen = (sizeof(ptTag->bHandles) * 8);

												cResult = uhfTxRxGen2Bytes(0x00,
																									NULL,
																									0x00,
																									ptTag->bHandles,
																									&uLen,
																									m_bTimeout,
																									0);

												if(cResult == ERR_GEN2_OK)
                            ptTag->tTag.bFlag |= TAG_FLAG_NEW_TAG;
                    }
                }
            }
        }
    }

    m_bIsWaitTxIrq = 1;

    if(cResult == ERR_GEN2_OK)
        ptTag->tTag.cRSSI = uhfGetRssi(m_bRSSIMode, &ptTag->tTag.bIQValue);

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2ReqRN16(const AccessTagInfo_t* ptTag, RN16_t* puRN16)
{
    u8 bBuffer[3];
    u16 uLen;
    s8 cResult = ERR_GEN2_PARAM;

		bBuffer[0] = EPC_REQRN;
    bBuffer[1] = ptTag->bHandles[0];
    bBuffer[2] = ptTag->bHandles[1];

		uLen = 32; // RN16+CRC16

    if((ptTag != NULL) && (puRN16 != NULL))
    {
        puRN16->uRN16 = 0;

				cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRC,
																	bBuffer,
																	(8 * 3),
																	puRN16->bRN16s,
																	&uLen,
																	m_bTimeout,
																	0x00u);

#ifdef ALB_DEBUG
		printf("%09lu ReqRN16TxRx result: %+d\r\n", getSysMilliseconds(), cResult);
#endif

				if(cResult != ERR_GEN2_OK)
            cResult = ERR_GEN2_REQRN;
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2DirectWriteTag(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8* pbDatas)
{
    u8 bBuffer[8];
    u16 uBitCount;
    RN16_t uRN16;
    u16 uLen;
    s8 cResult = ERR_GEN2_PARAM;

		if((ptTag != NULL) && (pbDatas != NULL))
				cResult = gen2ReqRN16(ptTag, &uRN16);		// need again ? not using AS3993 direct command (9Fh) ?

#ifdef ALB_DEBUG
		printf("%09lu Gen2ReqRN16 result: %+d\r\n", getSysMilliseconds(), cResult);
#endif

		if(cResult == ERR_GEN2_OK)
    {
        memset(bBuffer, 0x00, sizeof(bBuffer));

				uBitCount = 0;
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, EPC_WRITE);
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, bMemBank);
        uBitCount = instrU32EBVToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 32, ulWordAddress);
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, (pbDatas[0] ^ uRN16.bRN16s[0]));
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, (pbDatas[1] ^ uRN16.bRN16s[1]));
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[0]);
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[1]);

				uLen = (32 + 1);    // 1 bit Header + 16 bits Handle + 16 bits CRC16

				cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRCEHEAD,
																	bBuffer,
																	uBitCount,
																	bBuffer,
																	&uLen,
																	GEN2_LONG_ACCESS_TIMEOUT,
																	0x00u);
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2DirectReadTag(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8 bWordCount, u8* pbBuffer, u16* puLength)
{
    volatile u16 uLen;
    u8 bBuffer[8];
    u16 uBitCount;
    u16 uRxBits;
    s8 cResult = ERR_GEN2_PARAM;

		if((ptTag != NULL) && (pbBuffer != NULL) && (puLength != NULL))
    {
        memset(bBuffer, 0x00, sizeof(bBuffer));

				uBitCount = 0;
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, EPC_READ);
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, bMemBank);
        uBitCount = instrU32EBVToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 32, ulWordAddress);
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, bWordCount);
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[0]);
        uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[1]);

        uLen = (*puLength);

				memset(pbBuffer, 0x00, uLen);

				uLen = ((uLen << 3) + 16 + 1); // 2bytes crc are not included in rx buffer

				(*puLength) = 0;

				uRxBits = ((bWordCount * 2 + 4) * 8 + 1); // + 2 bytes rn16 + 2bytes crc + 1 header bit

				if(uLen > uRxBits)
            uLen = uRxBits;

        cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRCEHEAD,
																	bBuffer,
																	uBitCount,
																	pbBuffer,
																	(u16*)&uLen,
																	GEN2_LONG_ACCESS_TIMEOUT,
																	0x00u);

        uLen = (uLen >> 3);

        if(cResult == ERR_GEN2_OK)
        {
            if(((uLen & 1) == 0) && (uLen > 2))
            {
                uLen -= 2;    // Without RN16

								if(((u16*)&pbBuffer[uLen])[0] == ptTag->uHandle)
                    (*puLength) = uLen;
                else
                    cResult = ERR_CHIP_CRC;
            }
            else
                cResult = ERR_CHIP_RXCOUNT;
        }

        (*puLength) = uLen;
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2DirectBlockErase(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8 bWordCount)
{
    u8 bBuffer[8];
    u16 uBitCount;
    u16 uLen;
    s8 cResult = ERR_GEN2_PARAM;

		if(ptTag != NULL)
		{
				memset(bBuffer, 0x00, sizeof(bBuffer));

				uBitCount = 0;
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, EPC_BLOCKERASE);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, bMemBank);
				uBitCount = instrU32EBVToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 32, ulWordAddress);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, bWordCount);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[0]);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[1]);

				uLen = (32 + 1);    // 1 bit Header+16 bits Handle+ 16 bits CRC16

				cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRCEHEAD,
																	bBuffer,
																	uBitCount,
																	bBuffer,
																	&uLen,
																	GEN2_LONG_ACCESS_TIMEOUT,
																	0x00u);
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2DirectAccessTag(AccessTagInfo_t* ptTag)
{
    u8 bBuffer[5];
    RN16_t uRN16;
    u16 uLen;
    u8 i;
    s8 cResult = ERR_GEN2_PARAM;

		if(ptTag != NULL)
    {
        for(i = 0; i <= 2; i += 2)
        {
            cResult = gen2ReqRN16(ptTag, &uRN16);

						if(cResult != ERR_GEN2_OK)
                break;

            bBuffer[0] = EPC_ACCESS;
            bBuffer[1] = ptTag->bPasswords[i+0] ^ uRN16.bRN16s[0];
            bBuffer[2] = ptTag->bPasswords[i+1] ^ uRN16.bRN16s[1];
            bBuffer[3] = ptTag->bHandles[0];
            bBuffer[4] = ptTag->bHandles[1];

            uLen = 32;		// RN16 + CRC16

            cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRC,
																			bBuffer,
																			(8 * sizeof(bBuffer)),
																			uRN16.bRN16s,
																			&uLen,
																			m_bTimeout,
																			0x00u);

						if((cResult != ERR_GEN2_OK) || (ptTag->uHandle != uRN16.uRN16))
            {
								cResult = ERR_GEN2_ACCESS;
                break;
            }
        }
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2DirectKillTag(AccessTagInfo_t* ptTag, u8 bRFU)
{
    u8 bBuffer[6];
    u16 uBitCount;
    RN16_t uRN16;
    u16 uLen;
    u8 i;
    s8 cResult = ERR_GEN2_PARAM;

		if(ptTag != NULL)
    {
        for(i = 0; i <= 2; i += 2)
        {
            cResult = gen2ReqRN16(ptTag, &uRN16);

						if(cResult != ERR_GEN2_OK)
                break;

						memset(bBuffer, 0x00, sizeof(bBuffer));

            uBitCount = 0;
            uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, EPC_KILL);
            uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, (ptTag->bPasswords[i+0] ^ uRN16.bRN16s[0]));
            uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, (ptTag->bPasswords[i+1] ^ uRN16.bRN16s[1]));
            uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 3, bRFU);
            uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[0]);
            uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[1]);

						if(i == 0)
						{
								uLen = 32; 		// RN16 + CRC16

								cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRC,
																					bBuffer,
																					uBitCount,
																					uRN16.bRN16s,
																					&uLen,
																					m_bTimeout,
																					0x00u);
						}
						else
						{
								uLen = (32 + 1);    // 1 bit Header + 16 bits Handle + 16 bits CRC16

								cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRC,
																					bBuffer,
																					uBitCount,
																					uRN16.bRN16s,
																					&uLen,
																					GEN2_LONG_ACCESS_TIMEOUT,
																					0x00u);
						}
        }
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2DirectLockTag(AccessTagInfo_t* ptTag, LockTagActions_t* ptLockActions)
{
    u8 bBuffer[8];
    u16 uBitCount;
    u16 uLen;
    s8 cResult = ERR_GEN2_PARAM;

		if((ptTag != NULL) && (ptLockActions != NULL))
    {
				memset(bBuffer, 0x00, sizeof(bBuffer));

				uBitCount = 0;
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, EPC_LOCK);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bKillMask);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bAccessMask);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bEPCMask);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bTIDMask);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bUserMask);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bKillAction);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bAccessAction);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bEPCAction);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bTIDAction);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptLockActions->bUserAction);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[0]);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[1]);

				uLen = (32 + 1);    // 1 bit Header + 16 bits Handle + 16 bits CRC16

				cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRCEHEAD,
																	bBuffer,
																	uBitCount,
																	bBuffer,
																	&uLen,
																	GEN2_LONG_ACCESS_TIMEOUT,
																	0x00u);
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
static s8 gen2DirectUntraceTag(AccessTagInfo_t* ptTag, UntraceAction_t* ptUntraceAction)
{
    u8 bBuffer[6];
    u16 uBitCount;
    u16 uLen;
    s8 cResult = ERR_GEN2_PARAM;

		if((ptTag != NULL) && (ptUntraceAction != NULL))
		{
				memset(bBuffer, 0x00, sizeof(bBuffer));

				uBitCount = 0;
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, EPC_UNTRACE_HIGH);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, EPC_UNTRACE_LOW);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptUntraceAction->bRFU);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 1, ptUntraceAction->bUBit);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 6, ptUntraceAction->bEpcAction);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptUntraceAction->bTidHide);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 1, ptUntraceAction->bUserHide);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 2, ptUntraceAction->bRange);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[0]);
				uBitCount = insertByteToBytes(bBuffer, (sizeof(bBuffer) * 8), uBitCount, 8, ptTag->bHandles[1]);

				uLen = (32 + 1);    // 1 bit Header + 16 bits Handle + 16 bits CRC16

				cResult = uhfTxRxGen2Bytes(AS3993_CMD_TRANSMCRCEHEAD,
																	bBuffer,
																	uBitCount,
																	bBuffer,
																	&uLen,
																	GEN2_LONG_ACCESS_TIMEOUT,
																	0x00u);
		}

    return cResult;
}


//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2SelectTagTID(AccessTagInfo_t* ptTag)
{
    gen2SelectParams_t tSelectTIDParams;
    u8 bTry;
    s8 cResult = ERR_GEN2_PARAM;

		if(ptTag != NULL)
        cResult = uhfHopFrequencies();

    if(cResult == ERR_GEN2_OK)
    {
        uhfWriteByte(AS3993_REG_STATUSPAGE, m_bRSSIMode);

				if(m_bRSSIMode == RSSI_MODE_PEAK)     //if we use peak rssi mode, we have to send anti collision commands
            uhfWriteCmd(AS3993_CMD_ANTI_COLL_ON);

        memset(&tSelectTIDParams, 0x00, sizeof(tSelectTIDParams));

        tSelectTIDParams.bTarget = GEN2_IINV_S0;
        tSelectTIDParams.bAction = GEN2_ACT_MATCH_UNSEL_B_NM_SEL_A;
        tSelectTIDParams.bMemBank = MEM_TID;
        tSelectTIDParams.ulPointer = 0;

				if(ptTag->bTIDLen > 0)
        {
            tSelectTIDParams.bLength = (ptTag->bTIDLen * 8);

						if(ptTag->bTIDLen > sizeof(tSelectTIDParams.bMasks))
                memcpy(tSelectTIDParams.bMasks, ptTag->bTIDs, sizeof(tSelectTIDParams.bMasks));
            else
                memcpy(tSelectTIDParams.bMasks, ptTag->bTIDs, ptTag->bTIDLen);
        }
        tSelectTIDParams.bTruncation = 0;

        for(bTry=0; bTry<3; bTry++)			// ?
        {
						if(ptTag->bTIDLen > 0)
                cResult = gen2Select(&tSelectTIDParams);

						if(cResult == ERR_GEN2_OK)
                break;

            delay_ms(1);
        }

    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2SelectTagBank(u8 bSelBank, u32 ulByteAddress, u8 bMaskByte, u8* pbBankMask)
{
		gen2SelectParams_t tSelectBankParams;

    u8 bTry;
    s8 cResult = ERR_GEN2_PARAM;

		if(pbBankMask != NULL)
        cResult = uhfHopFrequencies();

    if(cResult == ERR_GEN2_OK)
    {
        uhfWriteByte(AS3993_REG_STATUSPAGE, m_bRSSIMode);

				if(m_bRSSIMode == RSSI_MODE_PEAK)     //if we use peak rssi mode, we have to send anti collision commands
            uhfWriteCmd(AS3993_CMD_ANTI_COLL_ON);

        memset(&tSelectBankParams, 0x00, sizeof(tSelectBankParams));

        tSelectBankParams.bTarget = GEN2_IINV_S0;
        tSelectBankParams.bAction = GEN2_ACT_MATCH_UNSEL_B_NM_SEL_A;
        tSelectBankParams.bMemBank = bSelBank;
        tSelectBankParams.ulPointer = ulByteAddress * 8;

        if(bMaskByte > 0)
        {
            tSelectBankParams.bLength = bMaskByte * 8;

						if(bMaskByte > sizeof(tSelectBankParams.bMasks))
                memcpy(tSelectBankParams.bMasks, pbBankMask, sizeof(tSelectBankParams.bMasks));
            else
                memcpy(tSelectBankParams.bMasks, pbBankMask, bMaskByte);
        }
        tSelectBankParams.bTruncation = 0;

        for(bTry=0; bTry<3; bTry++)
        {
						if(bMaskByte > 0)
                cResult = gen2Select(&tSelectBankParams);

						if(cResult == ERR_GEN2_OK)
                break;

            delay_ms(1);
        }
    }

    return cResult;
}


/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2Query(gen2QueryParams_t* ptQueryParams, AccessTagInfo_t* ptTag, u8 bIsTagAccess)
{
    ptTag->tTag.bEPCLen = 0;

		uhfClearHaveRFPowerOff();

		gen2ResetInventoryRound();

		return gen2QueryCmd(AS3993_CMD_QUERY,
                        ptQueryParams,
                        ptTag,
                        bIsTagAccess);
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2QueryRep(AccessTagInfo_t* ptTag, u8 bIsTagAccess)
{
    ptTag->tTag.bEPCLen = 0;

		return gen2QueryCmd(AS3993_CMD_QUERYREP,
                        NULL,
                        ptTag,
                        bIsTagAccess);
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2QueryAdjust(s8 bAdjustQ, AccessTagInfo_t* ptTag, u8 bIsTagAccess)
{
    u8 bCmd;

    ptTag->tTag.bEPCLen = 0;

		bCmd = AS3993_CMD_QUERYADJUSTNIC;

		if(bAdjustQ > 0)
        bCmd = AS3993_CMD_QUERYADJUSTUP;
    else if(bAdjustQ < 0)
        bCmd = AS3993_CMD_QUERYADJUSTDOWN;

    return gen2QueryCmd(bCmd,
                        NULL,
                        ptTag,
                        bIsTagAccess);
}

/**
  * @brief  
  * @param  None
  * @retval 
  */
BOOL gen2IsInventoryFinish(void)
{
    BOOL bResult;

		bResult = TRUE;

		if(g_tRegisters.s2.bGen2QBegin != 0)
    {
        if((uhfGetReaderPowerMode() == UHF_POWER_MODE_NORMAL_RF_ON) && (uhfIsHaveRFPowerOff() == FALSE))
        {
            if(m_tQueryInfo.uQueryCount == 0)
            {
                if((m_tQueryInfo.uCollisions > 0) && (m_tQueryInfo.bQNoChangeCount < GEN2_QUERY_TO_BREAK_BY_SAME_Q_COUNT))
                {
#if !RUN_ON_AS3980
                    if((m_tQueryInfo.bCurrentQ >= GEN2_QUERY_MIN_Q) && (m_tQueryInfo.bCurrentQ <= GEN2_QUERY_MAX_Q))
                        bResult = FALSE;
#else
                    bResult = FALSE;
#endif
                }
            }
            else
                bResult = FALSE;
        }
        else
            gen2ResetInventoryRound();
    }

#ifdef ALB_DEBUG
		printf("%09lu Is inventory finish: %d with QBegin=%d,count=%d,collision=%d \r\n",
						getSysMilliseconds(), bResult, g_tRegisters.s2.bGen2QBegin, m_tQueryInfo.uQueryCount, m_tQueryInfo.uCollisions);
#endif

    return bResult;
}

/**
  * @brief  
  * @param  Error code
  * @retval None
  */
static void gen2InventoryResultCheck(s8 cResult)
{
		switch(cResult)
    {
		case ERR_GEN2_OK:
				m_tQueryInfo.uAckCount++;
				m_uFoundTagsNum++;		// count the number of found tags
				break;
		case ERR_CHIP_NORESP:
				m_tQueryInfo.uNorespCount++;
				bNoResponse = TRUE;
				break;
    case ERR_CHIP_PREAMBLE:
    case ERR_CHIP_RXCOUNT:
		case ERR_CHIP_CRC:
				m_tQueryInfo.uCollisions++;
				bIsCollision = TRUE;
				break;
    }

		m_tQueryInfo.uQueryCount--;

    if(m_tQueryInfo.uQueryCount == 0)
    {
        if(gen2IsInventoryFinish())
        {
            if(m_bRSSIMode == RSSI_MODE_PEAK)             //if we use peak rssi mode, we have to send anti collision commands
                uhfWriteCmd(AS3993_CMD_ANTI_COLL_OFF);

						if(m_tQueryInfo.uNorespCount == (1u << m_tQueryInfo.bCurrentQ))		// additional condition for inventory end
						{
								if(++g_bEndInventoryNum >= GEN2_INVENTORY_END_TIME)
										g_bEndInventoryRound = TRUE;
						}

        }
    }
}

/**
  * @brief  
  * @param  None
  * @retval Error code
  */
static s8 gen2InventoryBegin(void)
{
#ifdef INVENTORY_USE_SELECT
    gen2SelectParams_t tSelParams;
#endif

		u8 bBuffer[2];
    s8 cResult;

#ifdef ALB_DEBUG
    printf("%09lu gen2InventoryBegin\r\n", getSysMilliseconds());
#endif

    gen2ResetInventoryRound();

		cResult = uhfHopFrequencies();

		if(cResult == ERR_GEN2_OK)
    {

#if !RUN_ON_AS3980
        if(m_tQueryInfo.bCurrentQ < GEN2_QUERY_MIN_Q)
            m_tQueryInfo.bCurrentQ = GEN2_QUERY_MIN_Q;
        else if(m_tQueryInfo.bCurrentQ > GEN2_QUERY_MAX_Q)
            m_tQueryInfo.bCurrentQ = GEN2_QUERY_MAX_Q;
#else
        m_tQueryInfo.bCurrentQ = GEN2_QUERY_MIN_Q;
#endif

        gen2Configure(&g_tQueryParams);

        uhfWriteByte(AS3993_REG_STATUSPAGE, m_bRSSIMode);

        if(m_bRSSIMode == RSSI_MODE_PEAK)				//if we use peak rssi mode, we have to send anti collision commands
            uhfWriteCmd(AS3993_CMD_ANTI_COLL_ON);

        if(m_bIsUseAutoACK)
            uhfSetAutoAckMode(UHF_AUTO_MODE_ACK_REQ_RN);
        else    //Disable AutoACK mode
            uhfSetAutoAckMode(UHF_AUTO_MODE_NONE);

#ifdef INVENTORY_USE_SELECT
        memset(&tSelParams, 0x00, sizeof(tSelParams));

        tSelParams.bAction = GEN2_ACT_MATCH_SEL_A_NM_UNSEL_B;
        tSelParams.bTarget = g_tRegisters.s2.bGen2Session;
        tSelParams.bMemBank = MEM_EPC;
        tSelParams.ulPointer = 0;
        tSelParams.bLength = 0;
        tSelParams.bTruncation = 0;

				gen2Select(&tSelParams);		// Match=>A, No-Match=>B ?

				tSelParams.bAction = GEN2_ACT_NM_SEL_A;

				gen2Select(&tSelParams);		// Match=>do nothing, No-Match=>A ?
#endif

        uhfReadBytes(AS3993_REG_IRQSTATUS1, bBuffer, 2);    // ensure that IRQ bits are reset

				uhfClrResponse();

				memset(&g_tCurrentTag, 0x00, sizeof(g_tCurrentTag));

				cResult = gen2Query(&g_tQueryParams, &g_tCurrentTag, m_bIsAccessTag);

				m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);

#ifdef ALB_DEBUG
				printf("%09lu Begin Query: %+d with CurrentQ=%d,Slots=%d \r\n",
						getSysMilliseconds(), cResult, m_tQueryInfo.bCurrentQ, m_tQueryInfo.uQueryCount);
#endif

				gen2InventoryResultCheck(cResult);

    }

    return cResult;
}

/**
  * @brief  
  * @param  None
  * @retval Error code
  */
static s8 gen2InventoryNext(void)
{
    s8 chQueryAdjust;
    s8 cResult = ERR_GEN2_PARAM;

#ifdef ALB_DEBUG
    printf("%09lu gen2InventoryNext\r\n", getSysMilliseconds());
#endif

		if((m_tQueryInfo.uQueryCount > 0) || (m_tQueryInfo.uCollisions > 0))
    {
#define NEW_ADJUST_ALOHA

#ifdef NEW_ADJUST_TIME

				if(m_tQueryInfo.uQueryCount > 0)
        {
						if(m_tQueryInfo.uCollisions >= ((1u << m_tQueryInfo.bCurrentQ) / 4))
						{
								bQueryPlusOnes++;
								if(bQueryPlusOnes >= 3)
								{
										if(m_cRxSensitivity < (GEN2_LEAST_SENSITIVITY))
												m_cRxSensitivity += GEN2_STEP_SENSITIVITY;
										else
												m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;

										uhfSetSensitivity(m_cRxSensitivity);
								}

								chQueryAdjust = (+1);

								cResult = gen2QueryAdjust(chQueryAdjust, &g_tCurrentTag, m_bIsAccessTag);

								m_tQueryInfo.bCurrentQ += chQueryAdjust;
								m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);
								m_tQueryInfo.bQNoChangeCount = 0;
								m_tQueryInfo.uCollisions = 0;
								m_tQueryInfo.uAckCount = 0;
								m_tQueryInfo.uNorespCount = 0;
						}
						else
								cResult = gen2QueryRep(&g_tCurrentTag, m_bIsAccessTag);

				}
				else
				{
						bQueryPlusOnes = 0;

						chQueryAdjust = 0;

						if(m_tQueryInfo.uCollisions >= ((1u << m_tQueryInfo.bCurrentQ) / 4))
								chQueryAdjust = (+1);
						else if(m_tQueryInfo.uCollisions < ((1u << m_tQueryInfo.bCurrentQ) / 8))
								chQueryAdjust = (-1);
						else if(m_tQueryInfo.bQNoChangeCount < GEN2_QUERY_TO_BREAK_BY_SAME_Q_COUNT)
								m_tQueryInfo.bQNoChangeCount++;

						cResult = gen2QueryAdjust(chQueryAdjust, &g_tCurrentTag, m_bIsAccessTag);

						m_tQueryInfo.bCurrentQ += chQueryAdjust;
						m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);
						m_tQueryInfo.bQNoChangeCount = 0;
						m_tQueryInfo.uCollisions = 0;
						m_tQueryInfo.uAckCount = 0;
						m_tQueryInfo.uNorespCount = 0;
				}

        gen2InventoryResultCheck(cResult);

#elif defined(NEW_ADJUST_QZERO)

				if(m_tQueryInfo.uQueryCount == 0)
				{

						if(m_tQueryInfo.bCurrentQ > 0)
						{

								// for sensitivity change
								if(m_tQueryInfo.uCollisions > ((1u << m_tQueryInfo.bCurrentQ) / 2))
								{
										if(m_cRxSensitivity < (GEN2_LEAST_SENSITIVITY))
												m_cRxSensitivity += GEN2_STEP_SENSITIVITY;
										else
												m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;

										uhfSetSensitivity(m_cRxSensitivity);
								}

								chQueryAdjust = 0;

								if(m_tQueryInfo.uCollisions >= ((1u << m_tQueryInfo.bCurrentQ) / 4))
										chQueryAdjust = (+1);
								else if(m_tQueryInfo.uCollisions < ((1u << m_tQueryInfo.bCurrentQ) / 8))
										chQueryAdjust = (-1);
								else if(m_tQueryInfo.bQNoChangeCount < GEN2_QUERY_TO_BREAK_BY_SAME_Q_COUNT)
										m_tQueryInfo.bQNoChangeCount++;

								m_tQueryInfo.bCurrentQ += chQueryAdjust;
								m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);
								m_tQueryInfo.bQNoChangeCount = 0;		// ?
								m_tQueryInfo.uCollisions = 0;
								m_tQueryInfo.uAckCount = 0;
								m_tQueryInfo.uNorespCount = 0;
						}
						else		// m_tQueryInfo.bCurrentQ == 0
						{
								chQueryAdjust = 0;

								if(m_tQueryInfo.uCollisions >= ((1u << m_tQueryInfo.bCurrentQ) / 4))
										chQueryAdjust = (+1);
								else if(m_tQueryInfo.uCollisions < ((1u << m_tQueryInfo.bCurrentQ) / 8))
										chQueryAdjust = (-1);
								else if(m_tQueryInfo.bQNoChangeCount < GEN2_QUERY_TO_BREAK_BY_SAME_Q_COUNT)
										m_tQueryInfo.bQNoChangeCount++;

								cResult = gen2QueryAdjust(chQueryAdjust, &g_tCurrentTag, m_bIsAccessTag);

								m_tQueryInfo.bCurrentQ += chQueryAdjust;
								m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);
								m_tQueryInfo.bQNoChangeCount = 0;
								m_tQueryInfo.uCollisions = 0;
								m_tQueryInfo.uAckCount = 0;
								m_tQueryInfo.uNorespCount = 0;

								gen2InventoryResultCheck(cResult);
						}

				}
				else
				{
            cResult = gen2QueryRep(&g_tCurrentTag, m_bIsAccessTag);

						gen2InventoryResultCheck(cResult);
				}

#elif defined(NEW_ADJUST_ALOHA)

				if(m_tQueryInfo.uQueryCount > 0)
        {
						if(bIsCollision == TRUE)
						{
								dAdjustQ += GEN2_ALOHA_QFP_INC;

								if(dAdjustQ > 15)
										dAdjustQ = 15;

								dTempQ = floor(dAdjustQ);
						}
						else if(bNoResponse == TRUE)
						{
								dAdjustQ -= GEN2_ALOHA_QFP_DEC;

								if(dAdjustQ < 0)
										dAdjustQ = 0;

								dTempQ = ceil(dAdjustQ);
						}
						else
						{
								dTempQ = (double)m_tQueryInfo.bCurrentQ;
						}


						if((u8)dTempQ > m_tQueryInfo.bCurrentQ)
						{
								chQueryAdjust = (+1);
								cResult = gen2QueryAdjust(chQueryAdjust, &g_tCurrentTag, m_bIsAccessTag);

								m_tQueryInfo.bCurrentQ += chQueryAdjust;
								m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);

								m_tQueryInfo.uCollisions = 0;
								m_tQueryInfo.uNorespCount = 0;
								m_tQueryInfo.uAckCount = 0;

								dAdjustQ = 0;
								dTempQ = 0;
						}
						else if((u8)dTempQ < m_tQueryInfo.bCurrentQ)
						{
								chQueryAdjust = (-1);
								cResult = gen2QueryAdjust(chQueryAdjust, &g_tCurrentTag, m_bIsAccessTag);

								m_tQueryInfo.bCurrentQ += chQueryAdjust;
								m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);

								m_tQueryInfo.uCollisions = 0;
								m_tQueryInfo.uNorespCount = 0;
								m_tQueryInfo.uAckCount = 0;

								dAdjustQ = 0;
								dTempQ = 0;
						}
						else
						{
								cResult = gen2QueryRep(&g_tCurrentTag, m_bIsAccessTag);
						}

						bIsCollision = FALSE;
						bNoResponse = FALSE;

				}
				else
				{
						chQueryAdjust = 0;
						cResult = gen2QueryAdjust(chQueryAdjust, &g_tCurrentTag, m_bIsAccessTag);

						m_tQueryInfo.bCurrentQ += chQueryAdjust;
						m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);

						m_tQueryInfo.uCollisions = 0;
						m_tQueryInfo.uNorespCount = 0;
						m_tQueryInfo.uAckCount = 0;

						bIsCollision = FALSE;
						bNoResponse = FALSE;

						dAdjustQ = 0;
						dTempQ = 0;
				}

        gen2InventoryResultCheck(cResult);

#else

				if(m_tQueryInfo.uQueryCount == 0)
        {
						// for sensitivity change
						if(m_tQueryInfo.uCollisions > ((1u << m_tQueryInfo.bCurrentQ) / 2))
						{
								if(m_cRxSensitivity < (GEN2_LEAST_SENSITIVITY))
										m_cRxSensitivity += GEN2_STEP_SENSITIVITY;
								else
										m_cRxSensitivity = g_tRegisters.s2.cRFRxSensitivity;

								uhfSetSensitivity(m_cRxSensitivity);
						}

#ifdef ALB_DEBUG
						printf("%09lu SetSensitivity: %+d\r\n", getSysMilliseconds(), m_cRxSensitivity);
#endif

						{
								chQueryAdjust = 0;

								if(m_tQueryInfo.uCollisions >= ((1u << m_tQueryInfo.bCurrentQ) / 4))//(1u << (m_tQueryInfo.bCurrentQ - 2)))
										chQueryAdjust = (+1);
								else if(m_tQueryInfo.uCollisions < ((1u << m_tQueryInfo.bCurrentQ) / 8))//(1u << (m_tQueryInfo.bCurrentQ - 3)))
										chQueryAdjust = (-1);
								else if(m_tQueryInfo.bQNoChangeCount < GEN2_QUERY_TO_BREAK_BY_SAME_Q_COUNT)
										m_tQueryInfo.bQNoChangeCount++;

								cResult = gen2QueryAdjust(chQueryAdjust, &g_tCurrentTag, m_bIsAccessTag);

								m_tQueryInfo.bCurrentQ += chQueryAdjust;
								m_tQueryInfo.uQueryCount = (1u << m_tQueryInfo.bCurrentQ);
								m_tQueryInfo.bQNoChangeCount = 0;
								m_tQueryInfo.uCollisions = 0;
								m_tQueryInfo.uAckCount = 0;
								m_tQueryInfo.uNorespCount = 0;

#ifdef ALB_DEBUG
								printf("%09lu QueryAdjust: %+d with CurrentQ=%d,Slots=%d \r\n",
										getSysMilliseconds(), cResult, m_tQueryInfo.bCurrentQ, m_tQueryInfo.uQueryCount);
#endif
						}
				}
        else
            cResult = gen2QueryRep(&g_tCurrentTag, m_bIsAccessTag);

        gen2InventoryResultCheck(cResult);

#endif
    }

#ifdef ALB_DEBUG
		if(m_tQueryInfo.uQueryCount == 0)
				printf("%09lu Found tags: %d\r\n", getSysMilliseconds(), m_uFoundTagsNum);
#endif

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2WriteTag(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8 bWordCount, u8* pbBuffer, u16* puLength)
{
    u16 uLen;
    u8 i, bTry;
    s8 cResult = ERR_GEN2_PARAM;

		if((ptTag != NULL) && (pbBuffer != NULL) && (puLength != NULL))
    {
        uLen = (*puLength);
        (*puLength) = 0;

				if((uLen > 0) && ((uLen & 1) == 0))
        {
            uLen >>= 1;   // To Word Length

						for(i = 0; ((i < bWordCount) && (uLen > 0)); i++)
            {
                for(bTry = 0; bTry < GEN2_WRITE_TAG_RETRY_TIME; bTry++)
                {
                    cResult = gen2DirectWriteTag(ptTag, bMemBank, (ulWordAddress + i), &pbBuffer[(i << 1)]);

#ifdef ALB_DEBUG
										printf("%09lu Gen2DirectWriteTag result: %+d\r\n", getSysMilliseconds(), cResult);
#endif

										if(cResult == ERR_GEN2_OK)
                    {
                        (*puLength) += 2;
                        uLen--;

												break;
                    }

                    // Potentially writing is still ongoing, avoid metastable eeprom and stuff
                    delay_ms(20);
                }

                if(cResult != ERR_GEN2_OK)
                    break;
            }
        }
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2ReadTag(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8 bWordCount, u8* pbBuffer, u16* puLength)
{
    u8 bBuffer[READ_TAG_BUFFER_FULL_SIZE];
    u16 uLen, uBufferLen, uMaxReadLen;
    s8 cResult = ERR_GEN2_PARAM;
    u8 uReadLen;
    u16 uBytesToRead = 0;
    u8 bTry;
    u32 i;

		if((ptTag != NULL) && (pbBuffer != NULL) && (puLength != NULL))
    {
        uBufferLen = (*puLength);

				memset(&pbBuffer[0], 0x00, uBufferLen);

        uMaxReadLen = ((uBufferLen + 1) >> 1);		// get word length

				// Define uMaxReadLen
#if READ_TAG_BUFFER_SIZE>80
        if(bMemBank == MEM_USER)
        {
            if(uMaxReadLen > READ_TAG_BUFFER_WORD_SIZE)
                uMaxReadLen = READ_TAG_BUFFER_WORD_SIZE;
        }
        else
        {
#if READ_TAG_BUFFER_SIZE>160
            if(uMaxReadLen > (READ_TAG_BUFFER_WORD_SIZE/4))
                uMaxReadLen = (READ_TAG_BUFFER_WORD_SIZE/4);
#else
            if(uMaxReadLen > (READ_TAG_BUFFER_WORD_SIZE/2))
                uMaxReadLen = (READ_TAG_BUFFER_WORD_SIZE/2);
#endif
        }
#else
        if(uMaxReadLen > READ_TAG_BUFFER_WORD_SIZE)
            uMaxReadLen = READ_TAG_BUFFER_WORD_SIZE;
#endif

        i = 0;
        bTry = 0;

        while((i < uBufferLen) && (bWordCount > 0))  	// uBufferLen=0x30, bWordCount=0x18
        {
            uLen = READ_TAG_BUFFER_FULL_SIZE;
            uReadLen = READ_TAG_BUFFER_WORD_SIZE; 		// Without Handle

						if(uReadLen > bWordCount)
                uReadLen = bWordCount;
            if(uReadLen > uMaxReadLen)
                uReadLen = uMaxReadLen;
            if(uReadLen == 0)
                break;

            cResult = gen2DirectReadTag(ptTag, bMemBank, ulWordAddress, uReadLen, &bBuffer[0], &uLen);

            if(cResult != ERR_GEN2_OK)
            {
                bTry++;

								if((bTry >= GEN2_READ_TAG_RETRY_TIME) || (cResult == ERR_TAG_MEMORY_OVERRUN || cResult == ERR_TAG_MEMORY_LOCK))
                {
                    bTry = 0;

										if(uMaxReadLen <= 1)
                        break;

                    uReadLen = (uLen >> 1);

                    if((uMaxReadLen > uReadLen) && (uReadLen > 0))
                        uMaxReadLen = uReadLen;
                    else
                        uMaxReadLen = (uMaxReadLen >> 1);
                }
            }
            else
            {
                if((i + uLen) > uBufferLen)
                    uLen = (uBufferLen - i);

                memcpy(&pbBuffer[i], &bBuffer[0], uLen);

                uBytesToRead += uLen;
                i += uLen;

                uLen = (uLen >> 1);
                ulWordAddress += uLen;

                if(uLen < bWordCount)
                    bWordCount -= uLen;
                else
                    bWordCount = 0;
            }
        }
    }

		if(puLength != NULL)
        *puLength = uBytesToRead;

    if((cResult != ERR_CHIP_NORESP) && (uBytesToRead > 0))
    {
        if((cResult != ERR_TAG_MEMORY_OVERRUN) && (cResult != ERR_TAG_MEMORY_LOCK))		// ?
            cResult = ERR_GEN2_OK;
    }

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2AccessTag(AccessTagInfo_t* ptTag)
{
    u8 bTry;
    s8 cResult = ERR_GEN2_PARAM;

		if(ptTag != NULL)
		{
				for(bTry = 0; bTry < GEN2_ACCESS_TAG_RETRY_TIME; bTry++)
				{
						cResult = gen2DirectAccessTag(ptTag);

						if(cResult == ERR_GEN2_OK)
								break;
				}
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2KillTag(AccessTagInfo_t* ptTag, u8 bRFU)
{
    u8 bTry;
    s8 cResult = ERR_GEN2_PARAM;

		if(ptTag != NULL)
		{
				for(bTry = 0; bTry < GEN2_KILL_TAG_RETRY_TIME; bTry++)
				{
						cResult = gen2DirectKillTag(ptTag, bRFU);

						if(cResult == ERR_GEN2_OK)
								break;
				}
		}

		return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2LockTag(AccessTagInfo_t* ptTag, LockTagActions_t* ptLockActions)
{
    u8 bTry;
    s8 cResult = ERR_GEN2_PARAM;

		if((ptTag != NULL) && (ptLockActions != NULL))
		{
				for(bTry = 0; bTry < GEN2_LOCK_TAG_RETRY_TIME; bTry++)
				{
						cResult = gen2DirectLockTag(ptTag, ptLockActions);

						if(cResult == ERR_GEN2_OK)
								break;
				}
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2UntraceTag(AccessTagInfo_t* ptTag, UntraceAction_t* ptUntraceAction)
{
    u8 bTry;
    s8 cResult = ERR_GEN2_PARAM;

		if((ptTag != NULL) && (ptUntraceAction != NULL))
		{
				for(bTry = 0; bTry < GEN2_UNTRACE_TAG_RETRY_TIME; bTry++)
				{
						cResult = gen2DirectUntraceTag(ptTag, ptUntraceAction);

						if(cResult == ERR_GEN2_OK)
								break;
				}
		}

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2BlockErase(AccessTagInfo_t* ptTag, u8 bMemBank, u32 ulWordAddress, u8 bWordCount)
{
    u8 bTry;
    s8 cResult = ERR_GEN2_PARAM;

		if(ptTag != NULL)
		{
				for(bTry = 0; bTry < GEN2_ERASE_TAG_RETRY_TIME; bTry++)		// ?
				{
						cResult = gen2DirectBlockErase(ptTag, bMemBank, ulWordAddress, bWordCount);

						if(cResult == ERR_GEN2_OK)
								break;
				}
		}

    return cResult;
}



//----------------------------------------------------------------------------//

/**
  * @brief  
  * @param  None
  * @retval Error code
  */
s8 gen2Inventory(void)
{
#ifdef ALB_DEBUG
    u8 DbgCount;
#endif

		s8 cResult;

#ifdef _SERIAL_DEBUG_
    showRxTxEvents();
    dbg_waitForPrint();
    g_bDbgOverflowToShowLog=0;
    g_bDbgShowLog=0;
#endif

		if(gen2IsInventoryFinish())
    {
#ifdef ALB_DEBUG
        DbgCount = 0;
#endif

				cResult = gen2InventoryBegin();
    }
    else
    {
#ifdef ALB_DEBUG
        DbgCount++;
#endif

				cResult = gen2InventoryNext();
    }

#ifdef _SERIAL_DEBUG_
    showRxTxEvents();
    dbg_waitForPrint();
    g_bDbgOverflowToShowLog=0;
    g_bDbgShowLog=0;
#endif

#ifdef ALB_DEBUG
    u8 i, bValue;
		if(cResult == ERR_GEN2_OK)
    {
				printf("%09lu Inventory #%d: OK EPC: ", getSysMilliseconds(), DbgCount);
				for(i = 0; i< g_tCurrentTag.tTag.bEPCLen; i++)
				{
						bValue=('0' | (g_tCurrentTag.tTag.bFullEPCs[i]>>4));
						if(bValue>'9')
								bValue+=7; // 7= ('A'-'9'-1);
						printf("%c", bValue);

						bValue=('0' | (g_tCurrentTag.tTag.bFullEPCs[i] & 0x0F));
						if(bValue>'9')
								bValue+=7; // 7= ('A'-'9'-1);
						printf("%c", bValue);
				}
				printf("\r\n");
    }
    else
    {
        printf("%09lu Inventory #%d: Fail(%+d) \r\n", getSysMilliseconds(), DbgCount, cResult);
    }
#endif

    return cResult;
}

/**
  * @brief  
  * @param  
  * @retval Error code
  */
s8 gen2ReadTagTID(AccessTagInfo_t* ptTag)
{
    s8 cResult = ERR_GEN2_PARAM;
    u16 uLen;

		if(ptTag != NULL)
    {
        uLen = sizeof(ptTag->bTIDs);

				cResult = gen2ReadTag(ptTag, MEM_TID, 0, (sizeof(ptTag->bTIDs) >> 1), &ptTag->bTIDs[0], &uLen);

				ptTag->bTIDLen = uLen;

				if(cResult == ERR_TAG_MEMORY_OVERRUN)		// ?
            cResult = ERR_GEN2_OK;
    }

    return cResult;
}






