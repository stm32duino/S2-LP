/**
 ******************************************************************************
 * @file    S2LP.h
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Abstract Class of a S2-LP sub-1GHz transceiver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __S2LP_H__
#define __S2LP_H__

/* Includes ------------------------------------------------------------------*/

#include "SPI.h"
#include "S2LP_Commands.h"
#include "S2LP_Config.h"
#include "S2LP_Csma.h"
#include "S2LP_Fifo.h"
#include "S2LP_General.h"
#include "S2LP_Gpio.h"
#include "S2LP_PacketHandler.h"
#include "S2LP_PktBasic.h"
#include "S2LP_PktStack.h"
#include "S2LP_PktWMbus.h"
#include "S2LP_Qi.h"
#include "S2LP_Radio.h"
#include "S2LP_Regs.h"
#include "S2LP_Timer.h"
#include "S2LP_Timer_ex.h"
#include "S2LP_Types.h"
#include <functional>

/* Defines -------------------------------------------------------------------*/
#define FIFO_SIZE 128

/* Typedefs ------------------------------------------------------------------*/
template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
   template <typename... Args>
   static Ret callback(Args... args) {
      return func(args...);
   }
   static std::function<Ret(Params...)> func;
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

typedef void (*S2LPEventHandler)(void);

typedef struct PAInfo_s {
  RangeExtType paRfRangeExtender;
  SGpioInit paSignalCSD;
  SGpioInit paSignalCPS;
  SGpioInit paSignalCTX;
  uint8_t paLevelValue;
} PAInfo_t;

/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of a S2-LP sub-1GHz transceiver
 */
class S2LP
{
  public:
    S2LP(SPIClass *spi, int csn, int sdn, int irqn, uint32_t frequency=868000000, uint32_t xtalFrequency=50000000, PAInfo_t paInfo={.paRfRangeExtender=RANGE_EXT_NONE, .paSignalCSD={S2LP_GPIO_0, S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP, S2LP_GPIO_DIG_OUT_TX_RX_MODE}, .paSignalCPS={S2LP_GPIO_1, S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP, S2LP_GPIO_DIG_OUT_RX_STATE}, .paSignalCTX={S2LP_GPIO_2, S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP, S2LP_GPIO_DIG_OUT_TX_STATE}, .paLevelValue=0x25}, S2LPGpioPin irq_gpio=S2LP_GPIO_3, uint8_t my_addr=0x44, uint8_t multicast_addr=0xEE, uint8_t broadcast_addr=0xFF);
    void begin(void);
    void end(void);
    void attachS2LPReceive(S2LPEventHandler func);
    uint8_t send(uint8_t *payload, uint8_t payload_len, uint8_t dest_addr, bool use_csma_ca = true);
    uint8_t getRecvPayloadLen(void);
    uint8_t read(uint8_t *payload, uint8_t payload_len);

  protected:
    void S2LPSetReadyState(void);
    S2LPCutType S2LPManagementGetCut(void);
    /** S2-LP Irq Callback */
    void S2LPIrqHandler(void);
    void disableS2LPIrq(void);
    void enableS2LPIrq(void);
    S2LPStatus S2LPSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
    S2LPStatus S2LPSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
    S2LPStatus S2LPSpiCommandStrobes(uint8_t cCommandCode);
    S2LPStatus S2LPSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
    S2LPStatus S2LPSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
    void SpiSendRecv(uint8_t *pcHeader, uint8_t *pcBuffer, uint16_t cNbBytes);
    void S2LPManagementRcoCalibration(void);
    void S2LPCmdStrobeCommand(S2LPCmd xCommandCode);
    void S2LPGeneralSetExtRef(ModeExtRef xExtMode);
    ModeExtRef S2LPGeneralGetExtRef(void);
    uint8_t S2LPGeneralGetDevicePN(void);
    uint8_t S2LPGeneralGetVersion(void);
    void S2LPRadioSetExternalSmpsMode(SFunctionalState xNewState);
    void S2LPRefreshStatus(void);
    void S2LPCsmaInit(SCsmaInit* pxSCsmaInit);
    void S2LPCsmaGetInfo(SCsmaInit* pxSCsmaInit);
    void S2LPCsma(SFunctionalState xNewState);
    SFunctionalState S2LPCsmaGetCsma(void);
    void S2LPCsmaPersistentMode(SFunctionalState xNewState);
    SFunctionalState S2LPCsmaGetPersistentMode(void);
    void S2LPCsmaSeedReloadMode(SFunctionalState xNewState);
    SFunctionalState S2LPCsmaGetSeedReloadMode(void);
    void S2LPCsmaSetBuCounterSeed(uint16_t nBuCounterSeed);
    uint16_t S2LPCsmaGetBuCounterSeed(void);
    void S2LPCsmaSetBuPrescaler(uint8_t cBuPrescaler);
    uint8_t S2LPCsmaGetBuPrescaler(void);
    void S2LPCsmaSetCcaPeriod(SCsmaPeriod xMultiplierTbit);
    uint8_t S2LPCsmaGetCcaPeriod(void);
    void S2LPCsmaSetCcaLength(uint8_t xCcaLength);
    uint8_t S2LPCsmaGetCcaLength(void);
    void S2LPCsmaSetMaxNumberBackoff(uint8_t cMaxNb);
    uint8_t S2LPCsmaGetMaxNumberBackoff(void);
    uint8_t S2LPFifoReadNumberBytesRxFifo(void);
    uint8_t S2LPFifoReadNumberBytesTxFifo(void);
    void S2LPFifoSetAlmostFullThresholdRx(uint8_t cThrRxFifo);
    uint8_t S2LPFifoGetAlmostFullThresholdRx(void);
    void S2LPFifoSetAlmostEmptyThresholdRx(uint8_t cThrRxFifo);
    uint8_t S2LPFifoGetAlmostEmptyThresholdRx(void);
    void S2LPFifoSetAlmostFullThresholdTx(uint8_t cThrTxFifo);
    uint8_t S2LPFifoGetAlmostFullThresholdTx(void);
    void S2LPFifoSetAlmostEmptyThresholdTx(uint8_t cThrTxFifo);
    uint8_t S2LPFifoGetAlmostEmptyThresholdTx(void);
    void S2LPFifoMuxRxFifoIrqEnable(SFunctionalState xNewState);
    void S2LPGpioInit(SGpioInit* pxGpioInitStruct);
    void S2LPGpioSetLevel(S2LPGpioPin xGpioX, OutputLevel xLevel);
    OutputLevel S2LPGpioGetLevel(S2LPGpioPin xGpioX);
    void S2LPGpioIrqDeInit(S2LPIrqs* pxIrqInit);
    void S2LPGpioIrqInit(S2LPIrqs* pxIrqInit);
    void S2LPGpioIrqConfig(IrqList xIrq, SFunctionalState xNewState);
    void S2LPGpioIrqGetMask(S2LPIrqs* pxIrqMask);
    void S2LPGpioIrqGetStatus(S2LPIrqs* pxIrqStatus);
    void S2LPGpioIrqClearStatus(void);
    SBool S2LPGpioIrqCheckFlag(IrqList xFlag);
    void S2LPSetPreambleLength(uint16_t cPreambleLength);
    uint16_t S2LPGetPreambleLength(void);
    void S2LPSetSyncLength(uint8_t cSyncLength);
    uint8_t S2LPGetSyncLength(void);
    void S2LPSetSyncWords(uint32_t lSyncWords, uint8_t xSyncLength);
    void S2LPGetSyncWords(uint32_t* lSyncWords, uint8_t* cSyncLength);
    void S2LPPacketHandlerWhitening(SFunctionalState xNewState);
    void S2LPPacketHandlerFec(SFunctionalState xNewState);
    void S2LPPacketHandler3OutOf6(SFunctionalState xNewState);
    void S2LPPacketHandlerManchester(SFunctionalState xNewState);
    uint8_t S2LPGetPacketFormat(void);
    void S2LPPktCommonFilterOnCrc(SFunctionalState xNewState);
    uint8_t S2LPGetReceivedDestinationAddress(void);
    uint8_t S2LPGetReceivedSourceAddress(void);
    uint8_t S2LPGetMyAddress(void);
    uint8_t S2LPGetBroadcastAddress(void);
    uint8_t S2LPGetMulticastAddress(void);
    uint8_t S2LPGetRxSourceMask(void);
    uint8_t S2LPGetRxSourceReferenceAddress(void);
    void S2LPPacketHandlerSetTxMode(DirectTx xNewState);
    void S2LPPacketHandlerSetRxMode(DirectRx xNewState);
    DirectTx S2LPPacketHandlerGetTxMode(void);
    DirectRx S2LPPacketHandlerGetRxMode(void);
    uint8_t S2LPPacketHandlerGetTransmittedSeqNumber(void);
    void S2LPPacketHandlerSetExtendedLenField(SFunctionalState xExtendedLenField);
    void S2LPPacketHandlerSwap4FSKSymbol(SFunctionalState xSwapSymbol);
    void S2LPPacketHandlerSwapFifoEndianess(SFunctionalState xEnableSwap);
    void S2LPPacketHandlerSwapPreamblePattern(SFunctionalState xEnableSwap);
    void S2LPPacketHandlerSetCrcMode(PktCrcMode xPktCrcMode);
    PktCrcMode S2LPPacketHandlerGetCrcMode(void);
    void S2LPPacketHandlerSelectSecondarySync(SFunctionalState xSecondarySync);
    void S2LPPacketHandlerSetAutoPcktFilter(SFunctionalState xNewState);
    void S2LPPacketHandlerSetRxPersistentMode(SFunctionalState xNewState);
    void S2LPPacketHandlerSetSrcAddrFlt(SFunctionalState xNewState);
    void S2LPPacketHandlerSetVariableLength(SFunctionalState xVarLen);
    void S2LPSetDualSyncWords(uint32_t lSyncWords);
    void S2LPGetDualSyncWords(uint32_t* lSyncWords);
    void S2LPSetRxSourceMask(uint8_t address);
    void S2LPSetRxSourceReferenceAddress(uint8_t address);
    void S2LPSetBroadcastAddress(uint8_t address);
    void S2LPSetMulticastAddress(uint8_t address);
    void S2LPSetMyAddress(uint8_t address);
    void S2LPPktBasicInit(PktBasicInit* pxPktBasicInit);
    void S2LPPktBasicGetInfo(PktBasicInit* pxPktBasicInit);
    void S2LPPktBasicAddressesInit(PktBasicAddressesInit* pxPktBasicAddresses);
    void S2LPPktBasicGetAddressesInfo(PktBasicAddressesInit* pxPktBasicAddresses);
    void S2LPPktBasicSetFormat(void);
    void S2LPPktBasicAddressField(SFunctionalState xAddressField);
    SFunctionalState S2LPPktBasicGetAddressField(void);
    void S2LPPktBasicSetPayloadLength(uint16_t nPayloadLength);
    uint16_t S2LPPktBasicGetPayloadLength(void);
    uint16_t S2LPPktBasicGetReceivedPktLength(void);
    void S2LPPktStackInit(PktStackInit* pxPktStackInit);
    void S2LPPktStackGetInfo(PktStackInit* pxPktStackInit);
    void S2LPPktStackAddressesInit(PktStackAddressesInit* pxPktStackAddresses);
    void S2LPPktStackGetAddressesInfo(PktStackAddressesInit* pxPktStackAddresses);
    void S2LPPktStackSetFormat(void);
    void S2LPPktStackSetPayloadLength(uint16_t nPayloadLength);
    uint16_t S2LPPktStackGetPayloadLength(void);
    uint16_t S2LPPktStackGetReceivedPktLength(void);
    void S2LPPktStackAckRequest(SFunctionalState xNewState);
    void S2LPPktStackAutoAck(SFunctionalState xNewState);
    void S2LPPktStackNRetx(uint8_t nRetx);
    void S2LPPktStackPiggybacking(SFunctionalState xNewState);
    void S2LPPktStackSeqNumForReload(uint8_t cReloadValue);
    SFlagStatus S2LPPktStackGetTXAckRequest(void);
    uint8_t S2LPPktStackGetNReTx(void);
    void S2LPPktWMbusInit(PktWMbusInit* pxPktWMbusInit);
    void S2LPPktWMbusGetInfo(PktWMbusInit* pxPktWMbusInit);
    void S2LPPktWMbusSetFormat(void);
    void S2LPPktWMbusSetPostamble(uint8_t cPostamble);
    uint8_t S2LPPktWMbusGetPostamble(void);
    void S2LPPktWMbusSetPostamblePattern(uint8_t cPostamble);
    uint8_t S2LPPktWMbusGetPostamblePattern(void);
    void S2LPPktWMbusSetSubmode(WMbusSubmode xWMbusSubmode);
    WMbusSubmode S2LPPktWMbusGetSubmode(void);
    void S2LPPktWMbusSetPayloadLength(uint16_t nPayloadLength);
    uint16_t S2LPPktWMbusGetPayloadLength(void);
    int32_t S2LPRadioGetRssidBm(void);
    int32_t S2LPRadioGetRssidBmRun(void);
    void S2LPRadioSetRssiThreshdBm(int32_t wRssiThrehsold);
    void S2LPRadioCsBlanking(SFunctionalState xCsBlank);
    void S2LPRadioRssiInit(SRssiInit* xSRssiInit);
    void S2LPRadioGetRssiInfo(SRssiInit* xSRssiInit);
    void S2LPRadioAntennaSwitching(SFunctionalState xAntennaSwitch);
    void S2LPRadioSetPqiCheck(uint8_t cPqiLevel);
    SFlagStatus S2LPQiGetCs(void);
    uint8_t S2LPRadioInit(SRadioInit* pxSRadioInitStruct);
    void S2LPRadioGetInfo(SRadioInit* pxSRadioInitStruct);
    void S2LPRadioSetSynthWord(uint32_t lSynthWord);
    uint32_t S2LPRadioGetSynthWord(void);
    void S2LPRadioSetChannel(uint8_t cChannel);
    uint8_t S2LPRadioGetChannel(void);
    void S2LPRadioSetRefDiv(SFunctionalState xNewState);
    SFunctionalState S2LPRadioGetRefDiv(void);
    void S2LPRadioSetDigDiv(SFunctionalState xNewState);
    SFunctionalState S2LPRadioGetDigDiv(void);
    void S2LPRadioSetChannelSpace(uint32_t lChannelSpace);
    uint32_t S2LPRadioGetChannelSpace(void);
    uint8_t S2LPRadioSetFrequencyBase(uint32_t lFBase);
    uint32_t S2LPRadioGetFrequencyBase(void);
    void S2LPRadioSetDatarate(uint32_t lDatarate);
    uint32_t S2LPRadioGetDatarate(void);
    void S2LPRadioSetFrequencyDev(uint32_t lFDev);
    uint32_t S2LPRadioGetFrequencyDev(void);
    void S2LPRadioSetChannelBW(uint32_t lBandwidth);
    uint32_t S2LPRadioGetChannelBW(void);
    void S2LPRadioSetModulation(ModulationSelect xModulation);
    ModulationSelect S2LPRadioGetModulation(void);
    void S2LPRadioSetXtalFrequency(uint32_t lXtalFrequency);
    uint32_t S2LPRadioGetXtalFrequency(void);
    void S2LPRadioSetIsiEqualizationMode(SIsiEqu xSIsiMode);
    SIsiEqu S2LPRadioGetIsiEqualizationMode(void);
    void S2LPRadioCalibrationVco(SFunctionalState xAmplitudeCalibration, SFunctionalState xFrequencyCalibration);
    void S2LPRadioSetTxCalibVcoAmpWord(uint8_t value);
    void S2LPRadioSetRxCalibVcoAmpWord(uint8_t value);
    void S2LPRadioSetTxCalibVcoFreqWord(uint8_t value);
    void S2LPRadioSetRxCalibVcoFreqWord(uint8_t value);
    void S2LPRadioSetMaxPALevel(SFunctionalState xNewState);
    void S2LPRadioSetPALeveldBm(uint8_t cIndex, int32_t wPowerdBm);
    int32_t S2LPRadioGetPALeveldBm(uint8_t cIndex);
    uint8_t S2LPRadioGetPALevelMaxIndex(void);
    void S2LPRadioSetPALevelMaxIndex(uint8_t cIndex);
    void S2LPRadioSetManualRampingMode(SFunctionalState xNewState);
    void S2LPRadioSetAutoRampingMode(SFunctionalState xNewState);
    void S2LPRadioAfcInit(SAfcInit* xSAfcInit);
    void S2LPRadioGetAfcInfo(SAfcInit* xSAfcInit);
    void S2LPRadioSymClkRecoverInit(SSymClkRecInit* xSSymClkRecInit);
    void S2LPRadioGetSymClkRecoverInfo(SSymClkRecInit* xSSymClkRecInit);
    void S2LPRadioSearchDatarateME(uint32_t lDatarate, uint16_t* pcM, uint8_t* pcE);
    void S2LPRadioSearchFreqDevME(uint32_t lFDev, uint8_t* pcM, uint8_t* pcE);
    void S2LPRadioSearchChannelBwME(uint32_t lBandwidth, uint8_t* pcM, uint8_t* pcE);
    uint32_t S2LPRadioComputeDatarate(uint16_t cM, uint8_t cE);
    uint32_t S2LPRadioComputeFreqDeviation(uint8_t cM, uint8_t cE, uint8_t bs, uint8_t refdiv);
    uint32_t S2LPRadioComputeChannelFilterBw(uint8_t cM, uint8_t cE);
    uint32_t S2LPRadioComputeFrequencyBase(uint32_t lSynthWord, uint8_t bs, uint8_t refdiv);
    uint32_t S2LPRadioComputeSynthWord(uint32_t frequency, uint8_t refdiv);
    uint8_t S2LPRadioComputeChannelSpacingRegValue(uint32_t lChannelSpace);
    uint32_t S2LPRadioComputeChannelSpacing(uint8_t cChSpaceRegVal);
    void S2LPRadioSearchWCP(uint8_t* cp_isel, uint8_t* pfd_split, uint32_t lFc, uint8_t refdiv);
    void S2LPRadioComputeIF(uint32_t nIF, uint8_t* pcAnaIf, uint8_t* pcPcDigIf);
    void S2LPTimerSetRxTimerStopCondition(RxTimeoutStopCondition xStopCondition);
    void S2LPTimerLdcrMode(SFunctionalState xNewState);
    void S2LPTimerLdcrAutoReload(SFunctionalState xNewState);
    SFunctionalState S2LPTimerLdcrGetAutoReload(void);
    void S2LpTimerFastRxTermTimer(SFunctionalState xNewState);
    void S2LpSetTimerFastRxTermTimer(uint8_t fast_rx_word);
    void S2LpSetTimerFastRxTermTimerUs(uint32_t fast_rx_us);
    void S2LPTimerSetRxTimer(uint8_t cCounter , uint8_t cPrescaler);
    void S2LPTimerSetRxTimerUs(uint32_t lDesiredUsec);
    void S2LPTimerSetRxTimerCounter(uint8_t cCounter);
    void S2LPTimerSetRxTimerPrescaler(uint8_t cPrescaler);
    void S2LPTimerGetRxTimerUs(uint32_t* plTimeoutUsec, uint8_t* pcCounter , uint8_t* pcPrescaler);
    void S2LPTimerSetWakeUpTimer(uint8_t cCounter , uint8_t cPrescaler);
    void S2LPTimerSetWakeUpTimerUs(uint32_t lDesiredUsec);
    void S2LPTimerSetWakeUpTimerCounter(uint8_t cCounter);
    void S2LPTimerSetWakeUpTimerPrescaler(uint8_t cPrescaler);
    void S2LPTimerSetWakeUpTimerReloadUs(uint32_t lDesiredUsec);
    void S2LPTimerGetWakeUpTimerUs(uint32_t* plWakeUpUsec, uint8_t* pcCounter, uint8_t* pcPrescaler, uint8_t* pcMulti);
    void S2LPTimerSetWakeUpTimerReload(uint8_t cCounter , uint8_t cPrescaler, uint8_t cMulti);
    void S2LPTimerSetWakeUpTimerReloadCounter(uint8_t cCounter);
    void S2LPTimerSetWakeUpTimerReloadPrescaler(uint8_t cPrescaler);
    void S2LPTimerGetWakeUpTimerReloadUs(uint32_t* plWakeUpReloadUsec, uint8_t* pcCounter, uint8_t* pcPrescaler, uint8_t* pcMulti);
    uint16_t S2LPTimerGetRcoFrequency(void);
    void S2LPTimerCalibrationRco(SFunctionalState xCalibration);
    void S2LPTimerSleepB(SFunctionalState en);
    void S2LPTimerLdcIrqWa(SFunctionalState en);
    void S2LPTimerComputeWakeupTimerValues(uint32_t* plWakeUpUsec , uint8_t cCounter , uint8_t cPrescaler, uint8_t cMulti);
    void S2LPTimerComputeRxTimerValues(uint32_t* plDesiredUsec, uint8_t pcCounter, uint8_t pcPrescaler);
    void S2LPTimerComputeRxTimerRegValues(uint32_t plDesiredUsec , uint8_t* pcCounter , uint8_t* pcPrescaler);
    void S2LPTimerComputeWakeupTimerRegValues(uint32_t plDesiredUsec , uint8_t* pcCounter , uint8_t* pcPrescaler, uint8_t* pcMulti);
    void S2LPTimerSetRxTimerMs(float fDesiredMsec);
    void S2LPTimerGetRxTimer(float* pfTimeoutMsec, uint8_t* pcCounter , uint8_t* pcPrescaler);
    void S2LPTimerSetWakeUpTimerMs(float fDesiredMsec);
    void S2LPTimerSetWakeUpTimerReloadMs(float fDesiredMsec);
    void S2LPTimerGetWakeUpTimer(float* pfWakeUpMsec, uint8_t* pcCounter , uint8_t* pcPrescaler, uint8_t* pcMulti);
    void S2LPTimerGetWakeUpTimerReload(float* pfWakeUpReloadMsec, uint8_t* pcCounter, uint8_t* pcPrescaler, uint8_t* pcMulti);
    void S2LPTimerComputeWakeUpValues(float fDesiredMsec , uint8_t* pcCounter , uint8_t* pcPrescaler);

    SPIClass *dev_spi;
    int csn_pin;
    int sdn_pin;
    int irq_pin;
    uint32_t lFrequencyBase;
    uint32_t s_lXtalFrequency;
    PAInfo_t s_paInfo;
    S2LPGpioPin irq_gpio_selected;
    uint8_t my_address;
    uint8_t multicast_address;
    uint8_t broadcast_address;
    S2LPStatus g_xStatus;
    WMbusSubmode s_cWMbusSubmode;
    S2LPEventHandler current_event_callback;
    S2LPEventHandler irq_handler;
    int nr_of_irq_disabled;
    volatile FlagStatus xTxDoneFlag;
    uint8_t vectcRxBuff[FIFO_SIZE];
    uint8_t vectcTxBuff[FIFO_SIZE];
    uint8_t cRxData;
};

#endif /* __S2LP_H__ */
