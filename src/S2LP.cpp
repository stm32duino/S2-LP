/**
 ******************************************************************************
 * @file    S2LP.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Implementation of a S2-LP sub-1GHz transceiver.
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


/* Includes ------------------------------------------------------------------*/

#include "S2LP.h"

/* Defines -------------------------------------------------------------------*/
#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define S2LP_CMD_SIZE   2

#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/

#define LINEAR_FIFO_ADDRESS		0xFF  /*!< Linear FIFO address*/

#define XTAL_FREQUENCY          50000000U

#define PREAMBLE_LENGTH (64*4)
#define DATARATE 38400
#define MIN_PERIOD_WAKEUP ((8000*((PREAMBLE_LENGTH/4)-2))/DATARATE)

/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param spi object of the instance of the spi peripheral
 * @param csn the spi chip select pin
 * @param sdn the shutdown pin
 * @param irqn the pin to receive IRQ event
 * @param frequency the base carrier frequency in Hz
 * @param xtalFrequency the crystal oscillator frequency
 * @param paInfo information about external power amplifier
 * @param irq_gpio the S2-LP gpio used to receive the IRQ events
 * @param my_addr address of the device
 * @param multicast_addr multicast address
 * @param broadcast_addr broadcast address
 */
S2LP::S2LP(SPIClass *spi, int csn, int sdn, int irqn, uint32_t frequency, uint32_t xtalFrequency, PAInfo_t paInfo, S2LPGpioPin irq_gpio, uint8_t my_addr, uint8_t multicast_addr, uint8_t broadcast_addr) : dev_spi(spi), csn_pin(csn), sdn_pin(sdn), irq_pin(irqn), lFrequencyBase(frequency), s_lXtalFrequency(xtalFrequency), s_paInfo(paInfo), irq_gpio_selected(irq_gpio), my_address(my_addr), multicast_address(multicast_addr), broadcast_address(broadcast_addr)
{
  Callback<void()>::func = std::bind(&S2LP::S2LPIrqHandler, this);
  irq_handler = static_cast<S2LPEventHandler>(Callback<void()>::callback);
  memset((void *)&g_xStatus, 0, sizeof(S2LPStatus));
  s_cWMbusSubmode = WMBUS_SUBMODE_NOT_CONFIGURED;
  current_event_callback = NULL;
  nr_of_irq_disabled = 0;
  xTxDoneFlag = RESET;
  memset((void *)vectcRxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  memset((void *)vectcTxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  cRxData = 0;
  is_waiting_for_read = false;
  is_tx_done_before_read = false;
}

/**
* @brief  Initialize the S2-LP library.
* @param  None.
* @retval None.
*/
void S2LP::begin(void)
{
  pinMode(csn_pin, OUTPUT);
  digitalWrite(csn_pin, HIGH);

  pinMode(sdn_pin, OUTPUT);

  /* Shutdown the S2-LP */
  digitalWrite(sdn_pin, HIGH);
  delay(1);
  digitalWrite(sdn_pin, LOW);
  delay(1);

  /* S2-LP soft reset */
  S2LPCmdStrobeSres();

  SGpioInit xGpioIRQ={
    irq_gpio_selected,
    S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
    S2LP_GPIO_DIG_OUT_IRQ  
  };

  S2LPGpioInit(&xGpioIRQ);

  SRadioInit xRadioInit = {
    lFrequencyBase, /* base carrier frequency */
    MOD_2FSK,       /* modulation type */
    DATARATE,       /* data rate */
    20000,          /* frequency deviation */
    100000          /* bandwidth */
  };

  S2LPRadioInit(&xRadioInit);

  S2LPRadioSetMaxPALevel(S_DISABLE);

  if(s_paInfo.paRfRangeExtender == RANGE_EXT_NONE)
  {
    S2LPRadioSetPALeveldBm(7, 12);
  } else
  {
    /* in case we are using a board with external PA, the S2LPRadioSetPALeveldBm will be not functioning because
    the output power is affected by the amplification of this external component.
    Set the raw register. */
    /* For example, paLevelValue=0x25 will give about 19dBm on a STEVAL FKI-915V1 */
    S2LPSpiWriteRegisters(PA_POWER8_ADDR, 1, &s_paInfo.paLevelValue);

    if(s_paInfo.paRfRangeExtender == RANGE_EXT_SKYWORKS_SE2435L)
    {
      S2LPGpioInit(&s_paInfo.paSignalCSD);
      S2LPGpioInit(&s_paInfo.paSignalCPS);
      S2LPGpioInit(&s_paInfo.paSignalCTX);
    }
  }

  S2LPRadioSetPALevelMaxIndex(7);

  PktBasicInit xBasicInit={
    PREAMBLE_LENGTH,    /* Preamble length */
    32,                 /* Sync length */
    0x88888888,         /* Sync word */
    S_ENABLE,           /* Variable length */
    S_DISABLE,          /* Extended length field */
    PKT_CRC_MODE_8BITS, /* CRC mode */
    S_ENABLE,           /* Enable address */
    S_DISABLE,          /* Enable FEC */
    S_ENABLE            /* Enable Whitening */
  };

  S2LPPktBasicInit(&xBasicInit);

  PktBasicAddressesInit xAddressInit={
    S_ENABLE,          /* Filtering my address */
    my_address,        /* My address */
    S_ENABLE,          /* Filtering multicast address */
    multicast_address, /* Multicast address */
    S_ENABLE,          /* Filtering broadcast address */
    broadcast_address  /* broadcast address */
  };

  S2LPPktBasicAddressesInit(&xAddressInit);

  SCsmaInit xCsmaInit={
    S_DISABLE,          /* Persistent mode enable/disable */
    CSMA_PERIOD_64TBIT, /* CS Period */
    3,                  /* CS Timeout */
    5,                  /* Max number of backoffs */
    0xFA21,             /* BU counter seed */
    32                  /* CU prescaler */
  };

  S2LPCsmaInit(&xCsmaInit);
  S2LPPacketHandlerSetRxPersistentMode(S_ENABLE);

  SRssiInit xSRssiInit = {
    .cRssiFlt = 14,
    .xRssiMode = RSSI_STATIC_MODE,
    .cRssiThreshdBm = -60,
  };
  S2LPRadioRssiInit(&xSRssiInit);

  S2LPManagementRcoCalibration();

  /* Enable SQI */
  S2LPRadioSetPqiCheck(0x00);
  S2LPRadioSetPqiCheck(S_ENABLE);

  /* S2LP IRQs enable */
  S2LPGpioIrqDeInit(NULL);
  S2LPGpioIrqConfig(RX_DATA_READY,S_ENABLE);
  S2LPGpioIrqConfig(TX_DATA_SENT , S_ENABLE);

  /* IRQ registers blanking */
  S2LPGpioIrqClearStatus();

  /* clear FIFO if needed */
  S2LPCmdStrobeFlushRxFifo();

  /* set the LDC mode wkup */
  S2LPTimerSetWakeUpTimerUs(1000*MIN_PERIOD_WAKEUP);

  /* set the rx timeout */
  S2LPTimerSetRxTimerUs(30000);

  /* use SLEEP_A mode (default) */
  S2LPTimerSleepB(S_DISABLE);

  /* enable LDC mode, FAST RX TERM and start Rx */
  S2LPTimerLdcrMode(S_ENABLE);

  /* enable the fast rx timer */
  S2LpTimerFastRxTermTimer(S_ENABLE);

  /* the RX command triggers the LDC in fast RX termination mode */
  S2LPCmdStrobeCommand(CMD_RX);

  enableS2LPIrq();
}

/**
* @brief  DeInitialize the S2-LP library.
* @param  None.
* @retval None.
*/
void S2LP::end(void)
{
  /* Shutdown the S2-LP */
  digitalWrite(sdn_pin, HIGH);
  delay(1);
  digitalWrite(sdn_pin, LOW);
  delay(1);

  /* S2-LP soft reset */
  S2LPCmdStrobeSres();

  /* Detach S2-LP IRQ */
  detachInterrupt(irq_pin);

  /* Reset CSN pin */
  pinMode(csn_pin, INPUT);

  /* Reset SDN pin */
  pinMode(sdn_pin, INPUT);

  /* Reset all internal variables */  
  memset((void *)&g_xStatus, 0, sizeof(S2LPStatus));
  s_cWMbusSubmode = WMBUS_SUBMODE_NOT_CONFIGURED;
  current_event_callback = NULL;
  nr_of_irq_disabled = 0;
  xTxDoneFlag = RESET;
  memset((void *)vectcRxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  memset((void *)vectcTxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  cRxData = 0;
  is_waiting_for_read = false;
  is_tx_done_before_read = false;
}

/**
* @brief  Attach the callback for Receive event.
* @param  func the Receive callback.
* @retval None.
*/
void S2LP::attachS2LPReceive(S2LPEventHandler func)
{
  current_event_callback = func;
}

/**
* @brief  Send the payload packet.
* @param  payload pointer to the data to be sent.
* @param  payload_len length in bytes of the payload data.
* @param  dest_addr destination address.
* @param  use_csma_ca should CSMA/CA be enabled for transmission.
* @retval zero in case of success, non-zero error code otherwise.
* @note  the maximum payload size allowed is 128 bytes
*/
uint8_t S2LP::send(uint8_t *payload, uint8_t payload_len, uint8_t dest_addr, bool use_csma_ca)
{
  uint32_t start_time;
  uint32_t current_time;

  if(payload_len > FIFO_SIZE)
  {
    return 1;
  }

  disableS2LPIrq();

  /* Go to Ready mode */
  if(S2LPSetReadyState())
  {
    enableS2LPIrq();
    return 1;
  }

  /* Disable LDC */
  S2LPTimerLdcrMode(S_DISABLE);
  S2LpTimerFastRxTermTimer(S_DISABLE);

  S2LPPktBasicSetPayloadLength(payload_len);

  S2LPSetRxSourceReferenceAddress(dest_addr);

  if(use_csma_ca)
  {
    S2LPCsma(S_ENABLE);
  }

  /* Flush TX FIFO */
  S2LPCmdStrobeCommand(CMD_FLUSHTXFIFO);

  memcpy(vectcTxBuff, payload, payload_len);

  S2LPSpiWriteFifo(payload_len, vectcTxBuff);

  /* IRQ registers blanking */
  S2LPGpioIrqClearStatus();

  enableS2LPIrq();

  S2LPCmdStrobeCommand(CMD_TX);

  start_time = millis();

  /* wait for TX done */
  do
  {
    current_time = millis();
  } while(!xTxDoneFlag && (current_time - start_time) <= 1000);

  if(use_csma_ca)
  {
    S2LPCsma(S_DISABLE);
  }

  /* Enable LDC */
  S2LPTimerLdcrMode(S_ENABLE);
  S2LpTimerFastRxTermTimer(S_ENABLE);

  if(is_waiting_for_read)
  {
    is_tx_done_before_read = true;
  } else
  {
    /* Return to RX state */
    S2LPCmdStrobeCommand(CMD_RX);
  }

  disableS2LPIrq();

  /* Timeout case */
  if(!xTxDoneFlag)
  {
    enableS2LPIrq();
    return 1;
  }

  xTxDoneFlag = RESET;
  enableS2LPIrq();

  return 0;
}

/**
* @brief  Get the payload size of the received packet.
* @param  None.
* @retval the payload size of the received packet.
*/
uint8_t S2LP::getRecvPayloadLen(void)
{
  return cRxData;
}

/**
* @brief  Read the payload packet.
* @param  payload pointer to the data to be sent.
* @param  payload_len length in bytes of the payload data.
* @retval the number of read bytes.
* @note  the maximum payload size allowed is 128 bytes
*/
uint8_t S2LP::read(uint8_t *payload, uint8_t payload_len)
{
  uint8_t ret_val = cRxData;

  disableS2LPIrq();

  if(payload_len < cRxData)
  {
    enableS2LPIrq();
    return 0;
  }

  memcpy(payload, vectcRxBuff, cRxData);

  cRxData = 0;

  is_waiting_for_read = false;

  if(is_tx_done_before_read)
  {
    is_tx_done_before_read = false;

    /* Return to RX state */
    S2LPCmdStrobeCommand(CMD_RX);
  } else
  {
    /* Return to Sleep state */
    S2LPCmdStrobeCommand(CMD_SLEEP);

    if(S2LPManagementGetCut()==S2LP_CUT_2_0)
    {
      /* apply the workaround to exit from SLEEP (2nd part) */
      delay(6);
      S2LPTimerLdcIrqWa(S_DISABLE);
    }
  }

  enableS2LPIrq();

  return ret_val;
}

/**
* @brief  Set the Ready state.
* @param  None.
* @retval zero in case of success, non-zero error code otherwise.
*/
uint8_t S2LP::S2LPSetReadyState(void)
{
  uint8_t ret_val = 0;
  uint32_t start_time;
  uint32_t current_time;

  start_time = millis();

  do
  {
    S2LPRefreshStatus();

    if(g_xStatus.MC_STATE == MC_STATE_RX || g_xStatus.MC_STATE == MC_STATE_TX)
    {
      S2LPCmdStrobeCommand(CMD_SABORT);
    } else
    {
      S2LPCmdStrobeCommand(CMD_READY);
    }

    S2LPRefreshStatus();

    current_time = millis();
  } while(g_xStatus.MC_STATE != MC_STATE_READY && (current_time - start_time) <= 1000);

  if((current_time - start_time) > 1000)
  {
    ret_val = 1;
  }

  return ret_val;
}

S2LPCutType S2LP::S2LPManagementGetCut(void)
{
  uint8_t tmp;

  /* Read Cut version from S2LP register */
  S2LPSpiReadRegisters(0xF1, 1, &tmp);
  return (S2LPCutType)tmp;
}

/**
* @brief  IRQ Handler.
* @param  None.
* @retval None.
*/
void S2LP::S2LPIrqHandler(void)
{
  S2LPIrqs xIrqStatus;

  /* Get the IRQ status */
  S2LPGpioIrqGetStatus(&xIrqStatus);

  /* Check the SPIRIT TX_DATA_SENT IRQ flag */
  if(xIrqStatus.IRQ_TX_DATA_SENT)
  {
    /* set the tx_done_flag to manage the event in the send() */
    xTxDoneFlag = SET;
  }

  /* Check the S2LP RX_DATA_READY IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_READY)
  {
    if(S2LPManagementGetCut()==S2LP_CUT_2_0)
    {/* apply the workaround to exit from SLEEP (1st part) */
      S2LPTimerLdcIrqWa(S_ENABLE);
    }

    /* Get the RX FIFO size */
    cRxData = S2LPFifoReadNumberBytesRxFifo();

    /* Read the RX FIFO */
    S2LPSpiReadFifo(cRxData, vectcRxBuff);

    /* Flush the RX FIFO */
    S2LPCmdStrobeFlushRxFifo();

    is_waiting_for_read = true;

    /* Call application callback */
    if(current_event_callback)
    {
      current_event_callback();
    }
  }
}

/**
* @brief  Disable the S2LP interrupts.
* @param  None.
* @retval None.
*/
void S2LP::disableS2LPIrq(void)
{
  detachInterrupt(irq_pin);
  nr_of_irq_disabled++;
}

/**
* @brief  Enable the S2LP interrupts.
* @param  None.
* @retval None.
*/
void S2LP::enableS2LPIrq(void)
{
  if(nr_of_irq_disabled > 0)
  {
    nr_of_irq_disabled--;
    if(nr_of_irq_disabled == 0)
    {
      attachInterrupt(irq_pin, irq_handler, FALLING);
    }
  }
}

/**
* @brief  Write single or multiple registers.
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval Device status
*/ 
S2LPStatus S2LP::S2LPSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer )
{
  uint8_t header[S2LP_CMD_SIZE]={WRITE_HEADER,cRegAddress};
  S2LPStatus status;
  
  SpiSendRecv( header, pcBuffer, cNbBytes );

  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1]; 
  
  return status;
}

/**
* @brief  Read single or multiple registers.
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval Device status
*/
S2LPStatus S2LP::S2LPSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer )
{
  uint8_t header[S2LP_CMD_SIZE]={READ_HEADER,cRegAddress};
  S2LPStatus status;

  SpiSendRecv( header, pcBuffer, cNbBytes );

  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1]; 

  return status;
}

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval Device status
*/
S2LPStatus S2LP::S2LPSpiCommandStrobes(uint8_t cCommandCode)
{
  uint8_t header[S2LP_CMD_SIZE]={COMMAND_HEADER,cCommandCode};
  S2LPStatus status;

  SpiSendRecv( header, NULL, 0 );
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}

/**
* @brief  Write data into TX FIFO.
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval Device status
*/
S2LPStatus S2LP::S2LPSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t header[S2LP_CMD_SIZE]={WRITE_HEADER,LINEAR_FIFO_ADDRESS};
  S2LPStatus status;

  SpiSendRecv( header, pcBuffer, cNbBytes );
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}


/**
* @brief  Read data from RX FIFO.
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval Device status
*/
S2LPStatus S2LP::S2LPSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t header[S2LP_CMD_SIZE]={READ_HEADER,LINEAR_FIFO_ADDRESS};
  S2LPStatus status;

  SpiSendRecv( header, pcBuffer, cNbBytes );
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}

/**
* @brief  Basic SPI function to send/receive data.
* @param  pcHeader: pointer to header to be sent
* @param  pcBuffer: pointer to data to be sent/received
* @param  cNbBytes: number of bytes to send/receive
* @retval Device status
*/
void S2LP::SpiSendRecv(uint8_t *pcHeader, uint8_t *pcBuffer, uint16_t cNbBytes)
{
  disableS2LPIrq();

  dev_spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  digitalWrite(csn_pin, LOW);

  dev_spi->transfer(pcHeader, S2LP_CMD_SIZE);

  if(cNbBytes)
  {
    dev_spi->transfer(pcBuffer, cNbBytes);
  }

  digitalWrite(csn_pin, HIGH);

  enableS2LPIrq();
}

/**
* @brief  Management of RCO calibration.
* @param  None.
* @retval None.
*/
void S2LP::S2LPManagementRcoCalibration(void)
{
  uint8_t tmp[2],tmp2;

  S2LPSpiReadRegisters(0x6D, 1, &tmp2);
  tmp2 |= 0x01;
  S2LPSpiWriteRegisters(0x6D, 1, &tmp2);

  S2LPSpiCommandStrobes(0x63);
  delay(100);
  S2LPSpiCommandStrobes(0x62);

  do
  {
    S2LPSpiReadRegisters(0x8D, 1, tmp);
  }
  while((tmp[0]&0x10)==0);

  S2LPSpiReadRegisters(0x94, 2, tmp);
  S2LPSpiReadRegisters(0x6F, 1, &tmp2);
  tmp[1]=(tmp[1]&0x80)|(tmp2&0x7F);

  S2LPSpiWriteRegisters(0x6E, 2, tmp);
  S2LPSpiReadRegisters(0x6D, 1, &tmp2);
  tmp2 &= 0xFE;

  S2LPSpiWriteRegisters(0x6D, 1, &tmp2);
}
