/*
 *  File:     spi.c
 *  Author:   Keith Penney
 *  Created:  201111
 *  Desc:
 *    This source file and its associated header file implement the hardware control, ISRs, and
 *    software state machine controller for the SPI1 interface of the STM32WB55 for use with
 *    adxl.c/h
 */

#include "spi.h"
#include "adxl.h"   // Remove dependency!
#include <stdio.h>


/* ================================ Debug Prints ============================ */
#define SPI_DBG_EN        0
#if SPI_DBG_EN
#include <stdio.h>
#define SPI_DBG(...)     do{printf("SPI [%d] " ,__LINE__);printf(__VA_ARGS__);}while(0);
#else
#define SPI_DBG(...)
#endif

/* ============================== Private Defines =========================== */
// Make these smaller if possible
#define CFG_ADXL_MAX_BYTES_READ 32
#define CFG_ADXL_MAX_BYTES_WRITE 32
#define SPI_BLOCKING_TIMEOUT  0x1000

#define SPI_PERIPHERAL          SPI1

#ifndef SPI_DMA_CONTROLLER
#define SPI_DMA_CONTROLLER      DMA1
#endif
// RX
#ifndef SPI_DMA_CHANNEL_P2M
#define SPI_DMA_CHANNEL_P2M     LL_DMA_CHANNEL_3
#endif
// TX
#ifndef SPI_DMA_CHANNEL_M2P
#define SPI_DMA_CHANNEL_M2P     LL_DMA_CHANNEL_4
#endif
// LL Macro auto-configuration based on DMA channels
#if SPI_DMA_CHANNEL_P2M == LL_DMA_CHANNEL_1
#define SPI_DMA_P2M_ISR_TCIF  DMA_ISR_TCIF1
#define SPI_DMA_P2M_ISR_HTIF  DMA_ISR_HTIF1
#define SPI_DMA_P2M_ISR_TEIF  DMA_ISR_TEIF1
#define SPI_DMA_P2M_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC1(dma)
#define SPI_DMA_P2M_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE1(dma)
#define SPI_DMA_P2M_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT1(dma)
#elif SPI_DMA_CHANNEL_P2M == LL_DMA_CHANNEL_2
#define SPI_DMA_P2M_ISR_TCIF  DMA_ISR_TCIF2
#define SPI_DMA_P2M_ISR_HTIF  DMA_ISR_HTIF2
#define SPI_DMA_P2M_ISR_TEIF  DMA_ISR_TEIF2
#define SPI_DMA_P2M_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC2(dma)
#define SPI_DMA_P2M_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE2(dma)
#define SPI_DMA_P2M_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT2(dma)
#elif SPI_DMA_CHANNEL_P2M == LL_DMA_CHANNEL_3
#define SPI_DMA_P2M_ISR_TCIF  DMA_ISR_TCIF3
#define SPI_DMA_P2M_ISR_HTIF  DMA_ISR_HTIF3
#define SPI_DMA_P2M_ISR_TEIF  DMA_ISR_TEIF3
#define SPI_DMA_P2M_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC3(dma)
#define SPI_DMA_P2M_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE3(dma)
#define SPI_DMA_P2M_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT3(dma)
#elif SPI_DMA_CHANNEL_P2M == LL_DMA_CHANNEL_4
#define SPI_DMA_P2M_ISR_TCIF  DMA_ISR_TCIF4
#define SPI_DMA_P2M_ISR_HTIF  DMA_ISR_HTIF4
#define SPI_DMA_P2M_ISR_TEIF  DMA_ISR_TEIF4
#define SPI_DMA_P2M_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC4(dma)
#define SPI_DMA_P2M_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE4(dma)
#define SPI_DMA_P2M_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT4(dma)
#elif SPI_DMA_CHANNEL_P2M == LL_DMA_CHANNEL_5
#define SPI_DMA_P2M_ISR_TCIF  DMA_ISR_TCIF5
#define SPI_DMA_P2M_ISR_HTIF  DMA_ISR_HTIF5
#define SPI_DMA_P2M_ISR_TEIF  DMA_ISR_TEIF5
#define SPI_DMA_P2M_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC5(dma)
#define SPI_DMA_P2M_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE5(dma)
#define SPI_DMA_P2M_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT5(dma)
#elif SPI_DMA_CHANNEL_P2M == LL_DMA_CHANNEL_6
#define SPI_DMA_P2M_ISR_TCIF  DMA_ISR_TCIF6
#define SPI_DMA_P2M_ISR_HTIF  DMA_ISR_HTIF6
#define SPI_DMA_P2M_ISR_TEIF  DMA_ISR_TEIF6
#define SPI_DMA_P2M_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC6(dma)
#define SPI_DMA_P2M_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE6(dma)
#define SPI_DMA_P2M_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT6(dma)
#elif SPI_DMA_CHANNEL_P2M == LL_DMA_CHANNEL_7
#define SPI_DMA_P2M_ISR_TCIF  DMA_ISR_TCIF7
#define SPI_DMA_P2M_ISR_HTIF  DMA_ISR_HTIF7
#define SPI_DMA_P2M_ISR_TEIF  DMA_ISR_TEIF7
#define SPI_DMA_P2M_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC7(dma)
#define SPI_DMA_P2M_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE7(dma)
#define SPI_DMA_P2M_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT7(dma)
#endif

#if SPI_DMA_CHANNEL_M2P == LL_DMA_CHANNEL_1
#define SPI_DMA_M2P_ISR_TCIF  DMA_ISR_TCIF1
#define SPI_DMA_M2P_ISR_HTIF  DMA_ISR_HTIF1
#define SPI_DMA_M2P_ISR_TEIF  DMA_ISR_TEIF1
#define SPI_DMA_M2P_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC1(dma)
#define SPI_DMA_M2P_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE1(dma)
#define SPI_DMA_M2P_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT1(dma)
#elif SPI_DMA_CHANNEL_M2P == LL_DMA_CHANNEL_2
#define SPI_DMA_M2P_ISR_TCIF  DMA_ISR_TCIF2
#define SPI_DMA_M2P_ISR_HTIF  DMA_ISR_HTIF2
#define SPI_DMA_M2P_ISR_TEIF  DMA_ISR_TEIF2
#define SPI_DMA_M2P_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC2(dma)
#define SPI_DMA_M2P_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE2(dma)
#define SPI_DMA_M2P_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT2(dma)
#elif SPI_DMA_CHANNEL_M2P == LL_DMA_CHANNEL_3
#define SPI_DMA_M2P_ISR_TCIF  DMA_ISR_TCIF3
#define SPI_DMA_M2P_ISR_HTIF  DMA_ISR_HTIF3
#define SPI_DMA_M2P_ISR_TEIF  DMA_ISR_TEIF3
#define SPI_DMA_M2P_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC3(dma)
#define SPI_DMA_M2P_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE3(dma)
#define SPI_DMA_M2P_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT3(dma)
#elif SPI_DMA_CHANNEL_M2P == LL_DMA_CHANNEL_4
#define SPI_DMA_M2P_ISR_TCIF  DMA_ISR_TCIF4
#define SPI_DMA_M2P_ISR_HTIF  DMA_ISR_HTIF4
#define SPI_DMA_M2P_ISR_TEIF  DMA_ISR_TEIF4
#define SPI_DMA_M2P_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC4(dma)
#define SPI_DMA_M2P_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE4(dma)
#define SPI_DMA_M2P_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT4(dma)
#elif SPI_DMA_CHANNEL_M2P == LL_DMA_CHANNEL_5
#define SPI_DMA_M2P_ISR_TCIF  DMA_ISR_TCIF5
#define SPI_DMA_M2P_ISR_HTIF  DMA_ISR_HTIF5
#define SPI_DMA_M2P_ISR_TEIF  DMA_ISR_TEIF5
#define SPI_DMA_M2P_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC5(dma)
#define SPI_DMA_M2P_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE5(dma)
#define SPI_DMA_M2P_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT5(dma)
#elif SPI_DMA_CHANNEL_M2P == LL_DMA_CHANNEL_6
#define SPI_DMA_M2P_ISR_TCIF  DMA_ISR_TCIF6
#define SPI_DMA_M2P_ISR_HTIF  DMA_ISR_HTIF6
#define SPI_DMA_M2P_ISR_TEIF  DMA_ISR_TEIF6
#define SPI_DMA_M2P_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC6(dma)
#define SPI_DMA_M2P_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE6(dma)
#define SPI_DMA_M2P_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT6(dma)
#elif SPI_DMA_CHANNEL_M2P == LL_DMA_CHANNEL_7
#define SPI_DMA_M2P_ISR_TCIF  DMA_ISR_TCIF7
#define SPI_DMA_M2P_ISR_HTIF  DMA_ISR_HTIF7
#define SPI_DMA_M2P_ISR_TEIF  DMA_ISR_TEIF7
#define SPI_DMA_M2P_CLEAR_TCIF(dma)   LL_DMA_ClearFlag_TC7(dma)
#define SPI_DMA_M2P_CLEAR_TEIF(dma)   LL_DMA_ClearFlag_TE7(dma)
#define SPI_DMA_M2P_CLEAR_HTIF(dma)   LL_DMA_ClearFlag_HT7(dma)
#endif

/* ============================= Private Typedefs =========================== */
typedef struct {
  uint8_t start;
  uint8_t busy;
  uint8_t stop;
  uint32_t len;
  uint8_t wdata[CFG_ADXL_MAX_BYTES_WRITE];
  uint8_t rdata[CFG_ADXL_MAX_BYTES_READ];
} spi_controller_t;

/* ============================= Static Variables =========================== */
static spi_controller_t spi_controller;
//static queueItem_t adxl_qItem;
static void (*adxl_Callback)(void);

/* ======================= Private Function Prototypes ====================== */
static void _SPI_Start(uint32_t len);
static void _SPI_Stop(void);
static inline void _SPI_SetDMAAddrRead(uint32_t *pdata);
static inline void _SPI_SetDMAAddrWrite(uint32_t *pdata);

/* =========================== Function Definitions ========================= */
uint8_t SPI_SysInit(void) {
  spi_controller.start = 0;
  spi_controller.busy = 0;
  spi_controller.len = 0;
  spi_controller.stop = 0;
  // Set Rx FIFO threshold to 1/4 (8 bits)
  // This will trigger an RXNE IRQ when 8 bits have been received.
  LL_SPI_SetRxFIFOThreshold(SPI_PERIPHERAL, LL_SPI_RX_FIFO_TH_QUARTER);
  // Let's try this for a second.
  LL_DMA_DisableIT_TC(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M);
  LL_DMA_DisableIT_TC(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P);
  return 0;
}

void SPI_Start(uint32_t len) {
  SPI_DBG("len = %ld, busy? %d, stop? %d, start? %d\r\n", len, spi_controller.busy, spi_controller.stop, spi_controller.start);
  spi_controller.start = 1;
  spi_controller.len = len;
  return;
}

static void _SPI_Start(uint32_t len) {
#if SOFTWARE_NSS
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
#endif
  // Set spi_controller busy flag
  spi_controller.busy = 1;
  // We now configure DMA Rx channel in SPI_Control based on command type in the queue
  /*
  LL_DMA_ConfigAddresses(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M, LL_SPI_DMA_GetRegAddr(SPI_PERIPHERAL),
                         (uint32_t)&spi_controller.rdata, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  */
  // Configure DMA Tx channel to read from spi_controller.wdata and write to SPI_PERIPHERAL.DR
  LL_DMA_ConfigAddresses(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P, (uint32_t)&spi_controller.wdata, 
                         LL_SPI_DMA_GetRegAddr(SPI_PERIPHERAL), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  // Enable RXNE and TXE interrupts
  LL_SPI_EnableIT_RXNE(SPI_PERIPHERAL);
  LL_SPI_EnableIT_TXE(SPI_PERIPHERAL);
  // Set number of data to transfer for DMA channel
  LL_DMA_SetDataLength(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M, len); // Rx
  LL_DMA_SetDataLength(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P, len); // Tx
  // Enable DMA stream(s) for Rx/Tx
  // Rx Channel Enable and DMA transfer complete interrupt
  LL_DMA_EnableIT_TC(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M);
  LL_DMA_EnableIT_HT(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M); // Enable ALL interrupts from DMA
  //LL_DMA_EnableIT_TE(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M);
  LL_DMA_EnableChannel(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M);
  // Tx Channel Enable
  LL_DMA_EnableIT_TC(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P);
  //LL_DMA_EnableIT_HT(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P); // Enable ALL interrupts from DMA
  //LL_DMA_EnableIT_TE(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P);
  LL_DMA_EnableChannel(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P);
  // Enable DMA Tx buffer (SPI_PERIPHERAL->CRE:TXDMAEN)
  LL_SPI_EnableDMAReq_TX(SPI_PERIPHERAL);
  // Enable DMA Rx buffer (SPI_PERIPHERAL->CR2:RXDMAEN)
  LL_SPI_EnableDMAReq_RX(SPI_PERIPHERAL);
  // Enable SPI
  LL_SPI_Enable(SPI_PERIPHERAL);
  return;
}

/*
 * uint8_t SPI_BlockingReadOne(uint8_t wbyte, uint8_t *rdata);
 *    Read 1 byte of data from the SPI slave device in blocking mode.
 *    'wbyte' is the sole non-zero byte written to MOSI (first byte).
 *    The byte received during 'wbyte' is discarded.
 *    The following byte is placed in 'rdata'
 *    If the spi_controller busy flag is cleared, this function executes
 *    in approximately 5.12us (with 32MHz sysclk)
 */
uint8_t SPI_BlockingReadOne(uint8_t wbyte, uint8_t *rdata) {
  //uint16_t state = READ_REG(SPI_PERIPHERAL->SR);
  uint32_t timer = 0;
  //DBG1S();
  // If spi_controller is busy
  if (spi_controller.busy) {
    // Wait for stop signal
    while (!spi_controller.stop) {
      timer++;
      // Return if we reach the timeout
      if (timer > SPI_BLOCKING_TIMEOUT) {
        return SPI_BLOCKING_READ_TIMEOUT;
      }
    }
    // If we reach here, spi_controller.stop was set by ISR
    _SPI_Stop();
  }
  // Disable interrupts if they were previously enabled
  LL_SPI_DisableIT_ERR(SPI_PERIPHERAL);
  // Start the blocking read transaction
  // Load data into the DR
  LL_SPI_TransmitData8(SPI_PERIPHERAL, wbyte);
  LL_SPI_TransmitData8(SPI_PERIPHERAL, 0x00);
  LL_SPI_Enable(SPI_PERIPHERAL);
  uint32_t flvl = 0;
  // Wait for TXE
  while (flvl != LL_SPI_TX_FIFO_EMPTY) {
    flvl = LL_SPI_GetTxFIFOLevel(SPI_PERIPHERAL);
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  LL_SPI_DisableIT_TXE(SPI_PERIPHERAL);
  // Wait for RX level = 2
  timer = 0;
  while (flvl < LL_SPI_RX_FIFO_HALF_FULL) {
    flvl = LL_SPI_GetRxFIFOLevel(SPI_PERIPHERAL);
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  LL_SPI_ReceiveData8(SPI_PERIPHERAL);  // Throw away the first byte
  *rdata = LL_SPI_ReceiveData8(SPI_PERIPHERAL); // The second byte is the one we want
  LL_SPI_DisableIT_RXNE(SPI_PERIPHERAL);
  // Wait for SPI busy flag to clear
  while (LL_SPI_IsActiveFlag_BSY(SPI_PERIPHERAL)) {
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  LL_SPI_Disable(SPI_PERIPHERAL);
  //DBG1R();
  return SPI_BLOCKING_READ_SUCCESS;
}

/*
 * uint8_t SPI_BlockingRead(uint8_t wbyte, uint8_t nBytes, uint8_t *rdata);
 *    Read data from SPI device in blocking mode.
 *    'wbyte' is the sole non-zero byte written to MOSI (first byte).
 *    'nBytes' is the number of bytes to read (not counting 'wbyte')
 *    The byte received during 'wbyte' is discarded.
 *    All subsequent bytes are placed sequentially into 'rdata'
 *    If the spi_controller busy flag is cleared, the execution time of this
 *    function is roughly (5.014 + 1.136 * nBytes) microseconds (with 32MHz sysclk)
 */
uint8_t SPI_BlockingRead(uint8_t wbyte, uint8_t nBytes, uint8_t *rdata) {
  uint32_t timer = 0;
  if (nBytes == 0) {
    return SPI_BLOCKING_READ_ZEROBYTES;
  }
  //DBG1S();
  // If spi_controller is busy
  if (spi_controller.busy) {
    // Wait for stop signal
    while (!spi_controller.stop) {
      timer++;
      // Return if we reach the timeout
      if (timer > SPI_BLOCKING_TIMEOUT) {
        return SPI_BLOCKING_READ_TIMEOUT;
      }
    }
    // If we reach here, spi_controller.stop was set by ISR
    _SPI_Stop();
  }
  // Disable interrupts if they were previously enabled
  LL_SPI_DisableIT_ERR(SPI_PERIPHERAL);
  // Start the blocking read transaction
  // Load data into the DR
  LL_SPI_TransmitData8(SPI_PERIPHERAL, wbyte);    // Mandatory
  LL_SPI_TransmitData8(SPI_PERIPHERAL, 0x00);     // nBytes must be >= 1
  uint32_t nTx = 2;
  uint32_t nRx = 0;
  LL_SPI_Enable(SPI_PERIPHERAL);
  uint32_t txlvl = LL_SPI_GetTxFIFOLevel(SPI_PERIPHERAL);
  uint32_t rxlvl;
  while (nRx < nBytes + 1) {
    // Read from DR when data available
    rxlvl = LL_SPI_GetRxFIFOLevel(SPI_PERIPHERAL);
    if (rxlvl != LL_SPI_RX_FIFO_EMPTY) {
      if (nRx == 0) {
        LL_SPI_ReceiveData8(SPI_PERIPHERAL);  // Throw away the first byte
      } else {
        *(rdata + (nRx - 1)) = LL_SPI_ReceiveData8(SPI_PERIPHERAL);
      }
      nRx++;
      timer = 0;    // Reset timer when we receive a byte
    }
    // Write to DR when space available
    if (txlvl != LL_SPI_TX_FIFO_FULL) {
      if (nTx < nBytes + 1) {
        LL_SPI_TransmitData8(SPI_PERIPHERAL, 0x00);   // Write another dummy byte to keep SPI going
        nTx++;
        timer = 0;    // Reset timer when we receive a byte
      }
    }   
    // Check for timeout
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  // Wait for TXE
  while (txlvl != LL_SPI_TX_FIFO_EMPTY) {
    txlvl = LL_SPI_GetTxFIFOLevel(SPI_PERIPHERAL);
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  LL_SPI_DisableIT_TXE(SPI_PERIPHERAL);
  // Wait for RXNE = 0
  LL_SPI_DisableIT_RXNE(SPI_PERIPHERAL);
  // Wait for SPI busy flag to clear
  while (LL_SPI_IsActiveFlag_BSY(SPI_PERIPHERAL)) {
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  LL_SPI_Disable(SPI_PERIPHERAL);
  //DBG1R();
  return SPI_BLOCKING_READ_SUCCESS;
}

uint8_t _SPI_BlockingRead(uint8_t nHeader, uint8_t *wbytes, uint8_t nRead, uint8_t *rdata) {
  uint32_t timer = 0;
  if (nRead == 0) {
    return SPI_BLOCKING_READ_ZEROBYTES;
  }
  //DBG1S();
  // If spi_controller is busy
  if (spi_controller.busy) {
    // Wait for stop signal
    while (!spi_controller.stop) {
      timer++;
      // Return if we reach the timeout
      if (timer > SPI_BLOCKING_TIMEOUT) {
        return SPI_BLOCKING_READ_TIMEOUT;
      }
    }
    // If we reach here, spi_controller.stop was set by ISR
    _SPI_Stop();
  }
  // Disable interrupts if they were previously enabled
  LL_SPI_DisableIT_ERR(SPI_PERIPHERAL);
  // Start the blocking read transaction
  // Load data into the DR
  for (uint8_t n = 0; n < nHeader; n++) {
    LL_SPI_TransmitData8(SPI_PERIPHERAL, wbytes[n]); // Write header bytes
  }
  //LL_SPI_TransmitData8(SPI_PERIPHERAL, 0x00);        // nRead must be >= 1
  uint32_t nTx = nHeader;
  uint32_t nRx = 0;
  LL_SPI_Enable(SPI_PERIPHERAL);
  uint32_t txlvl = LL_SPI_GetTxFIFOLevel(SPI_PERIPHERAL);
  uint32_t rxlvl;
  while (nRx < nRead + nHeader) {
    // Read from DR when data available
    rxlvl = LL_SPI_GetRxFIFOLevel(SPI_PERIPHERAL);
    txlvl = LL_SPI_GetTxFIFOLevel(SPI_PERIPHERAL);
    if (rxlvl != LL_SPI_RX_FIFO_EMPTY) {
      if (nRx < nHeader) {
        LL_SPI_ReceiveData8(SPI_PERIPHERAL);  // Throw away the first byte
      } else {
        *(rdata + (nRx - nHeader)) = LL_SPI_ReceiveData8(SPI_PERIPHERAL);
      }
      nRx++;
      timer = 0;    // Reset timer when we receive a byte
    }
    // Write to DR when space available
    if (txlvl != LL_SPI_TX_FIFO_FULL) {
      if (nTx < nRead + nHeader) {
        LL_SPI_TransmitData8(SPI_PERIPHERAL, 0x00);   // Write another dummy byte to keep SPI going
        nTx++;
        timer = 0;    // Reset timer when we receive a byte
      }
    }   
    // Check for timeout
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  // Wait for TXE
  while (txlvl != LL_SPI_TX_FIFO_EMPTY) {
    txlvl = LL_SPI_GetTxFIFOLevel(SPI_PERIPHERAL);
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  LL_SPI_DisableIT_TXE(SPI_PERIPHERAL);
  // Wait for RXNE = 0
  LL_SPI_DisableIT_RXNE(SPI_PERIPHERAL);
  // Wait for SPI busy flag to clear
  while (LL_SPI_IsActiveFlag_BSY(SPI_PERIPHERAL)) {
    timer++;
    if (timer > SPI_BLOCKING_TIMEOUT) {
      return SPI_BLOCKING_READ_TIMEOUT;
    }
  }
  LL_SPI_Disable(SPI_PERIPHERAL);
  //DBG1R();
  return SPI_BLOCKING_READ_SUCCESS;
}

static void _SPI_Stop(void) {
  //DBG2S();
  uint32_t flvl;
  // Stop SPI procedure
  // Disable Rx DMA stream
  LL_DMA_DisableChannel(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M);
  // Disable Tx DMA stream
  LL_DMA_DisableChannel(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P);
  // Wait for FTLVL[1:0] = 00
  flvl = LL_SPI_GetTxFIFOLevel(SPI_PERIPHERAL);
  if (flvl) {
    // Early return if FTLVL != 00
    return;
  }
  // Wait for BSY=0
  if (LL_SPI_IsActiveFlag_BSY(SPI_PERIPHERAL)) {
    // Early return if BSY flag is asserted
    return;
  }
  // If we get here, we can disable SPI_PERIPHERAL
  // SPI_PERIPHERAL->CR2:SPE = 0
  LL_SPI_Disable(SPI_PERIPHERAL);
  // Tell the controller that the peripheral has been stopped
  spi_controller.stop = 0;
  // If there's anything left in the RX buffer, read it.
  // Read data until FRLVL[1:0] = 00
  flvl = LL_SPI_GetRxFIFOLevel(SPI_PERIPHERAL);
  
  while (flvl) {
    // Just throw it away for now
    LL_SPI_ReceiveData8(SPI_PERIPHERAL);
  }
  
  // Disable DMA Tx/Rx buffers by clearing TXDMAEN and RXDMAEN from SPI_PERIPHERAL->CR2
  LL_SPI_DisableDMAReq_RX(SPI_PERIPHERAL);
  LL_SPI_DisableDMAReq_TX(SPI_PERIPHERAL);
  spi_controller.busy = 0;

  // Handle the end of the ADXL command
  // If last command was a read (and not direct mode), copy the read data
  // CHANGE. It's the caller's responsibility to copy the data in its callback function
  /*
  if (((adxl_qItem.taddr & ADXL_TADDR_READ_BM) == ADXL_TADDR_READ) && ((adxl_qItem.direct & ADXL_QUEUEITEM_DIRECT_BM) == ADXL_QUEUEITEM_INDIRECT)) {
    for (uint32_t n = 0; n < adxl_qItem.datalen; n++) {
      *(adxl_qItem.pdata + n) = spi_controller.rdata[1 + n];  // Remember the first rdata byte is garbage
    }
  }
  */
  // Call the pre-registered callback if non-null
  if (adxl_Callback != ADXL_PNULL) {
    adxl_Callback();
    adxl_Callback = ADXL_PNULL; // Once we return, forget the callback
  }
  //DBG2R();
  return;
}

uint8_t SPI_GetRData(uint8_t *pData, uint32_t nBytes, uint32_t offset) {
  for (uint32_t n = 0; n < nBytes; n++) {
    if (n + offset > CFG_ADXL_MAX_BYTES_READ - 1) {
      return SPI_OVERRUN;
    }
    pData[n] = spi_controller.rdata[offset + n];
  }
  return SPI_OK;
}

void SPI_SetCmdCallback(void (*cb)(void)) {
  adxl_Callback = cb;
  return;
}

void SPI_Control(void) {
  // Handle a stop request
  if (spi_controller.stop) {
    _SPI_Stop();
  }
  if (spi_controller.start) {
    spi_controller.start = 0;
    //SPI_DBG("Starting %ld\r\n", spi_controller.len);
    _SPI_Start((uint32_t)spi_controller.len);
  }
  return;
}

int SPI_Busy(void) {
  // Is the controller busy?
  if (spi_controller.busy) {
    return SPI_BUSY;
  }
  // If we've registered a start but haven't serviced it, we're busy
  if (spi_controller.start) {
    return SPI_BUSY;
  }
  // Is the SPI peripheral busy?
  if (LL_SPI_IsActiveFlag_BSY(SPI_PERIPHERAL)) {
    return SPI_BUSY;
  }
  // Otherwise, we're free
  return SPI_FREE;
}

/*
 * uint8_t SPI_SetWData(uint8_t *pData, uint32_t nBytes, uint32_t offset);
 *    Copy 'nBytes' of data from pData to spi_controller.wdata starting from 
 *    byte 'offset'.
 *    Returns SPI_OVERRUN if nBytes + offset is greater than the size of
 *    spi_controller.wdata (but still copies up to the buffer size).
 *    Returns SPI_OK otherwise.
 */
uint8_t SPI_SetWData(uint8_t *pData, uint32_t nBytes, uint32_t offset) {
  //SPI_DBG("SetWData: 0x%x\r\n", pData[0]);
  if (nBytes == 0) {
    return SPI_OK;
  }
  for (uint32_t n = 0; n < nBytes; n++) {
    if (n + offset > CFG_ADXL_MAX_BYTES_WRITE - 1) {
      return SPI_OVERRUN;
    }
    spi_controller.wdata[n + offset] = pData[n];
  }
  return SPI_OK;
}

void SPI_SetDMAAddrRead(uint32_t *pdata) {
  return _SPI_SetDMAAddrRead(pdata);
}

void SPI_SetDMAReadDefault(void) {
  return _SPI_SetDMAAddrRead((uint32_t *)spi_controller.rdata);
}

static inline void _SPI_SetDMAAddrRead(uint32_t *pdata) {
  // Configure DMA Rx channel to read from SPI_PERIPHERAL DR and write to spi_controller.rdata
  LL_DMA_ConfigAddresses(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M, LL_SPI_DMA_GetRegAddr(SPI_PERIPHERAL),
                         (uint32_t)pdata, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  return;
}

static inline void _SPI_SetDMAAddrWrite(uint32_t *pdata) {
  // Configure DMA Tx channel to read from spi_controller.wdata and write to SPI_PERIPHERAL.DR
  LL_DMA_ConfigAddresses(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P, (uint32_t)pdata, 
                         LL_SPI_DMA_GetRegAddr(SPI_PERIPHERAL), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  return;
}

// DMA SPI RX channel
void SPI_DMAP2MISR(void) {
  // Must clear the interrupt flags manually
  uint32_t iflags = LL_DMA_ReadReg(SPI_DMA_CONTROLLER, ISR);
  if (iflags & SPI_DMA_P2M_ISR_TEIF) {
    // If Transfer Error Interrupt occurred
    SPI_DMA_P2M_CLEAR_TEIF(SPI_DMA_CONTROLLER);
  }
  if (iflags & SPI_DMA_P2M_ISR_TCIF) {
    // If Transfer Complete Interrupt occurred
    SPI_DMA_P2M_CLEAR_TCIF(SPI_DMA_CONTROLLER);
    uint32_t datalength = LL_DMA_GetDataLength(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M);
    // If receive datalength is 0, we're all done with the transfer
    if (datalength == 0) {
      // Also disable the DMA transfer complete IRQ
      LL_DMA_DisableIT_TC(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_P2M);
      spi_controller.stop = 1;
      // We need to stop the SPI from generating more interrupts.
      LL_SPI_DisableIT_RXNE(SPI_PERIPHERAL);
      LL_SPI_DisableIT_TXE(SPI_PERIPHERAL);
      LL_SPI_Disable(SPI_PERIPHERAL); // Let's do this for now
    }
  }
  if (iflags & SPI_DMA_P2M_ISR_HTIF) {
    // If Half-Transfer Complete Interrupt occurred
    SPI_DMA_P2M_CLEAR_HTIF(SPI_DMA_CONTROLLER);
  }
  return;
}

// DMA SPI TX channel
void SPI_DMAM2PISR(void) {
  // Must clear the interrupt flags manually
  uint32_t iflags = LL_DMA_ReadReg(SPI_DMA_CONTROLLER, ISR);
  if (iflags & SPI_DMA_M2P_ISR_TEIF) {
    // If Transfer Error Interrupt occurred
    SPI_DMA_M2P_CLEAR_TEIF(SPI_DMA_CONTROLLER);
  }
  if (iflags & SPI_DMA_M2P_ISR_TCIF) {
    // If Transfer Complete Interrupt occurred
    SPI_DMA_M2P_CLEAR_TCIF(SPI_DMA_CONTROLLER);
    uint32_t datalength = LL_DMA_GetDataLength(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P);
    // If receive datalength is 0, we're all done with the transfer
    if (datalength == 0) {
      // If finished, disable the DMA transfer complete IRQ
      LL_DMA_DisableIT_TC(SPI_DMA_CONTROLLER, SPI_DMA_CHANNEL_M2P);
    }
  }
  if (iflags & SPI_DMA_M2P_ISR_HTIF) {
    // If Half-Transfer Complete Interrupt occurred
    SPI_DMA_M2P_CLEAR_HTIF(SPI_DMA_CONTROLLER);
  }

  return;
}

void handleI2C1IRQ(void) {

  return;
}
