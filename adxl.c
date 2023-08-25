/*
 *  File:     adxl.c
 *  Author:   Keith Penney
 *  Created:  201111
 *  Desc:
 *  -----
 *    This source file and its associated header file implement an asynchronous read/write
 *    API for the ADXL372 accelerometer via SPI interface.
 */

#include "adxl.h"
#include "spi.h"
//#include "app_debug.h"

/* ============================== Private Defines =========================== */
#define ADXL_DBG_EN        0
#if ADXL_DBG_EN
#include <stdio.h>
#define ADXL_DBG(...)     do{printf("ADXL [%d] " ,__LINE__);printf(__VA_ARGS__);}while(0);
#else
#define ADXL_DBG(...)
#endif

// Set this to zero to skip compiling the static test functions if not being used.
#define ADXL_INCLUDE_TESTS            (0)

// _ALLOC_GET_FREE() is only called after it is verified that the buffer is not full, therefore if
// pIn==pOut, the buffer must be empty.
#define _ALLOC_GET_FREE_S(pIn, pOut)    (pIn >= pOut ? _ALLOC_BYTES_RESERVED - pIn + pOut : pOut - pIn)
#define _ALLOC_GET_FREE()               (_ALLOC_GET_FREE_S(adxl_wSpace.pIn, adxl_wSpace.pOut))
#define ALLOC_INSERT(index, element)   (adxl_wSpace.wspace[(index) % _ALLOC_BYTES_RESERVED] = element)
#define ALLOC_GET_POINTER(index)       ((uint8_t *)&adxl_wSpace.wspace[(index) % _ALLOC_BYTES_RESERVED])

// The number of bytes of SPI transaction header (e.g. 1 for ADXL372, 2 for ADXL363?)
#if ADXL_TARGET == ADXL_TARGET_ADXL372
#define ADXL_HEADER_OFFSET          (1)
#elif ADXL_TARGET == ADXL_TARGET_ADXL363
#define ADXL_HEADER_OFFSET          (2)
#else
#warn "No valid target defined!"
#define ADXL_HEADER_OFFSET          (0)
#endif

/* ========================= ADXL372 Register Map =========================== */
// 0x42 FIFO_DATA
// The series start bit is the 0th bit of the 2nd byte (LS byte).  If it is 1, it indicates
// the first sample in a series.
#define ADXL_REG_FIFO_DATA_START_MASK (0x1)

/* ============================= Private Typedefs =========================== */
typedef struct {
  uint8_t pIn;
  uint8_t pOut;
  uint8_t full;
  queueItem_t queue[ADXL_MAX_QUEUE_ITEMS];
} adxl_queue_t;

typedef struct {
  uint8_t len;
  uint8_t data[4];
} adxl_allocEntry_t;

typedef struct {
  uint8_t pIn;
  uint8_t pOut;
  uint8_t full;
  uint8_t wspace[_ALLOC_BYTES_RESERVED];
} adxl_alloc_t;

#define _ADXL_DEVICE_STATE_STBY       (0x00)
#define _ADXL_DEVICE_STATE_GDETECT    (0x01)
#define _ADXL_DEVICE_STATE_DAQ        (0x02)
struct {
  uint8_t state;
} adxl_dev;

#if ADXL_TARGET == ADXL_TARGET_ADXL363
typedef struct {
  uint32_t nSamples;  // How many samples we want
  uint32_t nEntries;  // How many samples we have in the fifo
  uint8_t *pData;
  void (*callback)(void);
} adxl_adc_fifo_t;
#endif

/* ============================= Static Variables =========================== */
// Command queue FIFO
static adxl_queue_t adxl_queue;
// The active queue item
static queueItem_t adxl_activeQueueItem;
// Write space FIFO (simple allocator)
static adxl_alloc_t adxl_wSpace;
// Local copy of writable device registers
static adxl_reg_t adxl_reg;
#if ADXL_TARGET == ADXL_TARGET_ADXL363
static adxl_adc_fifo_t adxl_adc_fifo;
#endif
// ======================= DEBUG/TESTING ======================
#if ADXL_INCLUDE_TESTS
#define TESTFIFOSAMPLES 512

#if ADXL_TARGET == ADXL_TARGET_ADXL372
#define _TEST_REG_WRITE ADXL_REG_OFFSET_X
#else
#define _TEST_REG_WRITE ADXL_REG_THRESH_ACT_L
#endif
static struct {
  uint8_t pIn;
  uint8_t pOut;
  uint8_t regValues[8];
  uint8_t fifoValues[2*TESTFIFOSAMPLES];
} testBucket;

static void testCallbackReadReg(void);
static void testCallbackReadBurst(void);
static void testReadBurst(void);
static void testReadReg(void);
static void testCallbackWriteReg(void);
static void testCallbackWriteBurst(void);
static void testWriteReg(void);
static void testWriteBurst(void);
static void testAllocator(void);
static void testGetXYZ(void);
static void testFIFO(void);
static void testExtTrig(void);
static void testDAQMode(void);
static void testDirectMode(void);
static void testBlockingRead(void);
static void test363ADCFifo(void);
static void test363ADC(void);
static void testThresholds(void);
static void testGDetectMode(void);
static void testActInact(void);
#endif  /*ADXL_INCLUDE_TESTS*/

/* ============================== Globals/Externs =========================== */

/* ======================= Private Function Prototypes ====================== */
static void _ADXL_InitCallbacks(void);
static uint8_t _ADXL_Read(uint8_t nRegStart, uint32_t nReads, uint8_t *pData, uint8_t direct, void (*callback)(void));
static uint8_t _ADXL_ReadReg(uint8_t nReg, uint8_t *pData);
static uint8_t _ADXL_ReadBurst(uint8_t nRegStart, uint32_t nReads, uint8_t *pData);
static uint8_t _ADXL_ReadRegDirect(uint8_t nReg, uint8_t *pData);
static uint8_t _ADXL_ReadBurstDirect(uint8_t nRegStart, uint32_t nReads, uint8_t *pData);
static uint8_t _ADXL_ReadRegBlocking(uint8_t nReg, uint8_t *pData);
static uint8_t _ADXL_ReadBurstBlocking(uint8_t nRegStart, uint8_t nReads, uint8_t *pData);
static uint8_t _ADXL_Write(uint8_t nRegStart, uint32_t nWrites, uint8_t *pData, void (*callback)(void));
static uint8_t _ADXL_WriteReg(uint8_t nReg, uint8_t *pData);
static uint8_t _ADXL_WriteRegAlloc(uint8_t nReg, uint8_t val);
static uint8_t _ADXL_WriteBurst(uint8_t nRegStart, uint32_t nWrites, uint8_t *pData);
static uint8_t _ADXL_WriteBurstAlloc(uint8_t nRegStart, uint8_t nWrites, uint8_t *pData);
static uint8_t ADXL_QueueAdd(queueItem_t *item);
static inline void _ADXL_CopyRData(void);
static void _ADXL_UpdateLocal(uint8_t nRegStart, int nWrites, uint8_t *pData);
static void ADXL_QueueInit(void);
static uint8_t _alloc_add(uint8_t len);
static void _alloc_free(void);
static uint8_t _ADXL_BlockingReadReg(uint8_t nReg, uint8_t *pData);
static uint8_t _ADXL_BlockingReadBurst(uint8_t nRegStart, uint8_t nReads, uint8_t *pData);

#if ADXL_TARGET == ADXL_TARGET_ADXL363
static void ADXL_CallbackReadADC(void);
uint8_t ADXL_ReadFIFO_363(uint32_t n, uint8_t *pData);
#endif

static struct {
  void (*readReg)(void);
  void (*readBurst)(void);
  void (*readRegDirect)(void);
  void (*readBurstDirect)(void);
  void (*writeReg)(void);
  void (*writeBurst)(void);
} user_callbacks;

/* =========================== Function Definitions ========================= */
/* ========================= High-Level API Functions ======================= */
void ADXL_Init(void) {
  ADXL_DBG("Init\r\n");
  _ADXL_InitCallbacks();
  SPI_SysInit();
  ADXL_QueueInit();
  // Set local device register defaults
  for (int n = 0; n < sizeof(adxl_reg_t); n++) {
    *((uint8_t *)&adxl_reg + n) = 0;
  }
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // Offsets only applicable for ADXL372
#if ADXL_TARGET_AXIS == _ADXL_AXIS_X
  ADXL_SetOffsetX(ADXL_TARGET_AXIS_OFFSET);
#elif ADXL_TARGET_AXIS == _ADXL_AXIS_Y
  ADXL_SetOffsetY(ADXL_TARGET_AXIS_OFFSET);
#else // Assume _ADXL_AXIS_Z
  ADXL_SetOffsetZ(ADXL_TARGET_AXIS_OFFSET);
#endif  // ADXL_TARGET_AXIS == _ADXL_AXIS_Y
#else // ADXL_TARGET == ADXL_TARGET_ADXL363
  adxl_adc_fifo.nSamples = 0;
  adxl_adc_fifo.nEntries = 0;
#endif // ADXL_TARGET == ADXL_TARGET_ADXL372
  adxl_dev.state = _ADXL_DEVICE_STATE_STBY;
  ADXL_SetModeGDetect();
  return;
}

void ADXL_Control(void) {
#if ADXL_STANDALONE
  SPI_Control();
#endif
  // If spi is busy, early return
  if (SPI_Busy()) {
    //ADXL_DBG("Busy\r\n");
    return;
  }
  // Check for another item in the queue
  if (ADXL_QueueNext() == ADXL_QUEUE_EMPTY) {
    return;
  }
  // If we get here, there is another item in the queue to be processed.
  // Check for another item in the command queue
  if (ADXL_QueueGet(&adxl_activeQueueItem) == ADXL_QUEUE_OK) {
    ADXL_DBG("New Item: 0x%x, %ld\r\n", adxl_activeQueueItem.taddr, adxl_activeQueueItem.datalen);
    // We have another item to process
    // Add the register addres + r/!w flag to the SPI's wdata buffer
#if ADXL_TARGET == ADXL_TARGET_ADXL372
    SPI_SetWData(&(adxl_activeQueueItem.taddr), 1, 0);
#else
    uint8_t hdr[2];
    if (ADXL_UNPACK_TADDR(adxl_activeQueueItem.taddr) == ADXL_363_CMD_READ_FIFO) {
      // If taddr is specifically "Read FIFO"
      hdr[0] = ADXL_363_CMD_READ_FIFO;
    } else if (ADXL_TADDR_ISREAD(adxl_activeQueueItem.taddr)) {
      // If taddr indicates a read
      hdr[0] = ADXL_363_CMD_READ;
    } else {
      // If taddr indicates a write
      hdr[0] = ADXL_363_CMD_WRITE;
    }
    hdr[1] = ADXL_UNPACK_TADDR(adxl_activeQueueItem.taddr);
    SPI_SetWData(hdr, 2, 0);
#endif
    //spi_controller.wdata[0] = adxl_activeQueueItem.taddr;
    if ((adxl_activeQueueItem.taddr & ADXL_TADDR_READ_BM) == ADXL_TADDR_WRITE) {
      // We have data to write; Copy it to SPI's wdata buffer (after the taddr byte)
      SPI_SetWData(adxl_activeQueueItem.pdata, adxl_activeQueueItem.datalen, ADXL_HEADER_OFFSET);
      //spi_controller.wdata[1 + n] = *(adxl_activeQueueItem.pdata + n);
    }
    // Check for direct mode
    if ((adxl_activeQueueItem.direct & ADXL_QUEUEITEM_DIRECT_BM) == ADXL_QUEUEITEM_DIRECT) {
      // Change DMA read address
      SPI_SetDMAAddrRead((uint32_t *)adxl_activeQueueItem.pdata);
    } else {
      // Otherwise, default to local rdata buffer
      SPI_SetDMAReadDefault();
      //SPI_SetDMAAddrRead((uint32_t *)spi_controller.rdata);
    }
    // If the command has a non-null callback, register it
    if (adxl_activeQueueItem.callback != ADXL_PNULL) {
      //adxl_Callback = adxl_activeQueueItem.callback;
      SPI_SetCmdCallback(adxl_activeQueueItem.callback);
    }
    // Start the next SPI transfer
    SPI_Start((uint32_t)adxl_activeQueueItem.datalen + ADXL_HEADER_OFFSET);
  }
  return;
}

static void _ADXL_InitCallbacks(void) {
  user_callbacks.readReg = ADXL_PNULL;
  user_callbacks.readBurst = ADXL_PNULL;
  user_callbacks.readRegDirect = ADXL_PNULL;
  user_callbacks.readBurstDirect = ADXL_PNULL;
  user_callbacks.writeReg = ADXL_PNULL;
  user_callbacks.writeBurst = ADXL_PNULL;
  return;
}

void ADXL_RegisterCallback(ADXL_NCB_t nCB, void (*callback)(void)) {
  switch (nCB) {
    case ADXL_NCB_READREG:
      user_callbacks.readReg = callback;
      break;
    case ADXL_NCB_READBURST:
      user_callbacks.readBurst = callback;
      break;
    case ADXL_NCB_READREGDIRECT:
      user_callbacks.readRegDirect = callback;
      break;
    case ADXL_NCB_READBURSTDIRECT:
      user_callbacks.readBurstDirect = callback;
      break;
    case ADXL_NCB_WRITEREG:
      user_callbacks.writeReg = callback;
      break;
    case ADXL_NCB_WRITEBURST:
      user_callbacks.writeBurst = callback;
      break;
    default:
      break;
  }
  return;
}

/*
 * uint8_t ADXL_GetX(uint8_t *pData);
 *    Get the most recent X acceleration value and store in 'pData' when finished.
 */
uint8_t ADXL_GetX(uint8_t *pData) {
  return _ADXL_ReadBurst(ADXL_REG_XDATA_PATCH, 2, pData);
}

/*
 * uint8_t ADXL_GetY(uint8_t *pData);
 *    Get the most recent Y acceleration value and store in 'pData' when finished.
 */
uint8_t ADXL_GetY(uint8_t *pData) {
  return _ADXL_ReadBurst(ADXL_REG_YDATA_PATCH, 2, pData);
}

/*
 * uint8_t ADXL_GetZ(uint8_t *pData);
 *    Get the most recent Z acceleration value and store in 'pData' when finished.
 */
uint8_t ADXL_GetZ(uint8_t *pData) {
  return _ADXL_ReadBurst(ADXL_REG_ZDATA_PATCH, 2, pData);
}

/* ========================= Writes with Allocator ========================== */

/*
 * uint8_t ADXL_ConfigDefault(void);
 *    Place the ADXL device into its (user-defined) default configuration using
 *    the ADXL_REG_..._DEFAULT macros for the corresponding register values.
 */
uint8_t ADXL_ConfigDefault(void) {
  adxl_dev.state = _ADXL_DEVICE_STATE_STBY;
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // Write to all applicable configuration registers in one sequential write.
  // FIFO_CTL, INT1_MAP, INT2_MAP, TIMING, MEASURE, and POWER_CTL registers (sequential)
  uint8_t data[6] = {ADXL_REG_FIFO_CTL_DEFAULT, 0x00, 0x00, ADXL_REG_TIMING_DEFAULT, ADXL_REG_MEASURE_DEFAULT, ADXL_REG_POWER_CTL_DEFAULT};
  return _ADXL_WriteBurstAlloc(ADXL_REG_FIFO_CTL, 6, data);
#else
  // Enable ADC, Place in Measure Mode, Enable External trigger
  uint8_t data[2] = {ADXL_REG_FILTER_CTL_DEFAULT, ADXL_REG_POWER_CTL_DEFAULT};
  return _ADXL_WriteBurstAlloc(ADXL_REG_FILTER_CTL, 2, data);
#endif
}

/*
 * uint8_t ADXL_HWReset(void);
 *    Trigger a hardware reset in the ADXL device by writing the reset code to
 *    the RESET register.
 */
uint8_t ADXL_HWReset(void) {
  return _ADXL_WriteRegAlloc(ADXL_REG_RESET, ADXL_REG_RESET_CODE);
}

/*
 * uint8_t ADXL_Standby(void);
 *    Place the ADXL device into standby mode
 */
uint8_t ADXL_Standby(void) {
  // PATCHED!
  // Set the power control reg to the default + standby mode (mask out whatever mode is in the default configuration).
  //return _ADXL_WriteRegAlloc(ADXL_REG_POWER_CTL, (ADXL_REG_POWER_CTL_DEFAULT & ~(ADXL_REG_POWER_CTL_MODE_MASK)) | ADXL_REG_POWER_CTL_MODE_STBY);
  return _ADXL_WriteRegAlloc(ADXL_REG_POWER_CTL, (adxl_reg.rPOWER_CTL & ~(ADXL_REG_POWER_CTL_MODE_MASK)) | ADXL_REG_POWER_CTL_MODE_STBY);
}

/*
 * uint8_t ADXL_ConfigFIFO(uint8_t fmt);
 *    Configure the onboard FIFO for recording the desired axes data.
 *    fmt must be one of ADXL_REG_FIFO_CTL_FMT_*
 */
uint8_t ADXL_ConfigFIFO(uint8_t fmt) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  //return _ADXL_WriteRegAlloc(ADXL_REG_FIFO_CTL, (fmt & ADXL_REG_FIFO_CTL_FMT_MASK) | (ADXL_REG_FIFO_CTL_DEFAULT & ~(ADXL_REG_FIFO_CTL_FMT_MASK)));
  return _ADXL_WriteRegAlloc(ADXL_REG_FIFO_CTL, (fmt & ADXL_REG_FIFO_CTL_FMT_MASK) | (adxl_reg.rFIFO_CTL & ~(ADXL_REG_FIFO_CTL_FMT_MASK)));
#else
  return 0; // TODO
#endif
}

/*
 * uint8_t ADXL_FullBWMode(void);
 *    Place the ADXL device into full bandwidth mode
 */
uint8_t ADXL_FullBWMode(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // Set the power control reg to the default + full BW mode (mask out whatever mode is in the default configuration).
  //return _ADXL_WriteRegAlloc(ADXL_REG_POWER_CTL, (ADXL_REG_POWER_CTL_DEFAULT & ~(ADXL_REG_POWER_CTL_MODE_MASK)) | ADXL_REG_POWER_CTL_MODE_FULL);
  return _ADXL_WriteRegAlloc(ADXL_REG_POWER_CTL, (adxl_reg.rPOWER_CTL & ~(ADXL_REG_POWER_CTL_MODE_MASK)) | ADXL_REG_POWER_CTL_MODE_FULL);
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_POWER_CTL, (adxl_reg.rPOWER_CTL & ~(ADXL_REG_POWER_CTL_MODE_MASK)) | ADXL_REG_POWER_CTL_MODE_MEAS);
#endif
}

/*
 * uint8_t ADXL_FIFOStreamMode(void);
 *    Place the FIFO into Stream mode.
 */
uint8_t ADXL_FIFOStreamMode(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // Set the FIFO to operate in Stream mode (most-recent N samples stored in FIFO)
  //return _ADXL_WriteRegAlloc(ADXL_REG_FIFO_CTL, (ADXL_REG_FIFO_CTL_DEFAULT & ~(ADXL_REG_FIFO_CTL_MODE_MASK)) | ADXL_REG_FIFO_CTL_MODE_STREAM);
  return _ADXL_WriteRegAlloc(ADXL_REG_FIFO_CTL, (adxl_reg.rFIFO_CTL & ~(ADXL_REG_FIFO_CTL_MODE_MASK)) | ADXL_REG_FIFO_CTL_MODE_STREAM);
#else
  return 0; // TODO. Does this apply to the ADXL363?
#endif
}

/*
 * uint8_t ADXL_SetOffsetX(uint8_t nOffset);
 *    Set offset associated with axis X.
 *    nOffset must be one of ADXL_REG_OFFSET_*
 */
uint8_t ADXL_SetOffsetX(uint8_t nOffset) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_OFFSET_X, (nOffset & ADXL_REG_OFFSET_MASK));
#else
  return 0; // Not implemented for ADXL363
#endif
}

/*
 * uint8_t ADXL_SetOffsetY(uint8_t nOffset);
 *    Set offset associated with axis Y.
 *    nOffset must be one of ADXL_REG_OFFSET_*
 */
uint8_t ADXL_SetOffsetY(uint8_t nOffset) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_OFFSET_Y, (nOffset & ADXL_REG_OFFSET_MASK));
#else
  return 0; // Not implemented for ADXL363
#endif
}

/*
 * uint8_t ADXL_SetOffsetZ(uint8_t nOffset);
 *    Set offset associated with axis Z.
 *    nOffset must be one of ADXL_REG_OFFSET_*
 */
uint8_t ADXL_SetOffsetZ(uint8_t nOffset) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_OFFSET_Z, (nOffset & ADXL_REG_OFFSET_MASK));
#else
  return 0; // Not implemented for ADXL363
#endif
}

uint8_t ADXL_SetODR(uint8_t ODR) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  /*return _ADXL_WriteRegAlloc(ADXL_REG_TIMING, (ADXL_REG_TIMING_DEFAULT & ~ADXL_REG_TIMING_ODR_MASK)\
                                              | (ODR & ADXL_REG_TIMING_ODR_MASK));*/
  return _ADXL_WriteRegAlloc(ADXL_REG_TIMING, (adxl_reg.rTIMING & ~ADXL_REG_TIMING_ODR_MASK)\
                                              | (ODR & ADXL_REG_TIMING_ODR_MASK));
#else
  /*return _ADXL_WriteRegAlloc(ADXL_REG_FILTER_CTL, (ADXL_REG_FILTER_CTL_DEFAULT & ~ADXL_REG_FILTER_CTL_ODR_MASK)\
                                                    | (ODR & ADXL_REG_FILTER_CTL_ODR_MASK));*/
  return _ADXL_WriteRegAlloc(ADXL_REG_FILTER_CTL, (adxl_reg.FILTER_CTL & ~ADXL_REG_FILTER_CTL_ODR_MASK)\
                                                  | (ODR & ADXL_REG_FILTER_CTL_ODR_MASK));
#endif
}

uint8_t ADXL_ExtTrigEn(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  //return _ADXL_WriteRegAlloc(ADXL_REG_TIMING, (ADXL_REG_TIMING_DEFAULT | ADXL_REG_TIMING_EXT_SYNC_EN));
  return _ADXL_WriteRegAlloc(ADXL_REG_TIMING, (adxl_reg.rTIMING | ADXL_REG_TIMING_EXT_SYNC_EN));
#else
  //return _ADXL_WriteRegAlloc(ADXL_REG_FILTER_CTL, (ADXL_REG_FILTER_CTL_DEFAULT | ADXL_REG_FILTER_CTL_EXT_SAMPLE_EN));
  return _ADXL_WriteRegAlloc(ADXL_REG_FILTER_CTL, (adxl_reg.rFILTER_CTL | ADXL_REG_FILTER_CTL_EXT_SAMPLE_EN));
#endif
}

uint8_t ADXL_ExtTrigDis(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  //return _ADXL_WriteRegAlloc(ADXL_REG_TIMING, (ADXL_REG_TIMING_DEFAULT & ~ADXL_REG_TIMING_EXT_SYNC_EN));
  return _ADXL_WriteRegAlloc(ADXL_REG_TIMING, (adxl_reg.rTIMING & ~ADXL_REG_TIMING_EXT_SYNC_EN));
#else
  //return _ADXL_WriteRegAlloc(ADXL_REG_FILTER_CTL, (ADXL_REG_FILTER_CTL_DEFAULT & ~ADXL_REG_FILTER_CTL_EXT_SAMPLE_EN));
  return _ADXL_WriteRegAlloc(ADXL_REG_FILTER_CTL, (adxl_reg.rFILTER_CTL & ~ADXL_REG_FILTER_CTL_EXT_SAMPLE_EN));
#endif
}

uint8_t ADXL_EnXActThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_ACT_X_L, (adxl_reg.rTHRESH_ACT_X_L | ADXL_REG_THRESH_ACT_X_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL | ADXL_REG_ACT_INACT_CTL_ACT_EN));
#endif
}

uint8_t ADXL_DisXActThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_ACT_X_L, (adxl_reg.rTHRESH_ACT_X_L & ~ADXL_REG_THRESH_ACT_X_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL & ~ADXL_REG_ACT_INACT_CTL_ACT_EN));
#endif
}

uint8_t ADXL_EnYActThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_ACT_Y_L, (adxl_reg.rTHRESH_ACT_Y_L | ADXL_REG_THRESH_ACT_Y_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL | ADXL_REG_ACT_INACT_CTL_ACT_EN));
#endif
}

uint8_t ADXL_DisYActThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_ACT_Y_L, (adxl_reg.rTHRESH_ACT_Y_L & ~ADXL_REG_THRESH_ACT_Y_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL & ~ADXL_REG_ACT_INACT_CTL_ACT_EN));
#endif
}

uint8_t ADXL_EnZActThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_ACT_Z_L, (adxl_reg.rTHRESH_ACT_Z_L | ADXL_REG_THRESH_ACT_Z_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL | ADXL_REG_ACT_INACT_CTL_ACT_EN));
#endif
}

uint8_t ADXL_DisZActThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_ACT_Z_L, (adxl_reg.rTHRESH_ACT_Z_L & ~ADXL_REG_THRESH_ACT_Z_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL & ~ADXL_REG_ACT_INACT_CTL_ACT_EN));
#endif
}

uint8_t ADXL_EnXInactThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_INACT_X_L, (adxl_reg.rTHRESH_INACT_X_L | ADXL_REG_THRESH_INACT_X_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL | ADXL_REG_ACT_INACT_CTL_INACT_EN));
#endif
}

uint8_t ADXL_DisXInactThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_INACT_X_L, (adxl_reg.rTHRESH_INACT_X_L & ~ADXL_REG_THRESH_INACT_X_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL & ~ADXL_REG_ACT_INACT_CTL_INACT_EN));
#endif
}

uint8_t ADXL_EnYInactThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_INACT_Y_L, (adxl_reg.rTHRESH_INACT_Y_L | ADXL_REG_THRESH_INACT_Y_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL | ADXL_REG_ACT_INACT_CTL_INACT_EN));
#endif
}

uint8_t ADXL_DisYInactThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_INACT_Y_L, (adxl_reg.rTHRESH_INACT_Y_L & ~ADXL_REG_THRESH_INACT_Y_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL & ~ADXL_REG_ACT_INACT_CTL_INACT_EN));
#endif
}

uint8_t ADXL_EnZInactThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_INACT_Z_L, (adxl_reg.rTHRESH_INACT_Z_L | ADXL_REG_THRESH_INACT_Z_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL | ADXL_REG_ACT_INACT_CTL_INACT_EN));
#endif
}

uint8_t ADXL_DisZInactThresh(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_WriteRegAlloc(ADXL_REG_THRESH_INACT_Z_L, (adxl_reg.rTHRESH_INACT_Z_L & ~ADXL_REG_THRESH_INACT_Z_EN));
#else
  return _ADXL_WriteRegAlloc(ADXL_REG_ACT_INACT_CTL, (adxl_reg.rACT_INACT_CTL & ~ADXL_REG_ACT_INACT_CTL_INACT_EN));
#endif
}

/*
 * uint8_t ADXL_SetXActThresh(uint16_t threshold);
 *    ADXL363: threshold is set in codes where a code is dependent on the measurement range:
 *      Range Code:
 *      2g    1mg
 *      4g    2mg
 *      8g    4mg
 *    ADXL372: threshold is set in codes where a code is 100mg
 */
uint8_t ADXL_SetXActThresh(uint16_t threshold) {
  // Patched!
  uint8_t data[2];
  ADXL_THRESHOLD_UNPACK_UINT16(threshold, data);
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // data[1] bit 0 is ACT_X_EN and should not be modified
  // data[1] bit 1 is ACT_REF and should not be modified
  data[1] = (data[1] & ~(ADXL_REG_THRESH_ACT_X_EN | ADXL_REG_THRESH_ACT_REF_REF)) | \
            (adxl_reg.rTHRESH_INACT_X_L & (ADXL_REG_THRESH_ACT_X_EN | ADXL_REG_THRESH_ACT_REF_REF));
#endif
  ADXL_DBG("data[0:1] = 0x%x 0x%x\r\n", data[0], data[1]);
  return _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_ACT_X_PATCH, 2, data);
}

uint8_t ADXL_SetYActThresh(uint16_t threshold) {
  // Patched!
  uint8_t data[2];
  ADXL_THRESHOLD_UNPACK_UINT16(threshold, data);
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // data[1] bit 0 is ACT_Y_EN and should not be modified
  data[1] = (data[1] & ~ADXL_REG_THRESH_ACT_Y_EN) | (adxl_reg.rTHRESH_ACT_Y_L & ADXL_REG_THRESH_ACT_Y_EN);
#endif
  return _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_ACT_Y_PATCH, 2, data);
}

uint8_t ADXL_SetZActThresh(uint16_t threshold) {
  // Patched!
  uint8_t data[2];
  ADXL_THRESHOLD_UNPACK_UINT16(threshold, data);
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // data[1] bit 0 is ACT_Z_EN and should not be modified
  data[1] = (data[1] & ~ADXL_REG_THRESH_ACT_Z_EN) | (adxl_reg.rTHRESH_ACT_Z_L & ADXL_REG_THRESH_ACT_Z_EN);
#endif
  return _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_ACT_Z_PATCH, 2, data);
}

uint8_t ADXL_SetActThreshTime(uint8_t time) {
  // Patched!  Make sure to calculate the time based on the ODR
  return _ADXL_WriteRegAlloc(ADXL_REG_TIME_ACT, time);
}

uint8_t ADXL_SetXInactThresh(uint16_t threshold) {
  // Patched!
  uint8_t data[2];
  ADXL_THRESHOLD_UNPACK_UINT16(threshold, data);
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // data[1] bit 0 is INACT_X_EN and should not be modified
  // data[1] bit 1 is INACT_REF and should not be modified
  data[1] = (data[1] & ~(ADXL_REG_THRESH_INACT_X_EN | ADXL_REG_THRESH_INACT_REF_REF)) | \
            (adxl_reg.rTHRESH_INACT_X_L & (ADXL_REG_THRESH_INACT_X_EN | ADXL_REG_THRESH_INACT_REF_REF));
#endif
  ADXL_DBG("data[0:1] = 0x%x 0x%x\r\n", data[0], data[1]);
  return _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_INACT_X_PATCH, 2, data);
}

uint8_t ADXL_SetYInactThresh(uint16_t threshold) {
  // Patched!
  uint8_t data[2];
  ADXL_THRESHOLD_UNPACK_UINT16(threshold, data);
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // data[1] bit 0 is INACT_Y_EN and should not be modified
  data[1] = (data[1] & ~ADXL_REG_THRESH_INACT_Y_EN) | (adxl_reg.rTHRESH_INACT_Y_L & ADXL_REG_THRESH_INACT_Y_EN);
#endif
  return _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_INACT_Y_PATCH, 2, data);
}

uint8_t ADXL_SetZInactThresh(uint16_t threshold) {
  // Patched!
  uint8_t data[2];
  ADXL_THRESHOLD_UNPACK_UINT16(threshold, data);
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // data[1] bit 0 is INACT_Z_EN and should not be modified
  data[1] = (data[1] & ~ADXL_REG_THRESH_INACT_Z_EN) | (adxl_reg.rTHRESH_INACT_Z_L & ADXL_REG_THRESH_INACT_Z_EN);
#endif
  return _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_INACT_Z_PATCH, 2, data);
}

uint8_t ADXL_SetInactThreshTime(uint16_t time) {
  // Patched!  Make sure to calculate the time based on the ODR
  uint8_t data[2];
  ADXL_INACTIVITY_UNPACK_UINT16(time, data);
  //ADXL_DBG("data[0:1] = 0x%x 0x%x\r\n", data[0], data[1]);
  return _ADXL_WriteBurstAlloc(ADXL_REG_TIME_INACT_PATCH, 2, data);
}

uint8_t ADXL_ReadDeviceIDs(uint8_t *pData) {
  return _ADXL_ReadBurst(ADXL_REG_DEVID_AD, 4, pData);
}

/* ======================== Operating Mode Settings ========================= */

uint8_t ADXL_SetModeGDetect(void) {
  if (adxl_dev.state == _ADXL_DEVICE_STATE_GDETECT) {
    // Device is already in the GDetect state (supposedly)
    return ADXL_QUEUE_OK;
  }
  adxl_dev.state = _ADXL_DEVICE_STATE_GDETECT;
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  uint8_t data[] = {
    0, //ADXL_REG_THRESH_ACT_X_H
    0, //ADXL_REG_THRESH_ACT_X_L
    0, //ADXL_REG_THRESH_ACT_Y_H
    0, //ADXL_REG_THRESH_ACT_Y_L
    0, //ADXL_REG_THRESH_ACT_Z_H
    0, //ADXL_REG_THRESH_ACT_Z_L
    1, //ADXL_REG_TIME_ACT
    0, //ADXL_REG_THRESH_INACT_X_H
    0, //ADXL_REG_THRESH_INACT_X_L
    0, //ADXL_REG_THRESH_INACT_Y_H
    0, //ADXL_REG_THRESH_INACT_Y_L
    0, //ADXL_REG_THRESH_INACT_Z_H
    0, //ADXL_REG_THRESH_INACT_Z_L
    0, //ADXL_REG_TIME_INACT_H
    0  //ADXL_REG_TIME_INACT_L
  };
#if ADXL_TARGET_AXIS == _ADXL_AXIS_X
  ADXL_THRESHOLD_UNPACK_UINT16(ADXL_ACT_THRESHOLD, &data[0]);  // ACT Threshold
  data[1] |= ADXL_REG_THRESH_ACT_X_EN;         // Enable X action threshold
  ADXL_THRESHOLD_UNPACK_UINT16(ADXL_INACT_THRESHOLD, &data[7]); // INACT Threshold
  data[8] |= ADXL_REG_THRESH_INACT_X_EN;      // Enable X inaction threshold
#elif ADXL_TARGET_AXIS == _ADXL_AXIS_Y
  ADXL_THRESHOLD_UNPACK_UINT16(ADXL_ACT_THRESHOLD, &data[2]);  // ACT Threshold
  data[3] |= ADXL_REG_THRESH_ACT_Y_EN;         // Enable Y action threshold
  ADXL_THRESHOLD_UNPACK_UINT16(ADXL_INACT_THRESHOLD, &data[9]); // INACT Threshold
  data[10] |= ADXL_REG_THRESH_INACT_Y_EN;      // Enable Y inaction threshold
#else   // Assume _ADXL_AXIS_Z
  ADXL_THRESHOLD_UNPACK_UINT16(ADXL_ACT_THRESHOLD, &data[4]);  // ACT Threshold
  data[5] |= ADXL_REG_THRESH_ACT_Z_EN;         // Enable Z action threshold
  ADXL_THRESHOLD_UNPACK_UINT16(ADXL_INACT_THRESHOLD, &data[11]); // INACT Threshold
  data[12] |= ADXL_REG_THRESH_INACT_Z_EN;      // Enable Z inaction threshold
#endif  // ADXL_TARGET_AXIS
  uint8_t result = _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_ACT_X_H, 15, data);
  if (result != ADXL_QUEUE_OK) {
    return result;
  }
  // Might want to reduce bandwidth in this mode.
  data[0] = ADXL_REG_FIFO_CTL_MODE_DIS | ADXL_FIFO_FMT | ADXL_REG_FIFO_CTL_FIFO_SAMPLES_H; // ADXL_REG_FIFO_CTL
  data[1] = ADXL_REG_INT1_MAP_ACT;              //ADXL_REG_INT1_MAP
  data[2] = 0;                                  //ADXL_REG_INT2_MAP
  data[3] = ADXL_REG_TIMING_ODR_3200;           //ADXL_REG_TIMING
  data[4] = ADXL_REG_MEASURE_LOW_NOISE | ADXL_REG_MEASURE_BW_400 | ADXL_REG_MEASURE_LINKLOOP_LOOPED; //ADXL_REG_MEASURE
  data[5] = ADXL_REG_POWER_CTL_MODE_FULL | ADXL_REG_POWER_CTL_FILTER_SETTLE | ADXL_REG_POWER_CTL_HPF_DIS | ADXL_REG_POWER_CTL_LPF_DIS; //ADXL_REG_POWER_CTL
  return _ADXL_WriteBurstAlloc(ADXL_REG_FIFO_CTL, 6, data);
#else // ADXL_TARGET = ADXL_TARGET_ADXL363
  // All the registers are sequential, so let's do this in one write
  uint8_t data[] = {
    0,  //ADXL_REG_THRESH_ACT_L
    0,  //ADXL_REG_THRESH_ACT_H
    1,  //ADXL_REG_TIME_ACT
    0,  //ADXL_REG_THRESH_INACT_L
    0,  //ADXL_REG_THRESH_INACT_H
    0,  //ADXL_REG_TIME_INACT_L
    0,  //ADXL_REG_TIME_INACT_H
    ADXL_REG_ACT_INACT_CTL_ACT_EN | ADXL_REG_ACT_INACT_CTL_INACT_EN | ADXL_REG_ACT_INACT_CTL_LINKLOOP_LOOP,//ADXL_REG_ACT_INACT_CTL
  };
  ADXL_THRESHOLD_UNPACK_UINT16(ADXL_ACT_THRESHOLD, &data[0]); // ACT Threshold
  ADXL_THRESHOLD_UNPACK_UINT16(ADXL_INACT_THRESHOLD, &data[3]); // INACT Threshold
  //ADXL_INACTIVITY_UNPACK_UINT16(0, &data[5]); // INACT Time
  //printf("data[0:6] = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
  uint8_t result = _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_ACT_L, 7, data);
  if (result != ADXL_QUEUE_OK) {
    return result;
  }
  data[0] = ADXL_REG_INTMAP1_ACT; //ADXL_REG_INTMAP1
  data[1] = 0; //ADXL_REG_INTMAP2
  data[2] = ADXL_REG_FILTER_CTL_ODR_400HZ | ADXL_REG_FILTER_CTL_RANGE_8G; //ADXL_REG_FILTER_CTL
  data[3] = ADXL_REG_POWER_CTL_MODE_MEAS | ADXL_REG_POWER_CTL_LOW_NOISE_ULOW; //ADXL_REG_POWER_CTL
  //printf("data[0:3] = 0x%x 0x%x 0x%x 0x%x\r\n", data[0], data[1], data[2], data[3]);
  return _ADXL_WriteBurstAlloc(ADXL_REG_INTMAP1, 4, data);
#endif // ADXL_TARGET == ADXL_TARGET_ADXL372
}

uint8_t ADXL_SetModeDAQ(void) {
  /*
  // Let's bypass this check for now and force the message.
  // Alternatively we could register a callback and set the state parameter in the callback
  // when we're pretty certain the message transmission was successful.
  if (adxl_dev.state == _ADXL_DEVICE_STATE_DAQ) {
    // Device is already in the DAQ state (supposedly)
    return ADXL_QUEUE_OK;
  }
  */
  adxl_dev.state = _ADXL_DEVICE_STATE_DAQ;
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // Disable all activity and inactivity detection, leaving everything else unchanged
  // This can be shortened depending on the axis of sensitivity
  uint8_t data[] = {
    adxl_reg.rTHRESH_ACT_X_L & ~ADXL_REG_THRESH_ACT_X_EN, // THRESH_ACT_X_L
    adxl_reg.rTHRESH_ACT_Y_H,  // THRESH_ACT_Y_H
    adxl_reg.rTHRESH_ACT_Y_L & ~ADXL_REG_THRESH_ACT_Y_EN, // THRESH_ACT_Y_L
    adxl_reg.rTHRESH_ACT_Z_H,  // THRESH_ACT_Z_H
    adxl_reg.rTHRESH_ACT_Z_L & ~ADXL_REG_THRESH_ACT_Z_EN, // THRESH_ACT_Z_L
    adxl_reg.rTIME_ACT,
    adxl_reg.rTHRESH_INACT_X_H,
    adxl_reg.rTHRESH_INACT_X_L & ~ADXL_REG_THRESH_INACT_X_EN, // THRESH_INACT_X_L
    adxl_reg.rTHRESH_INACT_Y_H,
    adxl_reg.rTHRESH_INACT_Y_L & ~ADXL_REG_THRESH_INACT_Y_EN, // THRESH_INACT_Y_L
    adxl_reg.rTHRESH_INACT_Z_H,
    adxl_reg.rTHRESH_INACT_Z_L & ~ADXL_REG_THRESH_INACT_Z_EN // THRESH_INACT_Z_L
  };
  uint8_t r = _ADXL_WriteBurstAlloc(ADXL_REG_THRESH_ACT_X_L, 12, data);
  if (r != ADXL_QUEUE_OK) {
    ADXL_DBG("returned %d\r\n", r);
  }
  data[0] = 0xFF; // FIFO_SAMPLES - set to maximum
  data[1] = ADXL_REG_FIFO_CTL_MODE_OLDEST | ADXL_FIFO_FMT | ADXL_REG_FIFO_CTL_FIFO_SAMPLES_H; // FIFO_CTL
  data[2] = 0; // INT1_MAP
  data[3] = 0; // INT2_MAP
  data[4] = ADXL_REG_TIMING_ODR_6400 | ADXL_REG_TIMING_EXT_SYNC_EN; // TIMING
  data[5] = ADXL_REG_MEASURE_BW_3200 | ADXL_REG_MEASURE_LOW_NOISE; // MEASURE
  data[6] = ADXL_REG_POWER_CTL_FILTER_SETTLE | ADXL_REG_POWER_CTL_LPF_DIS | ADXL_REG_POWER_CTL_HPF_DIS | ADXL_REG_POWER_CTL_MODE_FULL; // POWER_CTL
  return _ADXL_WriteBurstAlloc(ADXL_REG_FIFO_SAMPLES, 7, data);
#else
  return 0; // TODO
#endif  // ADXL_TARGET == ADXL_TARGET_ADXL372
}

uint8_t ADXL_SetModeStandby(void) {
  // Patched!
  adxl_dev.state = _ADXL_DEVICE_STATE_STBY;
  uint8_t pwrctl = (adxl_reg.rPOWER_CTL & ~ADXL_REG_POWER_CTL_MODE_MASK) | ADXL_REG_POWER_CTL_MODE_STBY;
  return _ADXL_WriteReg(ADXL_REG_POWER_CTL, &pwrctl);
}

/* ============================= Blocking Reads ============================= */
/*
 * uint8_t ADXL_GetFIFOFill(uint8_t *pData);
 *    Get the number of entries waiting in the onboard FIFO buffer and store in
 *    pData.  A 10-bit number is written to pData as two bytes, the order of
 *    which is target-specific. Use the macro ADXL_ENTRIES_PACK_UINT16(pData)
 *    to convert the read bytes to a uint16_t in a target agnostic manner.
 *    'pData' must have room for at least 2 bytes.
 */
uint8_t ADXL_GetFIFOFill(uint8_t *pData) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_ReadBurstBlocking(ADXL_REG_FIFO_ENTRIES_PATCH, 2, pData);
#else
  // We can actually use the above patched function for both the ADXL363 and 372
  // if we actually wanted to read the FIFO fill, but since we're faking a FIFO
  // for the 363, this function is redirected to the below.
  // Need to place the adc fifo nEntries (adxl_adc_fifo.nEntries) into pData
  _ADXL363_ENTRIES_UNPACK_INT16(adxl_adc_fifo.nEntries, pData);
  // Afterward, we can call _ADXL_DATA_PACK_INT16(pData) to extract the int16
  // again. Convoluted, but consistent with the call to the actual FIFO.
  return 0;
#endif
}

uint8_t ADXL_ReadDeviceIDsBlock(uint8_t *pData) {
  return _ADXL_ReadBurstBlocking(ADXL_REG_DEVID_AD, 4, pData);
}

/* ============================ Direct Mode Reads =========================== */
/*
 * uint8_t ADXL_ReadFIFO(uint32_t n, uint8_t *pData);
 *    Read 'n' samples of acceleration data from the ADXL FIFO and store at pointer
 *    'pData'.  Note!  Each sample requires 2 bytes, so pData must be at least 2n long.
 *    Note! Uses direct mode so pData[0] will be garbage.
 */
uint8_t ADXL_ReadFIFO(uint32_t n, uint8_t *pData) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  return _ADXL_ReadBurstDirect(ADXL_REG_FIFO_DATA, 2*n, pData);
#else
  return ADXL_ReadFIFO_363(n, pData);
#endif
}

/* ========================= Low-Level API Functions ======================== */
/*
 * uint8_t ADXL_QueueNext(void);
 *    Returns ADXL_QUEUE_EMPTY if the queue is empty, otherwise
 *    returns ADXL_QUEUE_OK.
 *    This is similar to the behavior of ADXL_QueueGet() but it
 *    does not shift the queue if it is non-empty, giving the
 *    calling context notice to prepare to receive data.
 */
uint8_t ADXL_QueueNext(void) {
  if (adxl_queue.pIn == adxl_queue.pOut) {
    // If the indices are equal and we are NOT full, we must be empty.
    if (adxl_queue.full == 0) {
      return ADXL_QUEUE_EMPTY;
    }
  }
  return ADXL_QUEUE_OK;
}

/*
 * uint8_t ADXL_QueueGet(volatile queueItem_t *item);
 *    Get the next item from the Queue (store into location pointed to by 'item')
 *    Copy FIRST, then increment.
 *    Return ADXL_QUEUE_EMPTY if queue is empty, otherwise ADXL_QUEUE_OK
 */
uint8_t ADXL_QueueGet(volatile queueItem_t *item) {
  // Check for empty queue
  if (adxl_queue.pIn == adxl_queue.pOut) {
    // If the indices are equal and we are NOT full, we must be empty.
    if (adxl_queue.full == 0) {
      return ADXL_QUEUE_EMPTY;
    }
  }
  // Copy data from the queue to 'item'
  for (int n = 0; n < sizeof(queueItem_t); n++) {
    *((uint8_t *)item + n) = *((uint8_t *)&(adxl_queue.queue[adxl_queue.pOut]) + n);
  }
  // Increment pOut index (wrap at boundary)
  if (adxl_queue.pOut == ADXL_MAX_QUEUE_ITEMS - 1) {
    adxl_queue.pOut = 0;
  } else {
    adxl_queue.pOut++;
  }
  // Clear full queue condition (regardless of whether it is necessary)
  adxl_queue.full = 0;
  return ADXL_QUEUE_OK;
}

/*
 * uint8_t _ADXL_ReadReg(uint8_t nReg, uint8_t *pData);
 *    Add a single byte register read from 'nReg' to the queue, reading the
 *    register data into pointer 'pData'
 */
static uint8_t _ADXL_ReadReg(uint8_t nReg, uint8_t *pData) {
  return _ADXL_Read(nReg, 1, pData, ADXL_QUEUEITEM_INDIRECT, ADXL_CallbackReadReg);
}

/*
 * static uint8_t _ADXL_ReadBurst(uint8_t nRegStart, uint32_t nReads, uint8_t *pData);
 *    Add a burst read to the queue starting from reg 'nRegStart' and reading
 *    'nReads' bytes into pointer 'pData'
 */
static uint8_t _ADXL_ReadBurst(uint8_t nRegStart, uint32_t nReads, uint8_t *pData) {
  return _ADXL_Read(nRegStart, nReads, pData, ADXL_QUEUEITEM_INDIRECT, ADXL_CallbackReadBurst);
}

/*
 * uint8_t _ADXL_ReadRegDirect(uint8_t nReg, uint8_t *pData);
 *    Add a single byte register read from 'nReg' to the queue, reading the
 *    register data into pointer 'pData'
 *    Note! This uses direct mode, which means the length of 'pData' must be >= nReads + 1
 *    and the first byte read into 'pData' is garbage.
 */
static uint8_t _ADXL_ReadRegDirect(uint8_t nReg, uint8_t *pData) {
  return _ADXL_Read(nReg, 1, pData, ADXL_QUEUEITEM_DIRECT, ADXL_CallbackReadRegDirect);
}

/*
 * static uint8_t _ADXL_ReadBurstDirect(uint8_t nRegStart, uint32_t nReads, uint8_t *pData);
 *    Add a burst read to the queue starting from reg 'nRegStart' and reading
 *    'nReads' bytes into pointer 'pData'
 *    Note! This uses direct mode, which means the length of 'pData' must be >= nReads + 1
 *    and the first byte read into 'pData' is garbage.
 */
static uint8_t _ADXL_ReadBurstDirect(uint8_t nRegStart, uint32_t nReads, uint8_t *pData) {
  return _ADXL_Read(nRegStart, nReads, pData, ADXL_QUEUEITEM_DIRECT, ADXL_CallbackReadBurstDirect);
}

/*
 * static uint8_t _ADXL_ReadRegBlocking(uint8_t nReg, uint8_t *pData);
 *    Read a single byte from register 'nReg' in blocking mode (meaning bypass the queueing
 *    system and immediately start the transaction, blocking until the data is received).
 *    If commands in the queue are being processed, this function waits until the current
 *    command is finished, then takes over the SPI peripheral - pausing the queue.  If the
 *    queued command does not finish in time, it will time out according to SPI_BLOCKING_TIMEOUT
 *    (defined in loop cycles) and return SPI_BLOCKING_READ_TIMEOUT.
 *    Otherwise, returns SPI_BLOCKING_READ_SUCCESS.
 */
static uint8_t _ADXL_ReadRegBlocking(uint8_t nReg, uint8_t *pData) {
  return SPI_BlockingReadOne((nReg << 1) | ADXL_TADDR_READ, pData);
}

/*
 * static uint8_t _ADXL_ReadBurstBlocking(uint8_t nRegStart, uint8_t nReads, uint8_t *pData);
 *    Read 'nReads' bytes starting from register 'nReg' in blocking mode (meaning bypass the
 *    queueing system and immediately start the transaction, blocking until the data is received).
 *    If commands in the queue are being processed, this function waits until the current
 *    command is finished, then takes over the SPI peripheral - pausing the queue.  If the
 *    queued command does not finish in time, it will time out according to SPI_BLOCKING_TIMEOUT
 *    (defined in loop cycles) and return SPI_BLOCKING_READ_TIMEOUT.
 *    Otherwise, returns SPI_BLOCKING_READ_SUCCESS.
 *    Note: if 'nReads' is 0, immediately returns SPI_BLOCKING_READ_ZEROBYTES.
 */
static uint8_t _ADXL_ReadBurstBlocking(uint8_t nRegStart, uint8_t nReads, uint8_t *pData) {
  return SPI_BlockingRead((nRegStart << 1) | ADXL_TADDR_READ, nReads, pData);
}

/*
 * static uint8_t _ADXL_WriteReg(uint8_t nReg, uint8_t *pData);
 *    Add a single byte register write to 'nReg' to the queue, writing the
 *    register data with contents from pointer 'pData'
 */
static uint8_t _ADXL_WriteReg(uint8_t nReg, uint8_t *pData) {
  return _ADXL_Write(nReg, 1, pData, ADXL_CallbackWriteReg);
}

/*
 * static uint8_t _ADXL_WriteBurst(uint8_t nRegStart, uint32_t nWrites, uint8_t *pData);
 *    Add a multi-byte register write from 'nRegStart' to 'nRegStart' + nWrites to the
 *    queue, using contents from pointer 'pData' as the data to write (in order).
 */
static uint8_t _ADXL_WriteBurst(uint8_t nRegStart, uint32_t nWrites, uint8_t *pData) {
  return _ADXL_Write(nRegStart, nWrites, pData, ADXL_CallbackWriteBurst);
}

/*
 * static uint8_t _ADXL_WriteRegAlloc(uint8_t nReg, uint8_t val);
 *    NOTE! This uses 1 byte from the FIFO allocator and frees the byte at callback.
 *    Add a single byte register write to 'nReg' to the queue, setting the
 *    register data to 'val'
 */
static uint8_t _ADXL_WriteRegAlloc(uint8_t nReg, uint8_t val) {
  uint8_t index = _alloc_add(1);
  if (index != ALLOC_NOSPACE) {
    ALLOC_INSERT(index, val);
    return _ADXL_Write(nReg, 1, ALLOC_GET_POINTER(index), ADXL_CallbackWriteRegAlloc);
  } else {
    ADXL_DBG("No Space!\r\n");
    return ADXL_ALLOC_NOSPACE;
  }
}

/*
 * static uint8_t _ADXL_WriteBurstAlloc(uint8_t nRegStart, uint8_t nWrites, uint8_t *pData);
 *    NOTE! This allocates 'nWrites' bytes from the FIFO allocator and frees at callback.
 *    Thus, it can be used with variables on the stack since they get copied before they
 *    go out of context.
 *    Add a multi-byte register write from 'nRegStart' to 'nRegStart' + nWrites to the
 *    queue, using contents from pointer 'pData' as the data to write (in order).
 */
static uint8_t _ADXL_WriteBurstAlloc(uint8_t nRegStart, uint8_t nWrites, uint8_t *pData) {
  uint8_t index = _alloc_add(nWrites);
  if (index != ALLOC_NOSPACE) {
    for (int n = 0; n < nWrites; n++) {
      ALLOC_INSERT(index + n, pData[n]);
    }
    return _ADXL_Write(nRegStart, (uint32_t)nWrites, ALLOC_GET_POINTER(index), ADXL_CallbackWriteBurstAlloc);
  } else {
    ADXL_DBG("No Space!\r\n");
    return ADXL_ALLOC_NOSPACE;
  }
}

static inline void _ADXL_CopyRData(void) {
  SPI_GetRData(adxl_activeQueueItem.pdata, adxl_activeQueueItem.datalen, ADXL_HEADER_OFFSET);
  return;
}

/*
 * static void _ADXL_UpdateLocal(uint8_t nRegStart, int nWrites, uint8_t *pData);
 *    Update the local (RAM) copy of the contiguous writable device registers starting
 *    from nRegStart to nRegStart + nWrites.
 */
static void _ADXL_UpdateLocal(uint8_t nRegStart, int nWrites, uint8_t *pData) {
  if (nRegStart < _ADXL_REG_START_OFFSET) {
    ADXL_DBG("Updating fail.\r\n");
    return;
  }
  int offset = nRegStart - _ADXL_REG_START_OFFSET;
  for (int n = 0; n < nWrites; n++) {
    if ((offset + n) > _ADXL_REG_END_OFFSET) {
      break;
    }
    *((uint8_t *)&adxl_reg + offset + n) = pData[n];
  }
  /*
  uint32_t *pReg = (uint32_t *)&adxl_reg;
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  // 32 registers
  if (nRegStart > _ADXL_REG_END_OFFSET) {
    return;
  }
  ADXL_DBG("adxl_reg 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx\r\n", *(pReg), *(pReg+1), *(pReg+2), *(pReg+3),\
           *(pReg+4), *(pReg+5), *(pReg+6), *(pReg+7));
#else
  // 14 registers
  if (nRegStart > _ADXL_REG_END_OFFSET) {
    return;
  }
  ADXL_DBG("adxl_reg 0x%lx 0x%lx 0x%lx 0x%x 0x%x\r\n", *(pReg), *(pReg+1), *(pReg+2), adxl_reg.rFILTER_CTL, adxl_reg.rPOWER_CTL);
#endif
  */
  return;
}

void ADXL_CallbackReadRegDirect(void) {
  if (user_callbacks.readRegDirect != ADXL_PNULL) {
    user_callbacks.readRegDirect();
  }
  return;
}

void ADXL_CallbackReadBurstDirect(void) {
  if (user_callbacks.readBurstDirect != ADXL_PNULL) {
    user_callbacks.readBurstDirect();
  }
  return;
}

void ADXL_CallbackReadReg(void) {
  // Copy read data
  _ADXL_CopyRData();
#if ADXL_INCLUDE_TESTS
  // Testing
  testCallbackReadReg();
#endif
  if (user_callbacks.readReg != ADXL_PNULL) {
    user_callbacks.readReg();
  }
  return;
}

void ADXL_CallbackReadBurst(void) {
  // Copy read data
  _ADXL_CopyRData();
#if ADXL_INCLUDE_TESTS
  // Testing
  testCallbackReadBurst();
#endif
  if (user_callbacks.readBurst != ADXL_PNULL) {
    user_callbacks.readBurst();
  }
  return;
}

void ADXL_CallbackWriteReg(void) {
  // Testing
#if ADXL_INCLUDE_TESTS
  testCallbackWriteReg();
#endif
  if (user_callbacks.writeReg != ADXL_PNULL) {
    user_callbacks.writeReg();
  }
  return;
}

void ADXL_CallbackWriteRegAlloc(void) {
  _alloc_free();
  ADXL_CallbackWriteReg();
  return;
}

void ADXL_CallbackWriteBurst(void) {
#if ADXL_INCLUDE_TESTS
  // Testing
  testCallbackWriteBurst();
#endif
  if (user_callbacks.writeBurst != ADXL_PNULL) {
    user_callbacks.writeBurst();
  }
  return;
}

void ADXL_CallbackWriteBurstAlloc(void) {
  _alloc_free();
  ADXL_CallbackWriteBurst();
  return;
}

/* ====================== Static Function Definitions ======================= */

static void ADXL_QueueInit(void) {
  adxl_queue.pIn = 0;
  adxl_queue.pOut = 0;
  adxl_queue.full = 0;
  return;
}

/*
 * static uint8_t _ADXL_Read(uint8_t nRegStart, uint32_t nReads, uint8_t *pData, void (*callback)(void));
 *    This duplicates the queue management of ADXL_QueueAdd() which
 *    sacrifices encapsulation for efficient memory management.
 */
static uint8_t _ADXL_Read(uint8_t nRegStart, uint32_t nReads, uint8_t *pData, uint8_t direct, void (*callback)(void)) {
  // Check for full queue
  if (adxl_queue.full) {
    return ADXL_QUEUE_FULL;
  }
  // Fill next queue item with command data
  adxl_queue.queue[adxl_queue.pIn].taddr = ADXL_PACK_TADDR_READ(nRegStart);//(uint8_t)(((nRegStart << 1) | ADXL_TADDR_READ) & 0xFF);
  adxl_queue.queue[adxl_queue.pIn].datalen = nReads;
  adxl_queue.queue[adxl_queue.pIn].direct = (direct & ADXL_QUEUEITEM_DIRECT_BM);
  adxl_queue.queue[adxl_queue.pIn].pdata = pData;
  adxl_queue.queue[adxl_queue.pIn].callback = callback;
  // Increment pIn index (wrap at boundary)
  if (adxl_queue.pIn == ADXL_MAX_QUEUE_ITEMS - 1) {
    adxl_queue.pIn = 0;
  } else {
    adxl_queue.pIn++;
  }
  // Check for full queue condition
  // If adding an item results in equal indices, we are full
  if (adxl_queue.pIn == adxl_queue.pOut) {
    adxl_queue.full = 1;
  }
  return ADXL_QUEUE_OK;
}

/*
 * static uint8_t _ADXL_Write(uint8_t nRegStart, uint32_t nWrites, uint8_t *pData, void (*callback)(void));
 *    This duplicates the queue management of ADXL_QueueAdd() which
 *    sacrifices encapsulation for efficient memory management.
 */
static uint8_t _ADXL_Write(uint8_t nRegStart, uint32_t nWrites, uint8_t *pData, void (*callback)(void)) {
  // Check for full queue
  if (adxl_queue.full) {
    return ADXL_QUEUE_FULL;
  }
  // Fill next queue item with command data
  adxl_queue.queue[adxl_queue.pIn].taddr = ADXL_PACK_TADDR_WRITE(nRegStart);//(uint8_t)(((nRegStart << 1) | ADXL_TADDR_WRITE) & 0xFF);
  adxl_queue.queue[adxl_queue.pIn].datalen = nWrites;
  adxl_queue.queue[adxl_queue.pIn].pdata = pData;
  adxl_queue.queue[adxl_queue.pIn].callback = callback;
  // Increment pIn index (wrap at boundary)
  if (adxl_queue.pIn == ADXL_MAX_QUEUE_ITEMS - 1) {
    adxl_queue.pIn = 0;
  } else {
    adxl_queue.pIn++;
  }
  // Check for full queue condition
  // If adding an item results in equal indices, we are full
  if (adxl_queue.pIn == adxl_queue.pOut) {
    adxl_queue.full = 1;
  }
  // Update the local copy
  _ADXL_UpdateLocal(nRegStart, (int)nWrites, pData);
  return ADXL_QUEUE_OK;
}

static uint8_t _ADXL_BlockingReadReg(uint8_t nReg, uint8_t *pData) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  uint8_t wbyte = ADXL_PACK_TADDR_READ(nReg);
  return _SPI_BlockingRead(1, &wbyte, 1, pData);
#else
  uint8_t wbytes[2] = {ADXL_363_CMD_READ, nReg};
  return _SPI_BlockingRead(2, wbytes, 1, pData);
#endif
}

static uint8_t _ADXL_BlockingReadBurst(uint8_t nRegStart, uint8_t nReads, uint8_t *pData) {
#if ADXL_TARGET == ADXL_TARGET_ADXL372
  uint8_t wbyte = ADXL_PACK_TADDR_READ(nRegStart);
  return _SPI_BlockingRead(1, &wbyte, nReads, pData);
#else
  uint8_t wbytes[2] = {ADXL_363_CMD_READ, nRegStart};
  return _SPI_BlockingRead(2, wbytes, nReads, pData);
#endif
}

/*
 * static uint8_t ADXL_QueueAdd(queueItem_t *item);
 *    Add an item (at pointer 'item') to the ADXL command queue.
 *    Copy FIRST, then increment index.
 *    Returns ADXL_QUEUE_FULL if the queue is full, otherwise ADXL_QUEUE_OK
 */
static uint8_t ADXL_QueueAdd(queueItem_t *item) {
  // Check for full queue
  if (adxl_queue.full) {
    return ADXL_QUEUE_FULL;
  }
  // Copy data from 'item' to the queue
  for (int n = 0; n < sizeof(queueItem_t); n++) {
    *((uint8_t *)&(adxl_queue.queue[adxl_queue.pIn]) + n) = *((uint8_t *)item + n);
  }
  // Increment pIn index (wrap at boundary)
  if (adxl_queue.pIn == ADXL_MAX_QUEUE_ITEMS - 1) {
    adxl_queue.pIn = 0;
  } else {
    adxl_queue.pIn++;
  }
  // Check for full queue condition
  // If adding an item results in equal indices, we are full
  if (adxl_queue.pIn == adxl_queue.pOut) {
    adxl_queue.full = 1;
  }
  return ADXL_QUEUE_OK;
}

/*
 * static uint8_t _alloc_add(uint8_t len);
 *    Follows write FIRST, then INCREMENT policy.
 *    The allocator uses the first byte of any chunk to store the
 *    length of that allocated chunk, so when you request 'len' bytes
 *    you are actually requesting 'len' + 1 bytes.
 *    Returns 0xFF if 'len' + 1 cannot fit
 *    Returns the index of the next element that is free.
 *    This index is one past the previous value held by adxl_wSpace.pIn
 *      NOTE! To write to this element, you must use modulo math so
 *            you don't write past the end of the buffer!
 *            Use _ALLOC_INSERT(index, element) macro to avoid error.
 *    pIn has already been incremented after function call
 */
static uint8_t _alloc_add(uint8_t len) {
  if (adxl_wSpace.full) {
    ADXL_DBG("Full!\r\n");
    return ALLOC_NOSPACE;
  }
  uint8_t freeSpace = _ALLOC_GET_FREE();
  if (freeSpace < (len + 1)) {
    ADXL_DBG("Space %d!\r\n", freeSpace);
    return ALLOC_NOSPACE;
  }
  uint8_t pIn = adxl_wSpace.pIn;
  //ADXL_DBG("Add %d to %d\r\n", len + 1, pIn);
  // If we get here, there's room for the requested memory
  // Write the requested length to the element at pIn
  adxl_wSpace.wspace[adxl_wSpace.pIn] = len;
  // Both case where pIn + len + 1 > size and < size handled by modulo math
  adxl_wSpace.pIn = (pIn + len + 1) % _ALLOC_BYTES_RESERVED;
  // Check for FIFO full condition
  if (adxl_wSpace.pIn == adxl_wSpace.pOut) {
    adxl_wSpace.full = 1;
  }
  // Return the index just after the original pIn
  return ((pIn + 1) % _ALLOC_BYTES_RESERVED);
}

/*
 * uint8_t alloc_empty(void);
 *    Courtesy function for testing purposes.
 *    Returns 1 if alloc FIFO is empty, 0 otherwise.
 */ 
uint8_t alloc_empty(void) {
  if (adxl_wSpace.pIn == adxl_wSpace.pOut) {
    if (!adxl_wSpace.full) {
      return 1;
    }
  }
  return 0;
}

/*
 * static void _alloc_free(void);
 *    Free the next entry in the allocator FIFO by incrementing
 *    the pOut index by the entry at the current position of pOut.
 */
static void _alloc_free(void) {
  // Handle the case of empty FIFO
  if (adxl_wSpace.pIn == adxl_wSpace.pOut) {
    if (!adxl_wSpace.full) {
      ADXL_DBG("Empty!\r\n");
      return;
    }
  }
  // The length is stored at pOut
  uint8_t len = adxl_wSpace.wspace[adxl_wSpace.pOut];
  uint8_t pOut = adxl_wSpace.pOut;
  //ADXL_DBG("Free %d from %d\r\n", len + 1, pOut);
  // To catch up to pIn, we need to add len + 1 (modulo size)
  adxl_wSpace.pOut = (pOut + len + 1) % _ALLOC_BYTES_RESERVED;
  // Any free will make the FIFO no longer full
  adxl_wSpace.full = 0;
  return;
}

// ================== ADXL363 Target-Specific =================
#if ADXL_TARGET == ADXL_TARGET_ADXL363
// Here we need to mimic reading n samples of ADC data from the FIFO
// Since the ADXL363 doesn't allow using the FIFO for ADC data, we
// do it in software here by reading the ADC data register on every
// trigger in the ISR and storing it in pData.
uint8_t ADXL_ReadFIFO_363(uint32_t n, uint8_t *pData) {
  if (adxl_adc_fifo.nSamples != adxl_adc_fifo.nEntries) {
    return ADXL_FIFO_FILLING;
  }
  // Re-init adc fifo
  adxl_adc_fifo.nSamples = n;
  adxl_adc_fifo.nEntries = 0;
  adxl_adc_fifo.pData = pData;
  adxl_adc_fifo.callback = ADXL_CallbackReadADC;
  // Start the timer with the desired number of samples
  //AXL_StartCounter(n);
  return ADXL_QUEUE_OK;
}

uint8_t ADXL_ReadADC(uint8_t *pData) {
  return _ADXL_ReadBurst(ADXL_REG_ADC_DATA_L, 2, pData);
}

/*
 * void ADXL_FillADCFIFO(void);
 *    Used for local data FIFO-ing (for e.g. ADXL363)
 *    WARNING! This is called in interrupt mode! Don't make blocking read!
 */
void ADXL_FillADCFIFO(void) {
  if (adxl_adc_fifo.nEntries == adxl_adc_fifo.nSamples) {
    // If the fifo is full, just return
    return;
  }
  // Read directly from the ADC registers (storing in the adc fifo)
  uint8_t result = _ADXL_ReadBurst(ADXL_REG_ADC_DATA_L, 2, ((uint8_t *)adxl_adc_fifo.pData + 2*adxl_adc_fifo.nEntries));
  if (result == ADXL_QUEUE_FULL) {
    return;
  }
  // Increment the nEntries counter
  adxl_adc_fifo.nEntries++;
  if (adxl_adc_fifo.nEntries == adxl_adc_fifo.nSamples) {
    if (adxl_adc_fifo.callback != ADXL_PNULL) {
      adxl_adc_fifo.callback();
    }
  }
  return;
}

static void ADXL_CallbackReadADC(void) {
  // This should be called when the pseudo-fifo is full
  //ADXL_DBG("ADC FIFO Filled\r\n");
  return;
}
#endif
// ======================= DEBUG/TESTING ======================
#if ADXL_INCLUDE_TESTS
void testADXL(void) {
  static int needsConfig = 0;
  if (needsConfig) {
    // Initialize reg values to 0
    testBucket.pIn = 0;
    testBucket.pOut = 0;
    for (int n = 0; n < 8; n++) {
      testBucket.regValues[n] = 0;
    }
    ADXL_HWReset();
    ADXL_ConfigDefault();
#if ADXL_TARGET == ADXL_TARGET_ADXL372
    ADXL_SetOffsetX(ADXL_REG_OFFSET_PLUS56);
    //ADXL_FIFOStreamMode();
    ADXL_ConfigFIFO(ADXL_REG_FIFO_CTL_FMT_Z);
#else
    //ADXL_ExtTrigDis();
#endif
    ADXL_FullBWMode();
    needsConfig = 0;
  }
  //testReadReg();
  //testReadBurst();
  //testWriteReg();
  //testWriteBurst();
  //testAllocator();
  //testGetXYZ();
  //testFIFO();
  //testExtTrig();
  //testDirectMode();
  //testBlockingRead();
  testDAQMode();
  //test363ADCFifo();
  //test363ADC();
  //testThresholds();
  //testActInact();
  //testGDetectMode();
  return;
}

static void testReadReg(void) {
  _ADXL_ReadReg(ADXL_REG_DEVID_AD, &testBucket.regValues[0]);
  testBucket.pIn++;
  _ADXL_ReadReg(ADXL_REG_DEVID_MST, &testBucket.regValues[1]);
  testBucket.pIn++;
  _ADXL_ReadReg(ADXL_REG_PARTID, &testBucket.regValues[2]);
  testBucket.pIn++;
  return;
}

static void testCallbackReadReg(void) {
  //ADXL_DBG("regValues[%d] = 0x%x\r\n", testBucket.pOut, testBucket.regValues[testBucket.pOut]);
  ADXL_DBG("regValues[0:2] = 0x%x, 0x%x, 0x%x\r\n", testBucket.regValues[0], testBucket.regValues[1], testBucket.regValues[2]);
  //testBucket.pOut++;
  return;
}

static void testCallbackReadBurst(void) {
  /*
  int16_t data[4];
  data[0] = ADXL_DATA_PACK_UINT16_FROM_BYTES(testBucket.regValues[0], testBucket.regValues[1]);
  data[1] = ADXL_DATA_PACK_UINT16_FROM_BYTES(testBucket.regValues[2], testBucket.regValues[3]);
  data[2] = ADXL_DATA_PACK_UINT16_FROM_BYTES(testBucket.regValues[4], testBucket.regValues[5]);
  data[3] = ADXL_DATA_PACK_UINT16_FROM_BYTES(testBucket.regValues[6], testBucket.regValues[7]);
  //ADXL_DBG("X = %d\tY = %d\tZ = %d\r\n", x, y, z);
  ADXL_DBG("Data[0:3] = %d\t%d\t%d\t%d\r\n", data[0], data[1], data[2], data[3]);
  */
  //uint16_t entries = ADXL_ENTRIES_PACK_UINT16_FROM_BYTES(testBucket.regValues[0], testBucket.regValues[1]);
  //ADXL_DBG("Entries = %d\r\n", entries);
  /*
  uint8_t *p = testBucket.fifoValues;
  ADXL_DBG("FIFO[0:3] = 0x%x 0x%x 0x%x 0x%x\r\n", p[0], p[1], p[2], p[3]);
  */
  return;
}

static void testCallbackWriteReg(void) {
  return;
}

static void testCallbackWriteBurst(void) {
  return;
}

static void testReadBurst(void) {
  _ADXL_ReadBurst(ADXL_REG_DEVID_AD, 4, &testBucket.regValues[0]);
  testBucket.pIn += 4;
  return;
}

static void testWriteReg(void) {
  testBucket.regValues[1] = 0xC;
  _ADXL_WriteReg(_TEST_REG_WRITE, &testBucket.regValues[1]);
  testBucket.pIn++;
  _ADXL_ReadReg(_TEST_REG_WRITE, &testBucket.regValues[0]);
  testBucket.pIn++;
  return;
}

static void testWriteBurst(void) {
  testBucket.regValues[3] = 0xA;
  testBucket.regValues[4] = 0xB;
  testBucket.regValues[5] = 0xC;
  _ADXL_WriteBurst(_TEST_REG_WRITE, 3, &testBucket.regValues[3]);
  _ADXL_ReadBurst(_TEST_REG_WRITE, 3, &testBucket.regValues[0]);
  return;
}

static void testAllocator(void) {
  // First, let's request 1 byte to register 0x39
  if (_ADXL_WriteRegAlloc(ADXL_REG_FIFO_SAMPLES, 0x69) == ADXL_ALLOC_NOSPACE) {
    ADXL_DBG("1X\r\n");
  }
  // Then we'll request 3 bytes to write to registers 0x20-0x22
  uint8_t data[3];
  data[0] = 0xCA;
  data[1] = 0xBF;
  data[2] = 0xEE;
  if (_ADXL_WriteBurstAlloc(_TEST_REG_WRITE, 3, data) == ADXL_ALLOC_NOSPACE) {
    ADXL_DBG("3X\r\n");
  }
  // Finally, we'll read the values back to see if the writes were successful.
  //_ADXL_ReadReg(ADXL_REG_FIFO_SAMPLES, &testBucket.regValues[0]);
  //_ADXL_ReadBurst(_TEST_REG_WRITE, 3, &testBucket.regValues[0]);
  return;
}

static void testGetXYZ(void) {
  //int16_t x = ADXL_DATA_PACK_UINT16_FROM_BYTES(testBucket.regValues[0], testBucket.regValues[1]);
  int16_t x = ADXL_DATA_PACK_UINT16(&(testBucket.regValues[0]));
  //int16_t y = ADXL_DATA_PACK_UINT16_FROM_BYTES(testBucket.regValues[2], testBucket.regValues[3]);
  int16_t y = ADXL_DATA_PACK_UINT16(&(testBucket.regValues[2]));
  //int16_t z = ADXL_DATA_PACK_UINT16_FROM_BYTES(testBucket.regValues[4], testBucket.regValues[5]);
  int16_t z = ADXL_DATA_PACK_UINT16(&(testBucket.regValues[4]));
  ADXL_DBG("X = %d\tY = %d\tZ = %d\r\n", x, y, z);
  ADXL_GetX(&testBucket.regValues[0]);
  ADXL_GetY(&testBucket.regValues[2]);
  ADXL_GetZ(&testBucket.regValues[4]);
  return;
}

static void testFIFO(void) {
  ADXL_ReadFIFO(4, &testBucket.regValues[0]);
  return;
}

static void testExtTrig(void) {
  ADXL_GetFIFOFill(&testBucket.regValues[0]);
  return;
}

static void testDirectMode(void) {
  _ADXL_ReadBurstDirect(ADXL_REG_FIFO_DATA, 100, testBucket.fifoValues);
  //_ADXL_ReadBurst(ADXL_REG_FIFO_DATA, 100, testBucket.fifoValues);
  return;
}

static void testBlockingRead(void) {
  uint8_t test[16];
  //uint8_t result = SPI_BlockingReadOne((ADXL_REG_DEVID_AD << 1 | ADXL_TADDR_READ), &test);
  uint8_t result = SPI_BlockingRead((ADXL_REG_DEVID_AD << 1 | ADXL_TADDR_READ), 1, test);
  //uint8_t result = _ADXL_ReadReg(ADXL_REG_DEVID_AD, testBucket.regValues);
  if (result != 0) {
    ADXL_DBG("result = %d\r\n", result);
  }
  ADXL_DBG("test = 0x%x 0x%x 0x%x 0x%x\r\n", test[0], test[1], test[2], test[3]);
  return;
}

static void testDAQMode(void) {
  static uint8_t phase = 0;
  uint16_t nSamples;
  uint8_t result;
  switch (phase) {
    case 0: // Setup
      ADXL_SetModeDAQ();
      phase++;
      break;
    case 1:
      // Read FIFO counts in blocking mode
      ADXL_GetFIFOFill(&testBucket.regValues[0]);
      // Convert bytes to uint16_t
      nSamples = ADXL_ENTRIES_PACK_UINT16(&(testBucket.regValues[0]));
      result = ADXL_ReadFIFO((uint32_t)nSamples, &testBucket.fifoValues[0]);
      //result = ADXL_ReadFIFO((uint32_t)TESTFIFOSAMPLES, &testBucket.fifoValues[0]);
      if (result != ADXL_QUEUE_OK) {
        ADXL_DBG("result = 0x%x\r\n", result);
      }
      ADXL_DBG("nSamples = %d\r\n", nSamples);
      //AXL_StartCounter(100);
      //phase++;
      break;
    default:
      break;
  }
  return;
}

static void test363ADC(void) {
#if ADXL_TARGET == ADXL_TARGET_ADXL363
  uint8_t result = ADXL_ReadADC(&testBucket.regValues[0]);
  if (result != ADXL_QUEUE_OK) {
    ADXL_DBG("result = 0x%x\r\n", result);
  }
  ADXL_DBG("adc[0:1] = 0x%x 0x%x = %d\r\n", testBucket.regValues[0], testBucket.regValues[1],\
                                            ADXL_DATA_PACK_INT16_FROM_BYTES(testBucket.regValues[1], testBucket.regValues[0]));
  //ADXL_DBG("adc[0:1] = %d\r\n", ADXL_DATA_PACK_INT16_FROM_BYTES(testBucket.fifoValues[1], testBucket.fifoValues[0]));
#endif
  return;
}

static void test363ADCFifo(void) {
  static uint8_t firstTime = 1;
  if (firstTime) {
    //AXL_ReconfigureTimer(625);  // Configure timer for 625 Hz
    firstTime = 0;
  }
  //AXL_StartCounter(100);
  ADXL_GetFIFOFill(&testBucket.regValues[0]);
  uint16_t nSamples = ADXL_ENTRIES_PACK_UINT16(&(testBucket.regValues[0]));
  //ADXL_ReadFIFO((uint32_t)nSamples, &testBucket.fifoValues[0]);
  if (nSamples > 0) {
    //ADXL_DBG("ADC[0] = 0x%x 0x%x\r\n", testBucket.fifoValues[0], testBucket.fifoValues[1]);
    //ADXL_DBG("ADC[0] = %d\r\n", ADXL_DATA_PACK_INT16_FROM_BYTES(testBucket.fifoValues[1], testBucket.fifoValues[0]));
    ADXL_DBG("ADC[0:3] = %d, %d, %d, %d\r\n", ADXL_DATA_PACK_INT16(&testBucket.fifoValues[0]), ADXL_DATA_PACK_INT16(&testBucket.fifoValues[2]),
                                  ADXL_DATA_PACK_INT16(&testBucket.fifoValues[4]), ADXL_DATA_PACK_INT16(&testBucket.fifoValues[6]));
  }
  ADXL_ReadFIFO(10, &testBucket.fifoValues[0]);
  //AXL_StartCounter(100); // This is called by ReadFIFO in -363 fake-fifo mode.
  return;
}

#define actthresh 2000
#define inactthresh 1000
#define acttime 50
#define inacttime 315
static void testThresholds(void) {
  static int first = 1;
  if (first) {
    // First we'll perform a write to the threshold registers,
    ADXL_SetXActThresh(actthresh);
    ADXL_SetXInactThresh(inactthresh);
    ADXL_SetActThreshTime(acttime);
    ADXL_SetInactThreshTime(inacttime);
    first = 0;
    return;
  }
  // Then we'll read back from the registers to confirm: A) The write took. B) The macros encoded/decoded properly.
  // Might as well read in blocking mode
  uint8_t data[2];
  //_ADXL_ReadBurstBlocking(ADXL_REG_THRESH_ACT_X_PATCH, 2, data);
  _ADXL_BlockingReadBurst(ADXL_REG_THRESH_ACT_X_PATCH, 2, data);
  uint16_t thresh = ADXL_THRESHOLD_PACK_UINT16(data);
  ADXL_DBG("X Act Thresh Set: %d. Bytes read[0:1] 0x%x 0x%x; Converted = %d\r\n", actthresh, data[0], data[1], thresh);
  //_ADXL_ReadBurstBlocking(ADXL_REG_THRESH_INACT_X_PATCH, 2, data);
  _ADXL_BlockingReadBurst(ADXL_REG_THRESH_INACT_X_PATCH, 2, data);
  thresh = ADXL_THRESHOLD_PACK_UINT16(data);
  ADXL_DBG("X Inact Thresh Set: %d. Bytes read[0:1] 0x%x 0x%x; Converted = %d\r\n", inactthresh, data[0], data[1], thresh);
  //_ADXL_ReadRegBlocking(ADXL_REG_TIME_ACT, data);
  _ADXL_BlockingReadReg(ADXL_REG_TIME_ACT, data);
  ADXL_DBG("Act Time set: %d; read: %d\r\n", acttime, data[0]);
  //_ADXL_ReadBurstBlocking(ADXL_REG_TIME_INACT_PATCH, 2, data);
  _ADXL_BlockingReadBurst(ADXL_REG_TIME_INACT_PATCH, 2, data);
  thresh = ADXL_INACTIVITY_PACK_UINT16(data);
  ADXL_DBG("Inact Time set: %d; data[0:1] = 0x%x 0x%x. read: %d\r\n", inacttime, data[0], data[1], thresh);
  return;
}

static void testActInact(void) {
  static int first = 1;
  if (first == 0) {
    return;
  }
  uint8_t r;
  switch (first) {
    case 1:
      ADXL_HWReset();
      ADXL_DBG("Hardware Reset\r\n");
      first = 2;
      break;
    case 2:
      ADXL_DBG("Setting Activity & Inactivity\r\n");
      r = ADXL_SetZActThresh(100);
      if (r != ADXL_QUEUE_OK) {
        ADXL_DBG("Act returned %d\r\n", r);
      }
      r = ADXL_SetZInactThresh(50);
      if (r != ADXL_QUEUE_OK) {
        ADXL_DBG("Inact returned %d\r\n", r);
      }
      r = ADXL_EnZActThresh();
      if (r != ADXL_QUEUE_OK) {
        ADXL_DBG("Act Enable returned %d\r\n", r);
      }
      r = ADXL_EnZInactThresh();
      if (r != ADXL_QUEUE_OK) {
        ADXL_DBG("Inact Enable returned %d\r\n", r);
      }
      first = 3;
      break;
    case 3:
      ADXL_DBG("Reading\r\n");
      uint8_t data[2];
      _ADXL_BlockingReadBurst(ADXL_REG_THRESH_ACT_Z_PATCH, 2, data);
      uint16_t thresh = ADXL_THRESHOLD_PACK_UINT16(data);
      ADXL_DBG("Z Act Thresh Read %d\r\n", thresh);
      _ADXL_BlockingReadBurst(ADXL_REG_THRESH_INACT_Z_PATCH, 2, data);
      thresh = ADXL_THRESHOLD_PACK_UINT16(data);
      ADXL_DBG("Z Inact Thresh Read %d\r\n", thresh);
      first = 0;
      break;
    default:
      break;
  }

}

static void testGDetectMode(void) {
  static int first = 1;
  uint8_t r;
  if (first == 1) {
    ADXL_HWReset();
    ADXL_DBG("Hardware Reset\r\n");
    first = 2;
  } else if (first == 2) {
    ADXL_DBG("Going to G-Detect Mode\r\n");
    r = ADXL_SetModeGDetect();
    if (r != ADXL_QUEUE_OK) {
      ADXL_DBG("Returned %d\r\n", r);
    }
    first = 0;
  }
  return;
}

#else /*ADXL_INCLUDE_TESTS*/
void testADXL(void) {
  return;
}
#endif
