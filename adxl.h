/*
 *  File:     adxl.h
 *  Author:   Keith Penney
 *  Created:  201111
 *  Updated:  210122
 *  Desc:
 *    This header file and its associated source file implement an asynchronous read/write
 *    API for the ADXL372 accelerometer via SPI interface.  Most of the functions are also
 *    mapped/modified for use with the ADXL363 as well, but it does not have all the features
 *    available in the ADXL372.
 *
 *  Usage Notes:
 *  ------------
 *    API:
 *      Several generic functions have been implemented here, but a focus has been placed on
 *      the application required by the rfrtd project.  Thus, some features available in the
 *      ADXL device are not implemented here.  The driver stores a local copy of all the
 *      writable registers in the ADXL device (except for self-test and reset registers). It
 *      uses these copies to modify only certain parts of bitmapped registers as needed -
 *      leaving previously entered valued untouched (actually re-writing these values, but
 *      that's the best we can do with SPI transactions).
 *      Three distinct operating modes have been implemented here for use with the rfrtd
 *      application: G-Detect Mode, DAQ Mode, and Standby.
 *      Standby simply places the ADXL device in a low-power standby mode.
 *      G-Detect Mode uses the activity/inactivity detection hardware in the ADXL device to
 *      generate an interrupt signal used to monitor the speed and phase of the reciprocating
 *      movement.
 *      DAQ Mode places the device in external trigger mode and enables its hardware FIFO.
 *      In this mode, each rising edge received on the ADXL device EXT_SYNC input records a
 *      new acceleration measurement (along a single axis as specified by ADXL_TARGET_AXIS)
 *      and places it into the FIFO to be read later.
 *    Queue System:
 *      Implemented here is a simple FIFO-style command queue for SPI commands (read/writes)
 *      to be sent over the SPI peripheral (see spi.c/.h).  The queue allows for asynchronous
 *      processing of many commands without any extra application logic or concerns about
 *      timing or race conditions.  Once a command is processed, its associated callback
 *      function is called.  Here, the application MUST copy the data read by the SPI driver
 *      and can elaborate according to needs, while keeping in mind that the callbacks are
 *      called in interrupt mode and should be kept as short as possible.
 *    Direct Mode:
 *      For most use cases, the data read from the SPI peripheral is stored temporarily in a
 *      static buffer within spi.c and is available to copy from the SPI buffer if needed
 *      (typically done in adxl callback). This is called 'indirect' mode because the data
 *      does not go straight to the associated pointer directly.  The advantages of this are
 *      that the garbage data read from MISO while the write data is going out over MOSI is
 *      not copied to the users's buffer and that the buffer is a dedicated statically allocated
 *      buffer free from corruption by application code.
 *      An alternative is 'direct' mode where the DMA is redirected to the buffer supplied by
 *      the user.  In this mode, we avoid copying data twice but pay the price of keeping track
 *      of the garbage data (in the case of ADXL372, only 1 byte) and ensuring the application
 *      logic does not corrupt the data before the DMA read completes.  Direct mode is required
 *      for any reads of length longer than the SPI controller buffers allocated in spi.c/.h.
 *    Allocator:
 *      When we need to write data to the device over SPI, we need to ensure the data remains
 *      uncorrupted until the DMA engine has copied it into the SPI data register.  This either
 *      requires a series of statically allocated buffers (depending on the number/timing of
 *      writes) or a memory allocation scheme.  The allocator implemented here takes advantage
 *      of the fact that all commands are processed sequentially and thus memory can be allocated
 *      and freed without concerns about addressing or fragmentation.  Thus, the allocator is
 *      implemented as a simple FIFO of configurable size.  
 *      There is a separate write API which use allocated memory.  These can be safely used with
 *      temporary stack variables because the data is copied to the allocated block before the
 *      function returns (or the funtion returns an error code indicating the allocator does not
 *      have the requested space available).
 *    Blocking Mode:
 *      Some time-sensitive functions have been implemented using the blocking mode offered by the
 *      SPI driver (see spi.c/.h).  Because these functions can tie up the processor for a
 *      significant time (especially if called after a long read is being processed from the
 *      queue), they should be used only when absolutely necessary.
 *    Callbacks:
 *      The different read types all have dedicated callback functions which are executed in thread
 *      mode (not interrupt mode) after the SPI transaction completes. For example, the function
 *      ADXL_ReadDeviceIDs() calls _ADXL_ReadBurst(). After successful SPI transaction, the callback
 *      ADXL_CallbackReadBurst() is called which then calls the user callback if pre-registered
 *      with ADXL_RegisterCallback(ADXL_NCB_READBURST, pCallback);
 */

#ifndef __ADXL_H
#define __ADXL_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================= Includes =============================== */
#include <stdint.h>

// Define this to target the ADXL363 instead of the ADXL372 to enable acceleration
// simulation via external analog input
//#define ADXL_SIMULATOR

#define ADXL_TARGET_ADXL372         (0)
#define ADXL_TARGET_ADXL363         (1)

#ifdef ADXL_SIMULATOR
#define ADXL_TARGET ADXL_TARGET_ADXL363
#include "adxl363_regmap.h"
#define ADXL_SIMULATE_ReadADC       ADXL_FillADCFIFO
#else
#define ADXL_TARGET ADXL_TARGET_ADXL372
#include "adxl372_regmap.h"
#define ADXL_SIMULATE_ReadADC()
#endif

/* ============================= Exported Defines =========================== */

// Set this to 1 if the ADXL is the only device using the SPI driver. If multiple
// devices share the spi driver, set this to 0 and make sure to call SPI_Control()
// in your main loop.
#define ADXL_STANDALONE             (1)

/* ========================= Configurable Parameters ======================== */
// Maximum number of items in the command queue
#define ADXL_MAX_QUEUE_ITEMS        (16)
// Maximum number of bytes reserved for use by the allocator FIFO
#define _ALLOC_BYTES_RESERVED       (128)

// Select the axis of sensitivity depending on chip orientation
#define _ADXL_AXIS_X                (0x00)
#define _ADXL_AXIS_Y                (0x01)
#define _ADXL_AXIS_Z                (0x02)
#define ADXL_TARGET_AXIS            _ADXL_AXIS_Y
// Wrap macros to auto-target the desired axis of sensitivity
#if ADXL_TARGET_AXIS == _ADXL_AXIS_X
#define ADXL_SetActThresh(threshold)    (ADXL_SetXActThresh(threshold))
#define ADXL_SetInactThresh(threshold)  (ADXL_SetXInactThresh(threshold))
#elif ADXL_TARGET_AXIS == _ADXL_AXIS_Y
#define ADXL_SetActThresh(threshold)    (ADXL_SetYActThresh(threshold))
#define ADXL_SetInactThresh(threshold)  (ADXL_SetYInactThresh(threshold))
#else
#define ADXL_SetActThresh(threshold)    (ADXL_SetZActThresh(threshold))
#define ADXL_SetInactThresh(threshold)  (ADXL_SetZInactThresh(threshold))
#endif
// Set the activity and inactivity threshold values
#define ADXL_ACT_THRESHOLD          (14)  // Units 100mg for ADXL372
#define ADXL_INACT_THRESHOLD        (11)  // Units 100mg for ADXL372
// Set the desired offset applied to the axis of sensitivity
#define ADXL_TARGET_AXIS_OFFSET     (ADXL_REG_OFFSET_PLUS16)  // Only applicable to ADXL372

#if ADXL_TARGET_AXIS == _ADXL_AXIS_X
#define ADXL_FIFO_FMT               ADXL_REG_FIFO_CTL_FMT_X
#elif ADXL_TARGET_AXIS == _ADXL_AXIS_Y
#define ADXL_FIFO_FMT               ADXL_REG_FIFO_CTL_FMT_Y
#else // Assume _ADXL_AXIS_Z
#define ADXL_FIFO_FMT               ADXL_REG_FIFO_CTL_FMT_Z
#endif
/* ======================= Non-Configurable Parameters ====================== */
// Device Parameters/Limits
// The maximum number of samples that fit in the on-board FIFO is 512 (hard limit)
#define ADXL_FIFO_MAX_SAMPLES       (512)

// The maximum trigger rate (ODR) of the device
#define ADXL_MAX_TRIGGER_FREQ       _ADXL_MAX_TRIGGER_FREQ

/* ============================ Application Macros ========================== */
// Bitmask for isolating the R/!W flag from queueItem_t.taddr
#define ADXL_TADDR_READ_BM          (0x01)
#define ADXL_TADDR_READ             (0x01)
#define ADXL_TADDR_WRITE            (0x00)
#define ADXL_PNULL                  (void *)0x00
#define ADXL_QUEUEITEM_DIRECT       (0x01)
#define ADXL_QUEUEITEM_INDIRECT     (0x00)
#define ADXL_QUEUEITEM_DIRECT_BM    (0x01)

// Return values
#define ADXL_QUEUE_OK               (0)
#define ADXL_QUEUE_FULL             (1)
#define ADXL_QUEUE_EMPTY            (2)
#define ADXL_ALLOC_NOSPACE          (3)
#define ADXL_FIFO_FILLING           (4)
#define ALLOC_NOSPACE               (0xFF)

// Acceleration Data Conversion
// Exposing these macros from their corresponding definition in adxl3xx_regmap.h
#define ADXL_DATA_PACK_UINT16(pData)                  _ADXL_DATA_PACK_UINT16(pData)
#define ADXL_DATA_PACK_INT16(pData)                   _ADXL_DATA_PACK_INT16(pData)
#define ADXL_DATA_PACK_UINT16_FROM_BYTES(msb, lsb)    __ADXL_DATA_PACK_UINT16(msb, lsb)
#define ADXL_DATA_PACK_INT16_FROM_BYTES(msb, lsb)     __ADXL_DATA_PACK_INT16(msb, lsb)

// FIFO Number of Entries Data Conversion
// Exposing these macros from their corresponding definition in adxl3xx_regmap.h
#define ADXL_ENTRIES_PACK_UINT16(pData)               _ADXL_ENTRIES_PACK_UINT16(pData)
#define ADXL_ENTRIES_PACK_UINT16_FROM_BYTES(msb, lsb) __ADXL_ENTRIES_PACK_UINT16(msb, lsb)

// Threshold Value Conversion
#define ADXL_THRESHOLD_UNPACK_UINT16(n, pData)        _ADXL_THRESHOLD_UNPACK_UINT16(n, pData)
#define ADXL_THRESHOLD_PACK_UINT16(pData)             _ADXL_THRESHOLD_PACK_UINT16(pData)

// Inactivity Time Value Conversion
#define ADXL_INACTIVITY_UNPACK_UINT16(n, pData)       _ADXL_INACTIVITY_UNPACK_UINT16(n, pData)
#define ADXL_INACTIVITY_PACK_UINT16(pData)            _ADXL_INACTIVITY_PACK_UINT16(pData)

// Helper macros
#define ADXL_PACK_TADDR(addr, rnw)  ((uint8_t)(((addr << 1) | rnw) & 0xFF))
#define ADXL_PACK_TADDR_WRITE(addr) ADXL_PACK_TADDR(addr, ADXL_TADDR_WRITE)
#define ADXL_PACK_TADDR_READ(addr)  ADXL_PACK_TADDR(addr, ADXL_TADDR_READ)
#define ADXL_UNPACK_TADDR(taddr)    ((uint8_t)(taddr >> 1))
#define ADXL_TADDR_ISREAD(taddr)    ((taddr & ADXL_TADDR_READ_BM) == ADXL_TADDR_READ)
#define ADXL_TADDR_ISWRITE(taddr)   ((taddr & ADXL_TADDR_READ_BM) == ADXL_TADDR_WRITE)
/* ============================ Exported Typedefs =========================== */
typedef struct {
  uint8_t taddr;      // taddr[0] = type (0 for Write, 1 for Read); taddr[7:1] = Reg address
  uint8_t direct;     // 0x01 means DMA read address will be set to 'pdata' (bypassing SPI rdata buffer)
  uint32_t datalen;   // Length of data to read or write
  uint8_t *pdata;     // Pointer to data to write or place where read data will be stored
  void (*callback)(void); // Callback function when read data is ready or data has been written
} queueItem_t;

// User callback function registration enum constants
typedef enum {
  ADXL_NCB_READREG,
  ADXL_NCB_READBURST,
  ADXL_NCB_READREGDIRECT,
  ADXL_NCB_READBURSTDIRECT,
  ADXL_NCB_WRITEREG,
  ADXL_NCB_WRITEBURST
} ADXL_NCB_t;

/* ====================== Exported Function Prototypes ====================== */
void ADXL_Init(void);
void ADXL_Control(void);
void ADXL_RegisterCallback(ADXL_NCB_t nCB, void (*callback)(void));
uint8_t ADXL_ReadDeviceIDs(uint8_t *pData);
uint8_t ADXL_ReadDeviceIDsBlock(uint8_t *pData);
uint8_t ADXL_ConfigDefault(void);
uint8_t ADXL_HWReset(void);
uint8_t ADXL_Standby(void);
uint8_t ADXL_FullBWMode(void);
uint8_t ADXL_FIFOStreamMode(void);
uint8_t ADXL_GetX(uint8_t *pData);
uint8_t ADXL_GetY(uint8_t *pData);
uint8_t ADXL_GetZ(uint8_t *pData);
uint8_t ADXL_ConfigFIFO(uint8_t fmt);
uint8_t ADXL_ReadFIFO(uint32_t n, uint8_t *pData);
uint8_t ADXL_SetOffsetX(uint8_t nOffset);
uint8_t ADXL_SetOffsetY(uint8_t nOffset);
uint8_t ADXL_SetOffsetZ(uint8_t nOffset);
uint8_t ADXL_SetODR(uint8_t ODR);
uint8_t ADXL_ExtTrigEn(void);
uint8_t ADXL_ExtTrigDis(void);
uint8_t ADXL_SetXActThresh(uint16_t threshold);
uint8_t ADXL_SetYActThresh(uint16_t threshold);
uint8_t ADXL_SetZActThresh(uint16_t threshold);
uint8_t ADXL_EnXActThresh(void);
uint8_t ADXL_DisXActThresh(void);
uint8_t ADXL_EnYActThresh(void);
uint8_t ADXL_DisYActThresh(void);
uint8_t ADXL_EnZActThresh(void);
uint8_t ADXL_DisZActThresh(void);
uint8_t ADXL_EnXInactThresh(void);
uint8_t ADXL_DisXInactThresh(void);
uint8_t ADXL_EnYInactThresh(void);
uint8_t ADXL_DisYInactThresh(void);
uint8_t ADXL_EnZInactThresh(void);
uint8_t ADXL_DisZInactThresh(void);
uint8_t ADXL_SetActThreshTime(uint8_t time);
uint8_t ADXL_SetXInactThresh(uint16_t threshold);
uint8_t ADXL_SetYInactThresh(uint16_t threshold);
uint8_t ADXL_SetZInactThresh(uint16_t threshold);
uint8_t ADXL_SetInactThreshTime(uint16_t time);
uint8_t ADXL_GetFIFOFill(uint8_t *pData);

/* ======================== Operating Mode Settings ========================= */
uint8_t ADXL_SetModeGDetect(void);
uint8_t ADXL_SetModeDAQ(void);
uint8_t ADXL_SetModeStandby(void);

/* ============================= Low-Level API ============================== */
uint8_t ADXL_QueueNext(void);
uint8_t ADXL_QueueGet(volatile queueItem_t *item);
void ADXL_CallbackReadReg(void);
void ADXL_CallbackReadRegDirect(void);
void ADXL_CallbackReadBurst(void);
void ADXL_CallbackReadBurstDirect(void);
void ADXL_CallbackWriteReg(void);
void ADXL_CallbackWriteRegAlloc(void);
void ADXL_CallbackWriteBurst(void);
void ADXL_CallbackWriteBurstAlloc(void);
uint8_t alloc_empty(void);

#if ADXL_TARGET == ADXL_TARGET_ADXL363
void ADXL_FillADCFIFO(void);
uint8_t ADXL_ReadADC(uint8_t *pData);
#endif

// Debug/testing
void testADXL(void);

#ifdef __cplusplus
}
#endif

#endif //__ADXL_H

