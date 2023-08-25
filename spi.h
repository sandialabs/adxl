/*
 *  File:     spi.h
 *  Author:   Keith Penney
 *  Created:  201111
 *  Desc:
 *    This header file and its associated source file implement the hardware control, ISRs, and
 *    software state machine controller for the SPI1 interface of the STM32WB55 for use with
 *    adxl.c/h
 *  Usage Notes:
 *    SPI Peripheral Configuration:
 *      Disable SPI (NVIC) global IRQ. Iterrupts are not used - only DMA and blocking modes.
 *    API:
 *      The SPI peripheral operates primarily in DMA mode and is managed in software by a single
 *      function (SPI_Control) which should be called periodically in the main loop (or via a
 *      scheduler).  Part of the SPI controller software management is processing commands in
 *      the queue (implemented in adxl.c/.h).  When the peripheral is free, the next command in
 *      the queue (if not-empty) is copied to the local buffers to be available for the DMA engine.
 *      Queue management is covered in adxl.c/.h.
 *      This application occasionally calls for rapid, time-sensitive reads which are implemented
 *      as blocking functions.  These functions wait until the SPI controller says the peripheral
 *      is free, then take over the hardware (bypassing the queue system and SPI software control).
 */

#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================= Includes =============================== */
#include <stdint.h>
#include "main.h"   // Need this for all the STM32Cube_HAL/LL stuff

/* ====================== Exported Function Prototypes ====================== */
uint8_t SPI_SysInit(void);
void SPI_Control(void);
uint8_t SPI_BlockingRead(uint8_t wbyte, uint8_t nBytes, uint8_t *rdata);
uint8_t SPI_BlockingReadOne(uint8_t wbyte, uint8_t *rdata);
uint8_t _SPI_BlockingRead(uint8_t nHeader, uint8_t *wbytes, uint8_t nRead, uint8_t *rdata);
int SPI_Busy(void);
void SPI_Start(uint32_t len);
void SPI_SetCmdCallback(void (*cb)(void));
uint8_t SPI_SetWData(uint8_t *pData, uint32_t nBytes, uint32_t offset);
uint8_t SPI_GetRData(uint8_t *pData, uint32_t nBytes, uint32_t offset);
void SPI_SetDMAAddrRead(uint32_t *pdata);
void SPI_SetDMAReadDefault(void);
void SPI_DMAP2MISR(void);
void SPI_DMAM2PISR(void);

#define SPI_BLOCKING_READ_SUCCESS       (0x00)
#define SPI_BLOCKING_READ_TIMEOUT       (0x01)
#define SPI_BLOCKING_READ_ZEROBYTES     (0x02)
#define SPI_BUSY                        (1)
#define SPI_FREE                        (0)
#define SPI_OK                          (0x00)
#define SPI_OVERRUN                     (0x01)

#ifdef __cplusplus
}
#endif

#endif //__SPI_H

