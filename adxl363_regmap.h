/*
 *  File: adxl363_regmap.h
 *  Desc:
 *    A separate header file containing device-specific definitions for the ADXL363
 *    accelerometer.
 */

#ifndef __ADXL363_REGMAP_H
#define __ADXL363_REGMAP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= ADXL363 Register Map =========================== */
#define ADXL_REG_DEVID_AD             (0x00)
#define ADXL_REG_DEVID_MST            (0x01)
#define ADXL_REG_DEVID                (0x02)
#define ADXL_REG_REVID                (0x03)
#define ADXL_REG_XDATA                (0x08)
#define ADXL_REG_YDATA                (0x09)
#define ADXL_REG_ZDATA                (0x0a)
#define ADXL_REG_STATUS               (0x0b)
#define ADXL_REG_FIFO_ENTRIES_L       (0x0c)
#define ADXL_REG_FIFO_ENTRIES_H       (0x0d)
#define ADXL_REG_XDATA_L              (0x0e)
#define ADXL_REG_XDATA_H              (0x0f)
#define ADXL_REG_YDATA_L              (0x10)
#define ADXL_REG_YDATA_H              (0x11)
#define ADXL_REG_ZDATA_L              (0x12)
#define ADXL_REG_ZDATA_H              (0x13)
#define ADXL_REG_TEMP_L               (0x14)
#define ADXL_REG_TEMP_H               (0x15)
#define ADXL_REG_ADC_DATA_L           (0x16)
#define ADXL_REG_ADC_DATA_H           (0x17)
#define ADXL_REG_SOFT_RESET           (0x1f)
#define ADXL_REG_THRESH_ACT_L         (0x20)
#define ADXL_REG_THRESH_ACT_H         (0x21)
#define ADXL_REG_TIME_ACT             (0x22)
#define ADXL_REG_THRESH_INACT_L       (0x23)
#define ADXL_REG_THRESH_INACT_H       (0x24)
#define ADXL_REG_TIME_INACT_L         (0x25)
#define ADXL_REG_TIME_INACT_H         (0x26)
#define ADXL_REG_ACT_INACT_CTL        (0x27)
#define ADXL_REG_FIFO_CONTROL         (0x28)
#define ADXL_REG_FIFO_SAMPLES         (0x29)
#define ADXL_REG_INTMAP1              (0x2a)
#define ADXL_REG_INTMAP2              (0x2b)
#define ADXL_REG_FILTER_CTL           (0x2c)
#define ADXL_REG_POWER_CTL            (0x2d)
#define ADXL_REG_SELF_TEST            (0x2e)

// Defining a fake register for reading from the FIFO
#define ADXL_PSEUDOREG_FIFO           (0x30)

// 0x22 TIME_ACT
// Activity Time is measured in units of inverse ODR
// (i.e. 1 unit = 2.5ms for ODR=400Hz)
#define ADXL_REG_TIME_ACT_DEFAULT           (0x01)

// 0x27 ACT_INACT_CTL
#define ADXL_REG_ACT_INACT_CTL_ACT_EN       (0x01)
#define ADXL_REG_ACT_INACT_CTL_ACT_REF      (0x02)
#define ADXL_REG_ACT_INACT_CTL_INACT_EN     (0x04)
#define ADXL_REG_ACT_INACT_CTL_INACT_REF    (0x08)
#define ADXL_REG_ACT_INACT_CTL_LINKLOOP_MASK    (0x30)
#define ADXL_REG_ACT_INACT_CTL_LINKLOOP_DEFAULT (0x00)
#define ADXL_REG_ACT_INACT_CTL_LINKLOOP_LINKED  (0x10)
#define ADXL_REG_ACT_INACT_CTL_LINKLOOP_LOOP    (0x30)
#define ADXL_REG_ACT_INACT_CTL_DEFAULT      (0x00)

// 0x28 FIFO_CONTROL
#define ADXL_REG_FIFO_CTL_MODE_MASK         (0x03)
#define ADXL_REG_FIFO_CTL_MODE_DIS          (0x00)
#define ADXL_REG_FIFO_CTL_MODE_OLDEST       (0x01)
#define ADXL_REG_FIFO_CTL_MODE_STREAM       (0x02)
#define ADXL_REG_FIFO_CTL_MODE_TRIG         (0x03)

// 0x2A INTMAP1
#define ADXL_REG_INTMAP1_DATA_RDY           (0x01)
#define ADXL_REG_INTMAP1_FIFO_RDY           (0x02)
#define ADXL_REG_INTMAP1_FIFO_WATERMARK     (0x04)
#define ADXL_REG_INTMAP1_FIFO_OVERRUN       (0x08)
#define ADXL_REG_INTMAP1_ACT                (0x10)
#define ADXL_REG_INTMAP1_INACT              (0x20)
#define ADXL_REG_INTMAP1_AWAKE              (0x40)
#define ADXL_REG_INTMAP1_ACTIVE_LOW         (0x80)

// 0x2B INTMAP2
#define ADXL_REG_INTMAP2_DATA_RDY           (0x01)
#define ADXL_REG_INTMAP2_FIFO_RDY           (0x02)
#define ADXL_REG_INTMAP2_FIFO_WATERMARK     (0x04)
#define ADXL_REG_INTMAP2_FIFO_OVERRUN       (0x08)
#define ADXL_REG_INTMAP2_ACT                (0x10)
#define ADXL_REG_INTMAP2_INACT              (0x20)
#define ADXL_REG_INTMAP2_AWAKE              (0x40)
#define ADXL_REG_INTMAP2_ACTIVE_LOW         (0x80)

// 0x2C FILTER_CTL
#define ADXL_REG_FILTER_CTL_ODR_MASK        (0x07)
#define ADXL_REG_FILTER_CTL_ODR_12HZ        (0x00)
#define ADXL_REG_FILTER_CTL_ODR_25HZ        (0x01)
#define ADXL_REG_FILTER_CTL_ODR_50HZ        (0x02)
#define ADXL_REG_FILTER_CTL_ODR_100HZ       (0x03)
#define ADXL_REG_FILTER_CTL_ODR_200HZ       (0x04)
#define ADXL_REG_FILTER_CTL_ODR_400HZ       (0x05)
#define ADXL_REG_FILTER_CTL_EXT_SAMPLE_EN   (0x08)
#define ADXL_REG_FILTER_CTL_EXT_SAMPLE_DIS  (0x00)
#define ADXL_REG_FILTER_CTL_HALF_BW_EN      (0x10)
#define ADXL_REG_FILTER_CTL_HALF_BW_DIS     (0x00)
#define ADXL_REG_FILTER_CTL_RANGE_MASK      (0xC0)
#define ADXL_REG_FILTER_CTL_RANGE_2G        (0x00)
#define ADXL_REG_FILTER_CTL_RANGE_4G        (0x40)
#define ADXL_REG_FILTER_CTL_RANGE_8G        (0x80)
#define ADXL_REG_FILTER_CTL_DEFAULT   (ADXL_REG_FILTER_CTL_ODR_400HZ | ADXL_REG_FILTER_CTL_EXT_SAMPLE_EN | \
                                       ADXL_REG_FILTER_CTL_HALF_BW_DIS | ADXL_REG_FILTER_CTL_RANGE_8G)

// 0x2D POWER_CTL
#define ADXL_REG_POWER_CTL_MODE_MASK        (0x03)
#define ADXL_REG_POWER_CTL_MODE_STBY        (0x00)
#define ADXL_REG_POWER_CTL_MODE_MEAS        (0x02)
#define ADXL_REG_POWER_CTL_AUTOSLEEP        (0x04)
#define ADXL_REG_POWER_CTL_WAKEUP           (0x08)
#define ADXL_REG_POWER_CTL_LOW_NOISE_MASK   (0x30)
#define ADXL_REG_POWER_CTL_LOW_NOISE_NORMAL (0x00)
#define ADXL_REG_POWER_CTL_LOW_NOISE_LOW    (0x10)
#define ADXL_REG_POWER_CTL_LOW_NOISE_ULOW   (0x20)
#define ADXL_REG_POWER_CTL_EXT_CLK_EN       (0x40)
#define ADXL_REG_POWER_CTL_EXT_CLK_DIS      (0x00)
#define ADXL_REG_POWER_CTL_EXT_ADC_EN       (0x80)
#define ADXL_REG_POWER_CTL_EXT_ADC_DIS      (0x00)
#define ADXL_REG_POWER_CTL_DEFAULT    (ADXL_REG_POWER_CTL_MODE_MEAS | ADXL_REG_POWER_CTL_LOW_NOISE_NORMAL | \
                                       ADXL_REG_POWER_CTL_EXT_CLK_DIS | ADXL_REG_POWER_CTL_EXT_ADC_EN)

// PATCH LAYER
// These defines are to make the ADXL363 look like the ADXL372 as much as possible
#define ADXL_REG_PARTID               ADXL_REG_DEVID
#define ADXL_REG_FIFO_ENTRIES2        ADXL_REG_FIFO_ENTRIES_H
#define ADXL_REG_FIFO_ENTRIES         ADXL_REG_FIFO_ENTRIES_L
#define ADXL_REG_FIFO_ENTRIES_PATCH   ADXL_REG_FIFO_ENTRIES_L
#define ADXL_REG_FIFO_CTL             ADXL_REG_FIFO_CONTROL
#define ADXL_REG_RESET                ADXL_REG_SOFT_RESET
#define ADXL_REG_FIFO_DATA            ADXL_PSEUDOREG_FIFO
#define ADXL_REG_THRESH_ACT_X_PATCH   ADXL_REG_THRESH_ACT_L
#define ADXL_REG_THRESH_ACT_Y_PATCH   ADXL_REG_THRESH_ACT_L
#define ADXL_REG_THRESH_ACT_Z_PATCH   ADXL_REG_THRESH_ACT_L
#define ADXL_REG_TIME_INACT_PATCH     ADXL_REG_TIME_INACT_L
#define ADXL_REG_THRESH_INACT_X_PATCH ADXL_REG_THRESH_INACT_L
#define ADXL_REG_THRESH_INACT_Y_PATCH ADXL_REG_THRESH_INACT_L
#define ADXL_REG_THRESH_INACT_Z_PATCH ADXL_REG_THRESH_INACT_L
#define ADXL_REG_TIME_INACT_PATCH     ADXL_REG_TIME_INACT_L

// Different devices order the bytes H/L in different directions.
#define ADXL_REG_XDATA_PATCH          ADXL_REG_XDATA_L
#define ADXL_REG_YDATA_PATCH          ADXL_REG_YDATA_L
#define ADXL_REG_ZDATA_PATCH          ADXL_REG_ZDATA_L

// 0x1F SOFT RESET
#define ADXL_REG_RESET_CODE           (0x52)

// Command Bytes
#define ADXL_363_CMD_WRITE            (0x0A)
#define ADXL_363_CMD_READ             (0x0B)
#define ADXL_363_CMD_READ_FIFO        (0x0D)

// Data Conversion
// ADXL363 stores bits [11:8] in msb[3:0] and bits [7:0] in lsb[7:0]
#define __ADXL_DATA_PACK_UINT16(msb, lsb)       (((uint16_t)(msb & 0x000F) << 8) | ((uint16_t)(lsb & 0x00FF)))
// lsb is in the lower register, msb in the next higher
#define _ADXL_DATA_PACK_UINT16(pData)           (__ADXL_DATA_PACK_UINT16(*((uint8_t *)pData + 1), *(uint8_t *)pData))
#define __ADXL_DATA_PACK_INT16(msb, lsb)        (int16_t)((msb & 0x08) ? __ADXL_DATA_PACK_UINT16(msb, lsb) | 0xF000 : __ADXL_DATA_PACK_UINT16(msb, lsb))
#define _ADXL_DATA_PACK_INT16(pData)            (__ADXL_DATA_PACK_INT16(*((uint8_t *)pData + 1), *(uint8_t *)pData))

// Unpack Data to Fake a FIFO
// Places lsb into pData[0] and msb into pData[1]
#define _ADXL363_DATA_UNPACK_INT16(n, pData)    do {pData[0] = (uint8_t)(*((uint16_t *)&n) & 0xFF); \
                                                   pData[1] = (uint8_t)((*((uint16_t *)&n) >> 8) & 0x0F);} while (0);

// FIFO Entries Conversion
#define __ADXL_ENTRIES_PACK_UINT16(msb, lsb)    (((uint16_t)(msb & 0xFF) << 8) | (uint16_t)(lsb & 0xFF))
#define _ADXL_ENTRIES_PACK_UINT16(pData)        (__ADXL_ENTRIES_PACK_UINT16(*((uint8_t *)pData + 1), *((uint8_t *)pData)))

// Unpack Data to Fake a FIFO
// Places lsb into pData[0] and msb into pData[1]
#define _ADXL363_ENTRIES_UNPACK_INT16(n, pData)  do {pData[0] = (uint8_t)(*((uint16_t *)&n) & 0xFF); \
                                                   pData[1] = (uint8_t)((*((uint16_t *)&n) >> 8) & 0x0F);} while (0);

// Threshold Values (11-bit unsigned)
#define _ADXL_THRESHOLD_UNPACK_UINT16(n, pData)  do {*(uint8_t *)pData = (uint8_t)((uint16_t)n & 0xFF);\
                                                     *((uint8_t *)pData + 1) = (uint8_t)(((uint16_t)n & 0x0700) >> 8);} while (0)
#define _ADXL_THRESHOLD_PACK_UINT16(pData)       (((uint16_t)*(uint8_t *)pData) | ((uint16_t)*((uint8_t *)pData + 1) & 0x07) << 8)

// Inactivity Time Conversion (16-bit unsigned)
#define _ADXL_INACTIVITY_UNPACK_UINT16(n, pData) do {*((uint8_t *)pData + 1) = (uint8_t)(((uint16_t)n & 0xFF00) >> 8);\
                                                     *(uint8_t *)pData = (uint8_t)((uint16_t)n & 0x00FF);} while (0)
#define _ADXL_INACTIVITY_PACK_UINT16(pData)      (((uint16_t)*((uint8_t *)pData + 1) << 8 | (uint16_t)*(uint8_t *)pData))

// Device-Specific Limits
#define _ADXL_MAX_TRIGGER_FREQ       (625)

/* ======================= ADXL363 Exported Typedef ========================= */
typedef struct {
  // Includes only the writeable registers
  uint8_t rTHRESH_ACT_L;
  uint8_t rTHRESH_ACT_H;
  uint8_t rTIME_ACT;
  uint8_t rTHRESH_INACT_L;
  uint8_t rTHRESH_INACT_H;
  uint8_t rTIME_INACT_L;
  uint8_t rTIME_INACT_H;
  uint8_t rACT_INACT_CTL;
  uint8_t rFIFO_CONTROL;
  uint8_t rFIFO_SAMPLES;
  uint8_t rINTMAP1;
  uint8_t rINTMAP2;
  uint8_t rFILTER_CTL;
  uint8_t rPOWER_CTL;
} adxl_reg_t;

#define _ADXL_REG_START_OFFSET        (0x20)
#define _ADXL_REG_END_OFFSET          (0x2D)

#ifdef __cplusplus
}
#endif

#endif //__ADXL363_REGMAP_H

