/*
 *  File: adxl372_regmap.h
 *  Desc:
 *    A separate header file containing device-specific definitions for the ADXL372
 *    accelerometer.
 */

#ifndef __ADXL372_REGMAP_H
#define __ADXL372_REGMAP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= ADXL372 Register Map =========================== */
#define ADXL_REG_DEVID_AD             0x00 
#define ADXL_REG_DEVID_MST            0x01 
#define ADXL_REG_PARTID               0x02 
#define ADXL_REG_REVID                0x03 
#define ADXL_REG_STATUS               0x04 
#define ADXL_REG_STATUS2              0x05 
#define ADXL_REG_FIFO_ENTRIES2        0x06 
#define ADXL_REG_FIFO_ENTRIES         0x07 
#define ADXL_REG_XDATA_H              0x08 
#define ADXL_REG_XDATA_L              0x09 
#define ADXL_REG_YDATA_H              0x0a 
#define ADXL_REG_YDATA_L              0x0b 
#define ADXL_REG_ZDATA_H              0x0c 
#define ADXL_REG_ZDATA_L              0x0d 
#define ADXL_REG_MAXPEAK_X_H          0x15 
#define ADXL_REG_MAXPEAK_X_L          0x16 
#define ADXL_REG_MAXPEAK_Y_H          0x17 
#define ADXL_REG_MAXPEAK_Y_L          0x18 
#define ADXL_REG_MAXPEAK_Z_H          0x19 
#define ADXL_REG_MAXPEAK_Z_L          0x1a 
#define ADXL_REG_OFFSET_X             0x20 
#define ADXL_REG_OFFSET_Y             0x21 
#define ADXL_REG_OFFSET_Z             0x22 
#define ADXL_REG_THRESH_ACT_X_H       0x23 
#define ADXL_REG_THRESH_ACT_X_L       0x24 
#define ADXL_REG_THRESH_ACT_Y_H       0x25 
#define ADXL_REG_THRESH_ACT_Y_L       0x26 
#define ADXL_REG_THRESH_ACT_Z_H       0x27 
#define ADXL_REG_THRESH_ACT_Z_L       0x28 
#define ADXL_REG_TIME_ACT             0x29 
#define ADXL_REG_THRESH_INACT_X_H     0x2a 
#define ADXL_REG_THRESH_INACT_X_L     0x2b 
#define ADXL_REG_THRESH_INACT_Y_H     0x2c 
#define ADXL_REG_THRESH_INACT_Y_L     0x2d 
#define ADXL_REG_THRESH_INACT_Z_H     0x2e 
#define ADXL_REG_THRESH_INACT_Z_L     0x2f 
#define ADXL_REG_TIME_INACT_H         0x30 
#define ADXL_REG_TIME_INACT_L         0x31 
#define ADXL_REG_THRESH_ACT2_X_H      0x32 
#define ADXL_REG_THRESH_ACT2_X_L      0x33 
#define ADXL_REG_THRESH_ACT2_Y_H      0x34 
#define ADXL_REG_THRESH_ACT2_Y_L      0x35 
#define ADXL_REG_THRESH_ACT2_Z_H      0x36 
#define ADXL_REG_THRESH_ACT2_Z_L      0x37 
#define ADXL_REG_HPF                  0x38 
#define ADXL_REG_FIFO_SAMPLES         0x39 
#define ADXL_REG_FIFO_CTL             0x3a 
#define ADXL_REG_INT1_MAP             0x3b 
#define ADXL_REG_INT2_MAP             0x3c 
#define ADXL_REG_TIMING               0x3d 
#define ADXL_REG_MEASURE              0x3e 
#define ADXL_REG_POWER_CTL            0x3f 
#define ADXL_REG_SELF_TEST            0x40 
#define ADXL_REG_RESET                0x41 
#define ADXL_REG_FIFO_DATA            0x42 

// Register values
// 0x21-0x23 OFFSET_*
// See figure 36 on page 25 of the ADXL372 datasheet
#define ADXL_REG_OFFSET_ZERO          (0x0)
#define ADXL_REG_OFFSET_PLUS08        (0x1)
#define ADXL_REG_OFFSET_PLUS16        (0x2)
#define ADXL_REG_OFFSET_PLUS24        (0x3)
#define ADXL_REG_OFFSET_PLUS32        (0x4)
#define ADXL_REG_OFFSET_PLUS40        (0x5)
#define ADXL_REG_OFFSET_PLUS48        (0x6)
#define ADXL_REG_OFFSET_PLUS56        (0x7)
#define ADXL_REG_OFFSET_MINUS60       (0x8)
#define ADXL_REG_OFFSET_MINUS52       (0x9)
#define ADXL_REG_OFFSET_MINUS44       (0xA)
#define ADXL_REG_OFFSET_MINUS36       (0xB)
#define ADXL_REG_OFFSET_MINUS28       (0xC)
#define ADXL_REG_OFFSET_MINUS20       (0xD)
#define ADXL_REG_OFFSET_MINUS12       (0xE)
#define ADXL_REG_OFFSET_MINUS04       (0xF)
#define ADXL_REG_OFFSET_MASK          (0xF)

// 0x24 THRESH_ACT_X_L
#define ADXL_REG_THRESH_ACT_REF_REF   (0x02)
#define ADXL_REG_THRESH_ACT_REF_ABS   (0x00)
#define ADXL_REG_THRESH_ACT_X_EN      (0x01)
#define ADXL_REG_THRESH_ACT_X_DIS     (0x00)

// 0x26 THRESH_ACT_Y_L
#define ADXL_REG_THRESH_ACT_Y_EN      (0x01)
#define ADXL_REG_THRESH_ACT_Y_DIS     (0x00)

// 0x26 THRESH_ACT_Z_L
#define ADXL_REG_THRESH_ACT_Z_EN      (0x01)
#define ADXL_REG_THRESH_ACT_Z_DIS     (0x00)

// 0x29 TIME_ACT
// Activity time is measured in units of 3.3ms for ODR=6400Hz
// or 6.6ms for ODR=3200Hz
#define ADXL_REG_TIME_ACT_DEFAULT     (0x01)

// 0x2B THRESH_INACT_X_L
#define ADXL_REG_THRESH_INACT_X_EN    (0x01)
#define ADXL_REG_THRESH_INACT_X_DIS   (0x00)
#define ADXL_REG_THRESH_INACT_REF_REF (0x02)
#define ADXL_REG_THRESH_INACT_REF_ABS (0x00)

// 0x2D THRESH_INACT_Y_L
#define ADXL_REG_THRESH_INACT_Y_EN    (0x01)

// 0x2F THRESH_INACT_Z_L
#define ADXL_REG_THRESH_INACT_Z_EN    (0x01)

// 0x30 TIME_INACT_H
// Inactivity time is measured in units of 13ms for ODR=6400Hz
// or 26ms for ODR=3200Hz
#define ADXL_REG_TIME_INACT_H_DEFAULT (0x00)

// 0x3A FIFO_CTL
#define ADXL_REG_FIFO_CTL_FMT_MASK    (0x7 << 3)
#define ADXL_REG_FIFO_CTL_FMT_X       (0x1 << 3)
#define ADXL_REG_FIFO_CTL_FMT_Y       (0x2 << 3)
#define ADXL_REG_FIFO_CTL_FMT_Z       (0x4 << 3)
#define ADXL_REG_FIFO_CTL_FMT_XY      (0x3 << 3)
#define ADXL_REG_FIFO_CTL_FMT_XZ      (0x5 << 3)
#define ADXL_REG_FIFO_CTL_FMT_YZ      (0x6 << 3)
#define ADXL_REG_FIFO_CTL_FMT_XYZ     (0x0 << 3)
#define ADXL_REG_FIFO_CTL_FMT_PEAK    (0x7 << 3)
#define ADXL_REG_FIFO_CTL_MODE_MASK   (0x06)
#define ADXL_REG_FIFO_CTL_MODE_DIS    (0x00)
#define ADXL_REG_FIFO_CTL_MODE_STREAM (0x02)
#define ADXL_REG_FIFO_CTL_MODE_TRIG   (0x04)
#define ADXL_REG_FIFO_CTL_MODE_OLDEST (0x06)
#define ADXL_REG_FIFO_CTL_FIFO_SAMPLES_H (0x01)
#define ADXL_REG_FIFO_CTL_DEFAULT     (ADXL_REG_FIFO_CTL_MODE_OLDEST | ADXL_REG_FIFO_CTL_FMT_Y)

// 0x3B INT1_MAP
#define ADXL_REG_INT1_MAP_DATA_RDY    (0x01)
#define ADXL_REG_INT1_MAP_FIFO_RDY    (0x02)
#define ADXL_REG_INT1_MAP_FIFO_FULL   (0x04)
#define ADXL_REG_INT1_MAP_FIFO_OVR    (0x08)
#define ADXL_REG_INT1_MAP_INACT       (0x10)
#define ADXL_REG_INT1_MAP_ACT         (0x20)
#define ADXL_REG_INT1_MAP_AWAKE       (0x40)
#define ADXL_REG_INT1_MAP_ACTIVE_LOW  (0x80)

// 0x3C INT2_MAP
#define ADXL_REG_INT2_MAP_DATA_RDY    (0x01)
#define ADXL_REG_INT2_MAP_FIFO_RDY    (0x02)
#define ADXL_REG_INT2_MAP_FIFO_FULL   (0x04)
#define ADXL_REG_INT2_MAP_FIFO_OVR    (0x08)
#define ADXL_REG_INT2_MAP_INACT       (0x10)
#define ADXL_REG_INT2_MAP_ACT         (0x20)
#define ADXL_REG_INT2_MAP_AWAKE       (0x40)
#define ADXL_REG_INT2_MAP_ACTIVE_LOW  (0x80)

// 0x3D TIMING
#define ADXL_REG_TIMING_EXT_SYNC_EN   (0x01)
#define ADXL_REG_TIMING_EXT_SYNC_DIS  (0x00)
#define ADXL_REG_TIMING_EXT_CLK_EN    (0x02)
#define ADXL_REG_TIMING_EXT_CLK_DIS   (0x00)
#define ADXL_REG_TIMING_WKP_RATE_MASK (0x1C)
#define ADXL_REG_TIMING_ODR_MASK      (0xE0)
#define ADXL_REG_TIMING_ODR_400       (0x00)
#define ADXL_REG_TIMING_ODR_800       (0x20)
#define ADXL_REG_TIMING_ODR_1600      (0x40)
#define ADXL_REG_TIMING_ODR_3200      (0x60)
#define ADXL_REG_TIMING_ODR_6400      (0x80)
#define ADXL_REG_TIMING_DEFAULT       (ADXL_REG_TIMING_EXT_SYNC_EN | ADXL_REG_TIMING_EXT_CLK_DIS | ADXL_REG_TIMING_ODR_400)
//#define ADXL_REG_TIMING_DEFAULT       (ADXL_REG_TIMING_EXT_SYNC_DIS | ADXL_REG_TIMING_EXT_CLK_DIS | ADXL_REG_TIMING_ODR_400)

// 0x3E MEASURE
#define ADXL_REG_MEASURE_BW_200       (0x00)
#define ADXL_REG_MEASURE_BW_400       (0x01)
#define ADXL_REG_MEASURE_BW_800       (0x02)
#define ADXL_REG_MEASURE_BW_1600      (0x03)
#define ADXL_REG_MEASURE_BW_3200      (0x04)
#define ADXL_REG_MEASURE_LOW_NOISE    (0x08)
#define ADXL_REG_MEASURE_LINKLOOP_MASK    (0x30)
#define ADXL_REG_MEASURE_LINKLOOP_DEFAULT (0x00)
#define ADXL_REG_MEASURE_LINKLOOP_LINKED  (0x10)
#define ADXL_REG_MEASURE_LINKLOOP_LOOPED  (0x20)
#define ADXL_REG_MEASURE_USER_OR_DIS  (0x80)
#define ADXL_REG_MEASURE_USER_OR_EN   (0x00)
#define ADXL_REG_MEASURE_DEFAULT      (ADXL_REG_MEASURE_BW_200 | ADXL_REG_MEASURE_USER_OR_EN)

// 0x3F POWER_CTL
#define ADXL_REG_POWER_CTL_MODE_MASK  (0x03)
#define ADXL_REG_POWER_CTL_MODE_STBY  (0x00)
#define ADXL_REG_POWER_CTL_MODE_WKUP  (0x01)
#define ADXL_REG_POWER_CTL_MODE_INST  (0x02)
#define ADXL_REG_POWER_CTL_MODE_FULL  (0x03)
#define ADXL_REG_POWER_CTL_HPF_DIS    (0x04)
#define ADXL_REG_POWER_CTL_HPF_EN     (0x00)
#define ADXL_REG_POWER_CTL_LPF_DIS    (0x08)
#define ADXL_REG_POWER_CTL_LPF_EN     (0x00)
#define ADXL_REG_POWER_CTL_FILTER_SETTLE (0x10)
#define ADXL_REG_POWER_CTL_DEFAULT    (ADXL_REG_POWER_CTL_LPF_EN | ADXL_REG_POWER_CTL_HPF_DIS)

// 0x41 RESET
#define ADXL_REG_RESET_CODE           (0x52)

// Patch Layer
// Different devices order the bytes H/L in different directions.
#define ADXL_REG_XDATA_PATCH          ADXL_REG_XDATA_H
#define ADXL_REG_YDATA_PATCH          ADXL_REG_YDATA_H
#define ADXL_REG_ZDATA_PATCH          ADXL_REG_ZDATA_H
#define ADXL_REG_FIFO_ENTRIES_PATCH   ADXL_REG_FIFO_ENTRIES2
#define ADXL_REG_THRESH_ACT_X_PATCH   ADXL_REG_THRESH_ACT_X_H
#define ADXL_REG_THRESH_ACT_Y_PATCH   ADXL_REG_THRESH_ACT_Y_H
#define ADXL_REG_THRESH_ACT_Z_PATCH   ADXL_REG_THRESH_ACT_Z_H
#define ADXL_REG_THRESH_INACT_X_PATCH ADXL_REG_THRESH_INACT_X_H 
#define ADXL_REG_THRESH_INACT_Y_PATCH ADXL_REG_THRESH_INACT_Y_H 
#define ADXL_REG_THRESH_INACT_Z_PATCH ADXL_REG_THRESH_INACT_Z_H 
#define ADXL_REG_TIME_INACT_PATCH     ADXL_REG_TIME_INACT_H

// Data Conversions
// ADXL372 stores bits [11:4] in msb[7:0] and bits [3:0] in lsb[7:4]
#define __ADXL_DATA_PACK_UINT16(msb, lsb)       (((uint16_t)msb << 4) | ((uint16_t)(lsb & 0x00FF) >> 4))
// msb is in the lower register, lsb in the next higher
#define _ADXL_DATA_PACK_UINT16(pData)           (__ADXL_DATA_PACK_UINT16(*((uint8_t *)pData), *((uint8_t *)pData + 1)))

#define __ADXL_DATA_PACK_INT16(msb, lsb)        (int16_t)((msb & 0x80) ? __ADXL_DATA_PACK_UINT16(msb, lsb) | 0xF000 : __ADXL_DATA_PACK_UINT16(msb, lsb))
// Since data is contiguous, we should be able to do some casting/pointer magic
// Note! This may not work because of sign-extension in twos complement. Test it.
#define _ADXL_DATA_PACK_INT16(pData)            ((*(int16_t *)pData) >> 4)

// FIFO Entries Conversion
#define __ADXL_ENTRIES_PACK_UINT16(msb, lsb)    (((uint16_t)(msb & 0xFF) << 8) | (uint16_t)(lsb & 0xFF))
#define _ADXL_ENTRIES_PACK_UINT16(pData)        (__ADXL_ENTRIES_PACK_UINT16(*((uint8_t *)pData), *((uint8_t *)pData + 1)))

// Threshold Values (11-bit unsigned)
#define _ADXL_THRESHOLD_UNPACK_UINT16(n, pData)  do {*(uint8_t *)pData = (uint8_t)(((uint16_t)n >> 3) & 0xFF);\
                                                     *((uint8_t *)pData + 1) = (uint8_t)(((uint16_t)n & 0x07) << 5);} while (0)
#define _ADXL_THRESHOLD_PACK_UINT16(pData)      (((uint16_t)*(uint8_t *)pData) << 3 | ((uint16_t)*((uint8_t *)pData + 1) & 0xE0) >> 5)

// Inactivity Time Conversion (16-bit unsigned)
#define _ADXL_INACTIVITY_UNPACK_UINT16(n, pData) do {*(uint8_t *)pData = (uint8_t)(((uint16_t)n & 0xFF00) >> 8);\
                                                     *((uint8_t *)pData + 1) = (uint8_t)((uint16_t)n & 0x00FF);} while (0)
#define _ADXL_INACTIVITY_PACK_UINT16(pData)     (((uint16_t)*(uint8_t *)pData) << 8 | (uint16_t)*((uint8_t *)pData + 1))

// Device-Specific Limits
#define _ADXL_MAX_TRIGGER_FREQ       (6200)
//#define _ADXL_MAX_TRIGGER_FREQ       (3100)
// Triggering up to 6.2kHz requires setting ODR to 6400 Hz in the TIMING register


/* ======================= ADXL372 Exported Typedef ========================= */
typedef struct {
  // Includes only the writeable registers
  uint8_t rOFFSET_X;
  uint8_t rOFFSET_Y;
  uint8_t rOFFSET_Z;
  uint8_t rTHRESH_ACT_X_H;
  uint8_t rTHRESH_ACT_X_L;
  uint8_t rTHRESH_ACT_Y_H;
  uint8_t rTHRESH_ACT_Y_L;
  uint8_t rTHRESH_ACT_Z_H;
  uint8_t rTHRESH_ACT_Z_L;
  uint8_t rTIME_ACT;
  uint8_t rTHRESH_INACT_X_H;
  uint8_t rTHRESH_INACT_X_L;
  uint8_t rTHRESH_INACT_Y_H;
  uint8_t rTHRESH_INACT_Y_L;
  uint8_t rTHRESH_INACT_Z_H;
  uint8_t rTHRESH_INACT_Z_L;
  uint8_t rTIME_INACT_H;
  uint8_t rTIME_INACT_L;
  uint8_t rTHRESH_ACT2_X_H;
  uint8_t rTHRESH_ACT2_X_L;
  uint8_t rTHRESH_ACT2_Y_H;
  uint8_t rTHRESH_ACT2_Y_L;
  uint8_t rTHRESH_ACT2_Z_H;
  uint8_t rTHRESH_ACT2_Z_L;
  uint8_t rHPF;
  uint8_t rFIFO_SAMPLES;
  uint8_t rFIFO_CTL;
  uint8_t rINT1_MAP;
  uint8_t rINT2_MAP;
  uint8_t rTIMING;
  uint8_t rMEASURE;
  uint8_t rPOWER_CTL;
} adxl_reg_t;

#define _ADXL_REG_START_OFFSET        (0x20)
#define _ADXL_REG_END_OFFSET          (0x3F)
#ifdef __cplusplus
}
#endif

#endif //__ADXL372_REGMAP_H

