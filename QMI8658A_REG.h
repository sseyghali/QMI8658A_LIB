#ifndef QMI8658A_REG_H
#define QMI8658A_REG_H

#include <stdint.h>

#define uint_dev    uint8_t
#define qmi8658a_buffer(A)                                                   \
    union                                                                    \
    {                                                                        \
        uint32_t words[((A) + 3) >> 2];                                      \
        uint8_t  bytes[(A)];                                                 \
    }

#define QMI8658A_AXIS_X                    0
#define QMI8658A_AXIS_Y                    1
#define QMI8658A_AXIS_Z                    2
#define QMI8658A_AXES_NUM                  3
#define QMI8658A_CAL_REG_NUM               8
#define QMI8658A_WHO_AM_I_VALUE            0x05
#define QMI8658A_RESET_CMD                 0xB0

/* Gravity constants are kept from the initial project header for callers that
 * prefer m/s^2 output instead of g output. */
#define QMI8658A_GRAVITY_SUN               273.95f
#define QMI8658A_GRAVITY_EARTH             9.80665f
#define QMI8658A_GRAVITY_MOON              1.622f
#define QMI8658A_GRAVITY_MARS              3.69f
#define QMI8658A_GRAVITY_NONE              1.00f

/* 7-bit I2C/I3C addresses.  SA0 high or floating selects 0x6A; SA0 low selects
 * 0x6B.  The datasheet notes an internal weak pull-up on SA0 during reset. */
#define QMI8658A_ADDRESS_HIGH_LEVEL        0x6A
#define QMI8658A_ADDRESS_LOW_LEVEL         0x6B

typedef enum
{
    QMI8658A_WHO_AM_I       = 0x00,
    QMI8658A_REVISION_ID    = 0x01,
    QMI8658A_CTRL_1         = 0x02,
    QMI8658A_CTRL_2         = 0x03,
    QMI8658A_CTRL_3         = 0x04,
    QMI8658A_CTRL_5         = 0x06,
    QMI8658A_CTRL_7         = 0x08,
    QMI8658A_CTRL_8         = 0x09,
    QMI8658A_CTRL_9         = 0x0A,
    QMI8658A_CAL1_L         = 0x0B,
    QMI8658A_CAL1_H         = 0x0C,
    QMI8658A_CAL2_L         = 0x0D,
    QMI8658A_CAL2_H         = 0x0E,
    QMI8658A_CAL3_L         = 0x0F,
    QMI8658A_CAL3_H         = 0x10,
    QMI8658A_CAL4_L         = 0x11,
    QMI8658A_CAL4_H         = 0x12,
    QMI8658A_FIFO_WTM_TH    = 0x13,
    QMI8658A_FIFO_CTRL      = 0x14,
    QMI8658A_FIFO_SMPL_CNT  = 0x15,
    QMI8658A_FIFO_STATUS    = 0x16,
    QMI8658A_FIFO_DATA      = 0x17,
    QMI8658A_STATUSINT      = 0x2D,
    QMI8658A_STATUS0        = 0x2E,
    QMI8658A_STATUS1        = 0x2F,
    QMI8658A_TIMESTAMP_LOW  = 0x30,
    QMI8658A_TIMESTAMP_MID  = 0x31,
    QMI8658A_TIMESTAMP_HIGH = 0x32,
    QMI8658A_TEMP_L         = 0x33,
    QMI8658A_TEMP_H         = 0x34,
    QMI8658A_AX_L           = 0x35,
    QMI8658A_AX_H           = 0x36,
    QMI8658A_AY_L           = 0x37,
    QMI8658A_AY_H           = 0x38,
    QMI8658A_AZ_L           = 0x39,
    QMI8658A_AZ_H           = 0x3A,
    QMI8658A_GX_L           = 0x3B,
    QMI8658A_GX_H           = 0x3C,
    QMI8658A_GY_L           = 0x3D,
    QMI8658A_GY_H           = 0x3E,
    QMI8658A_GZ_L           = 0x3F,
    QMI8658A_GZ_H           = 0x40,
    QMI8658A_COD_STATUS     = 0x46,
    QMI8658A_dQW_L          = 0x49,
    QMI8658A_dQW_H          = 0x4A,
    QMI8658A_dQX_L          = 0x4B,
    QMI8658A_dQX_H          = 0x4C,
    QMI8658A_dQY_L          = 0x4D,
    QMI8658A_dQY_H          = 0x4E,
    QMI8658A_dQZ_L          = 0x4F,
    QMI8658A_dQZ_H          = 0x50,
    QMI8658A_dVX_L          = 0x51,
    QMI8658A_dVX_H          = 0x52,
    QMI8658A_dVY_L          = 0x53,
    QMI8658A_dVY_H          = 0x54,
    QMI8658A_dVZ_L          = 0x55,
    QMI8658A_dVZ_H          = 0x56,
    QMI8658A_TAP_STATUS     = 0x59,
    QMI8658A_STEP_CNT_LOW   = 0x5A,
    QMI8658A_STEP_CNT_MIDL  = 0x5B,
    QMI8658A_STEP_CNT_HIGH  = 0x5C,
    QMI8658A_RESET          = 0x60
} qmi8658a_reg_t;

typedef enum
{
    QMI8658A_SPI_4_WIRE_ENABLE   = 0x00,
    QMI8658A_SPI_3_WIRE_ENABLE   = 0x80,
    QMI8658A_ADDR_NON_INCREMENT  = 0x00,
    QMI8658A_ADDR_AUTO_INCREMENT = 0x40,
    QMI8658A_LITTLE_ENDIAN       = 0x00,
    QMI8658A_BIG_ENDIAN          = 0x20,
    QMI8658A_INT2_DISABLE        = 0x00,
    QMI8658A_INT2_ENABLE         = 0x10,
    QMI8658A_INT1_DISABLE        = 0x00,
    QMI8658A_INT1_ENABLE         = 0x08,
    QMI8658A_FIFO_INT_TO_INT2    = 0x00,
    QMI8658A_FIFO_INT_TO_INT1    = 0x04,
    QMI8658A_OSC_ENABLE          = 0x00,
    QMI8658A_OSC_DISABLE         = 0x01
} qmi8658a_ctrl1_t;

typedef enum
{
    QMI8658A_ACCEL_RANGE_2G  = 0x00,
    QMI8658A_ACCEL_RANGE_4G  = 0x01,
    QMI8658A_ACCEL_RANGE_8G  = 0x02,
    QMI8658A_ACCEL_RANGE_16G = 0x03
} qmi8658a_accel_range_t;

typedef enum
{
    QMI8658A_ACCEL_ODR_1000HZ = 0x03,
    QMI8658A_ACCEL_ODR_500HZ  = 0x04,
    QMI8658A_ACCEL_ODR_250HZ  = 0x05,
    QMI8658A_ACCEL_ODR_125HZ  = 0x06,
    QMI8658A_ACCEL_ODR_62_5HZ = 0x07,
    QMI8658A_ACCEL_ODR_31_25HZ = 0x08,
    QMI8658A_ACCEL_ODR_LOW_POWER_128HZ = 0x0C,
    QMI8658A_ACCEL_ODR_LOW_POWER_21HZ  = 0x0D,
    QMI8658A_ACCEL_ODR_LOW_POWER_11HZ  = 0x0E,
    QMI8658A_ACCEL_ODR_LOW_POWER_3HZ   = 0x0F
} qmi8658a_accel_odr_t;

typedef enum
{
    QMI8658A_GYRO_RANGE_16DPS   = 0x00,
    QMI8658A_GYRO_RANGE_32DPS   = 0x01,
    QMI8658A_GYRO_RANGE_64DPS   = 0x02,
    QMI8658A_GYRO_RANGE_128DPS  = 0x03,
    QMI8658A_GYRO_RANGE_256DPS  = 0x04,
    QMI8658A_GYRO_RANGE_512DPS  = 0x05,
    QMI8658A_GYRO_RANGE_1024DPS = 0x06,
    QMI8658A_GYRO_RANGE_2048DPS = 0x07
} qmi8658a_gyro_range_t;

typedef enum
{
    QMI8658A_GYRO_ODR_7174_4HZ = 0x00,
    QMI8658A_GYRO_ODR_3587_2HZ = 0x01,
    QMI8658A_GYRO_ODR_1793_6HZ = 0x02,
    QMI8658A_GYRO_ODR_896_8HZ  = 0x03,
    QMI8658A_GYRO_ODR_448_4HZ  = 0x04,
    QMI8658A_GYRO_ODR_224_2HZ  = 0x05,
    QMI8658A_GYRO_ODR_112_1HZ  = 0x06,
    QMI8658A_GYRO_ODR_56_05HZ  = 0x07,
    QMI8658A_GYRO_ODR_28_025HZ = 0x08
} qmi8658a_gyro_odr_t;

typedef enum
{
    QMI8658A_LPF_MODE_2_66_PERCENT  = 0x00,
    QMI8658A_LPF_MODE_3_63_PERCENT  = 0x01,
    QMI8658A_LPF_MODE_5_39_PERCENT  = 0x02,
    QMI8658A_LPF_MODE_13_37_PERCENT = 0x03
} qmi8658a_lpf_mode_t;

typedef enum
{
    QMI8658A_SYNCSAMPLE_DISABLE = 0x00,
    QMI8658A_SYNCSAMPLE_ENABLE  = 0x80,
    QMI8658A_DRDY_ENABLE        = 0x00,
    QMI8658A_DRDY_DISABLE       = 0x20,
    QMI8658A_GYRO_FULL_MODE     = 0x00,
    QMI8658A_GYRO_SNOOZE_MODE   = 0x10,
    QMI8658A_GYRO_DISABLE       = 0x00,
    QMI8658A_GYRO_ENABLE        = 0x02,
    QMI8658A_ACCEL_DISABLE      = 0x00,
    QMI8658A_ACCEL_ENABLE       = 0x01
} qmi8658a_ctrl7_t;

typedef enum
{
    QMI8658A_CTRL9_HANDSHAKE_INT1      = 0x00,
    QMI8658A_CTRL9_HANDSHAKE_STATUSINT = 0x80,
    QMI8658A_ACTIVITY_INT_TO_INT2      = 0x00,
    QMI8658A_ACTIVITY_INT_TO_INT1      = 0x40,
    QMI8658A_PEDOMETER_ENABLE          = 0x10,
    QMI8658A_SIGNIFICANT_MOTION_ENABLE = 0x08,
    QMI8658A_NO_MOTION_ENABLE          = 0x04,
    QMI8658A_ANY_MOTION_ENABLE         = 0x02,
    QMI8658A_TAP_ENABLE                = 0x01
} qmi8658a_ctrl8_t;

typedef enum
{
    QMI8658A_FIFO_SIZE_16_SAMPLES  = 0x00,
    QMI8658A_FIFO_SIZE_32_SAMPLES  = 0x01,
    QMI8658A_FIFO_SIZE_64_SAMPLES  = 0x02,
    QMI8658A_FIFO_SIZE_128_SAMPLES = 0x03
} qmi8658a_fifo_size_t;

typedef enum
{
    QMI8658A_FIFO_MODE_BYPASS = 0x00,
    QMI8658A_FIFO_MODE_FIFO   = 0x01,
    QMI8658A_FIFO_MODE_STREAM = 0x02
} qmi8658a_fifo_mode_t;

typedef enum
{
    QMI8658A_CTRL_CMD_ACK                       = 0x00,
    QMI8658A_CTRL_CMD_RST_FIFO                  = 0x04,
    QMI8658A_CTRL_CMD_REQ_FIFO                  = 0x05,
    QMI8658A_CTRL_CMD_WRITE_WOM_SETTING         = 0x08,
    QMI8658A_CTRL_CMD_ACCEL_HOST_DELTA_OFFSET   = 0x09,
    QMI8658A_CTRL_CMD_GYRO_HOST_DELTA_OFFSET    = 0x0A,
    QMI8658A_CTRL_CMD_CONFIGURE_TAP             = 0x0C,
    QMI8658A_CTRL_CMD_CONFIGURE_PEDOMETER       = 0x0D,
    QMI8658A_CTRL_CMD_CONFIGURE_MOTION          = 0x0E,
    QMI8658A_CTRL_CMD_RESET_PEDOMETER           = 0x0F,
    QMI8658A_CTRL_CMD_COPY_USID                 = 0x10,
    QMI8658A_CTRL_CMD_SET_RPU                   = 0x11,
    QMI8658A_CTRL_CMD_AHB_CLOCK_GATING          = 0x12,
    QMI8658A_CTRL_CMD_ON_DEMAND_CALIBRATION     = 0xA2,
    QMI8658A_CTRL_CMD_APPLY_GYRO_GAINS          = 0xAA
} qmi8658a_ctrl9_cmd_t;

typedef enum
{
    QMI8658A_WOM_INT1_INITIAL_LOW  = 0x00,
    QMI8658A_WOM_INT2_INITIAL_LOW  = 0x01,
    QMI8658A_WOM_INT1_INITIAL_HIGH = 0x02,
    QMI8658A_WOM_INT2_INITIAL_HIGH = 0x03
} qmi8658a_wom_int_t;

#define QMI8658A_CTRL2_VALUE(range, odr, st) \
    ((uint8_t)(((st) ? 0x80 : 0x00) | (((uint8_t)(range) & 0x07) << 4) | ((uint8_t)(odr) & 0x0F)))
#define QMI8658A_CTRL3_VALUE(range, odr, st) \
    ((uint8_t)(((st) ? 0x80 : 0x00) | (((uint8_t)(range) & 0x07) << 4) | ((uint8_t)(odr) & 0x0F)))
#define QMI8658A_CTRL5_VALUE(a_mode, a_en, g_mode, g_en) \
    ((uint8_t)((((uint8_t)(g_mode) & 0x03) << 5) | ((g_en) ? 0x10 : 0x00) | \
               (((uint8_t)(a_mode) & 0x03) << 1) | ((a_en) ? 0x01 : 0x00)))
#define QMI8658A_FIFO_CTRL_VALUE(size, mode) \
    ((uint8_t)((((uint8_t)(size) & 0x03) << 2) | ((uint8_t)(mode) & 0x03)))

#define QMI8658A_STATUSINT_CMD_DONE(reg)     (((reg) >> 7) & 0x01)
#define QMI8658A_STATUSINT_LOCKED(reg)       (((reg) >> 1) & 0x01)
#define QMI8658A_STATUSINT_AVAILABLE(reg)    ((reg) & 0x01)
#define QMI8658A_STATUS0_GYRO_AVAILABLE(reg) (((reg) >> 1) & 0x01)
#define QMI8658A_STATUS0_ACCEL_AVAILABLE(reg) ((reg) & 0x01)
#define QMI8658A_STATUS1_SIG_MOTION(reg)     (((reg) >> 7) & 0x01)
#define QMI8658A_STATUS1_NO_MOTION(reg)      (((reg) >> 6) & 0x01)
#define QMI8658A_STATUS1_ANY_MOTION(reg)     (((reg) >> 5) & 0x01)
#define QMI8658A_STATUS1_PEDOMETER(reg)      (((reg) >> 4) & 0x01)
#define QMI8658A_STATUS1_WOM(reg)            (((reg) >> 2) & 0x01)
#define QMI8658A_STATUS1_TAP(reg)            (((reg) >> 1) & 0x01)
#define QMI8658A_FIFO_STATUS_FULL(reg)       (((reg) >> 7) & 0x01)
#define QMI8658A_FIFO_STATUS_WTM(reg)        (((reg) >> 6) & 0x01)
#define QMI8658A_FIFO_STATUS_OVERFLOW(reg)   (((reg) >> 5) & 0x01)
#define QMI8658A_FIFO_STATUS_NOT_EMPTY(reg)  (((reg) >> 4) & 0x01)
#define QMI8658A_TAP_STATUS_POLARITY(reg)    (((reg) >> 7) & 0x01)
#define QMI8658A_TAP_STATUS_AXIS(reg)        (((reg) >> 4) & 0x03)
#define QMI8658A_TAP_STATUS_COUNT(reg)       ((reg) & 0x03)

#endif /* QMI8658A_REG_H */
