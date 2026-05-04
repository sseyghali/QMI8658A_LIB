#ifndef QMI8658A_H
#define QMI8658A_H

/**
 * @file QMI8658A.h
 * @brief QST QMI8658A 6-axis IMU driver.
 *
 * The API is intentionally close to the supplied MC3416 driver: a small device
 * object owns the bus interface pointer, configuration functions write the
 * corresponding registers, and read functions return either raw counts or
 * scaled physical units.
 */

#include <stdint.h>
#include <stdbool.h>
#include "QMI8658A_REG.h"
#include "QMI8658A_PORTS.h"
#include "io_driver/accel_io.h"

typedef struct
{
    uint8_t    address;
    accelIO_t *interface;
    qmi8658a_accel_range_t accel_range;
    qmi8658a_gyro_range_t  gyro_range;
    qmi8658a_ctrl7_t       enabled_sensors;
} qmi8658a_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} qmi8658a_raw_axes_t;

typedef struct
{
    float x;
    float y;
    float z;
} qmi8658a_float_axes_t;

typedef struct
{
    qmi8658a_float_axes_t accel_g;
    qmi8658a_float_axes_t gyro_dps;
    float temperature_c;
    uint32_t timestamp;
} qmi8658a_sample_t;

typedef struct
{
    uint8_t statusint;
    uint8_t status0;
    uint8_t status1;
} qmi8658a_status_t;

typedef struct
{
    uint8_t full;
    uint8_t watermark;
    uint8_t overflow;
    uint8_t not_empty;
    uint16_t byte_count;
} qmi8658a_fifo_status_t;

typedef struct
{
    uint8_t polarity;
    uint8_t axis;
    uint8_t count;
} qmi8658a_tap_status_t;

typedef struct
{
    uint8_t fw_version[3];
    uint8_t usid[6];
} qmi8658a_identity_t;

typedef struct
{
    uint8_t any_x_thr;
    uint8_t any_y_thr;
    uint8_t any_z_thr;
    uint8_t no_x_thr;
    uint8_t no_y_thr;
    uint8_t no_z_thr;
    uint8_t mode_ctrl;
    uint8_t any_window;
    uint8_t no_window;
    uint16_t sig_wait_window;
    uint16_t sig_confirm_window;
} qmi8658a_motion_config_t;

typedef struct
{
    uint8_t peak_window;
    uint8_t priority;
    uint16_t tap_window;
    uint16_t double_tap_window;
    uint8_t alpha;
    uint8_t gamma;
    uint16_t peak_mag_thr;
    uint16_t udm_thr;
} qmi8658a_tap_config_t;

typedef struct
{
    uint16_t sample_count;
    uint16_t fix_peak2peak;
    uint16_t fix_peak;
    uint16_t time_up;
    uint8_t time_low;
    uint8_t cnt_entry;
    uint8_t fix_precision;
    uint8_t sig_count;
} qmi8658a_pedometer_config_t;

qmi8658a_t *QMI8658A_Init(accelIO_t *io, uint8_t adr);
void QMI8658A_Deinit(qmi8658a_t *dev);

bool QMI8658A_ReadChipID(qmi8658a_t *dev, uint8_t *who_am_i, uint8_t *revision_id);
bool QMI8658A_Reset(qmi8658a_t *dev);
bool QMI8658A_SetInterface(qmi8658a_t *dev, qmi8658a_ctrl1_t cfg);
bool QMI8658A_SetAccelConfig(qmi8658a_t *dev, qmi8658a_accel_range_t range, qmi8658a_accel_odr_t odr, bool self_test);
bool QMI8658A_SetGyroConfig(qmi8658a_t *dev, qmi8658a_gyro_range_t range, qmi8658a_gyro_odr_t odr, bool self_test);
bool QMI8658A_SetFilter(qmi8658a_t *dev, qmi8658a_lpf_mode_t accel_lpf, bool accel_enable,
                        qmi8658a_lpf_mode_t gyro_lpf, bool gyro_enable);
bool QMI8658A_EnableSensors(qmi8658a_t *dev, qmi8658a_ctrl7_t sensors);
bool QMI8658A_SetActivityControl(qmi8658a_t *dev, qmi8658a_ctrl8_t cfg);

bool QMI8658A_ReadStatus(qmi8658a_t *dev, qmi8658a_status_t *status);
bool QMI8658A_ReadTimestamp(qmi8658a_t *dev, uint32_t *timestamp);
bool QMI8658A_ReadTemperature(qmi8658a_t *dev, float *temperature_c);
bool QMI8658A_ReadRawAccel(qmi8658a_t *dev, qmi8658a_raw_axes_t *raw);
bool QMI8658A_ReadRawGyro(qmi8658a_t *dev, qmi8658a_raw_axes_t *raw);
bool QMI8658A_ReadAccel(qmi8658a_t *dev, qmi8658a_float_axes_t *accel_g);
bool QMI8658A_ReadGyro(qmi8658a_t *dev, qmi8658a_float_axes_t *gyro_dps);
bool QMI8658A_ReadSample(qmi8658a_t *dev, qmi8658a_sample_t *sample);

bool QMI8658A_Ctrl9(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd);
bool QMI8658A_Ctrl9Write(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd, const uint8_t cal[QMI8658A_CAL_REG_NUM]);
bool QMI8658A_Ctrl9Read(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd, uint8_t cal[QMI8658A_CAL_REG_NUM]);
bool QMI8658A_WriteAccelOffset(qmi8658a_t *dev, int16_t x, int16_t y, int16_t z);
bool QMI8658A_WriteGyroOffset(qmi8658a_t *dev, int16_t x, int16_t y, int16_t z);
bool QMI8658A_SetPullups(qmi8658a_t *dev, uint8_t disable_mask);
bool QMI8658A_SetAhbClockGating(qmi8658a_t *dev, bool enable);
bool QMI8658A_ReadFwVersionUsid(qmi8658a_t *dev, qmi8658a_identity_t *identity);
bool QMI8658A_RunCOD(qmi8658a_t *dev, uint8_t *cod_status);
bool QMI8658A_ApplyGyroGains(qmi8658a_t *dev, uint16_t gx, uint16_t gy, uint16_t gz);

bool QMI8658A_FIFOConfig(qmi8658a_t *dev, uint8_t watermark, qmi8658a_fifo_size_t size, qmi8658a_fifo_mode_t mode);
bool QMI8658A_FIFOReset(qmi8658a_t *dev);
bool QMI8658A_FIFOGetStatus(qmi8658a_t *dev, qmi8658a_fifo_status_t *fifo_status);
bool QMI8658A_FIFORead(qmi8658a_t *dev, uint8_t *buffer, uint16_t max_len, uint16_t *read_len);

bool QMI8658A_ConfigureMotion(qmi8658a_t *dev, const qmi8658a_motion_config_t *cfg);
bool QMI8658A_EnableMotion(qmi8658a_t *dev, bool any_motion, bool no_motion, bool significant_motion);
bool QMI8658A_ConfigureTap(qmi8658a_t *dev, const qmi8658a_tap_config_t *cfg);
bool QMI8658A_EnableTap(qmi8658a_t *dev, bool enable);
bool QMI8658A_ReadTapStatus(qmi8658a_t *dev, qmi8658a_tap_status_t *tap);
bool QMI8658A_ConfigurePedometer(qmi8658a_t *dev, const qmi8658a_pedometer_config_t *cfg);
bool QMI8658A_EnablePedometer(qmi8658a_t *dev, bool enable);
bool QMI8658A_ReadStepCount(qmi8658a_t *dev, uint32_t *steps);
bool QMI8658A_ResetStepCount(qmi8658a_t *dev);
bool QMI8658A_ConfigureWakeOnMotion(qmi8658a_t *dev, uint8_t threshold_mg,
                                    qmi8658a_wom_int_t int_cfg, uint8_t blanking_samples);
bool QMI8658A_DisableWakeOnMotion(qmi8658a_t *dev);

bool QMI8658A_RunAccelSelfTest(qmi8658a_t *dev, qmi8658a_raw_axes_t *delta_raw, bool *passed);
bool QMI8658A_RunGyroSelfTest(qmi8658a_t *dev, qmi8658a_raw_axes_t *delta_raw, bool *passed);

#endif /* QMI8658A_H */
