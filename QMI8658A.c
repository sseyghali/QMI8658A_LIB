/**
 * @file QMI8658A.c
 * @brief QST QMI8658A 6-axis IMU driver implementation.
 */
#include "QMI8658A.h"

//#define DEBUG_MODE 0

#ifdef DEBUG_MODE
#include <DebugTask.h>
#define QMI_PRINTF(...)     debugf(Window33, __VA_ARGS__)
#else
#define QMI_PRINTF(...)
#endif

#define QMI8658A_DEFAULT_ACCEL_RANGE    QMI8658A_ACCEL_RANGE_2G
#define QMI8658A_DEFAULT_ACCEL_ODR      QMI8658A_ACCEL_ODR_250HZ
#define QMI8658A_DEFAULT_GYRO_RANGE     QMI8658A_GYRO_RANGE_256DPS
#define QMI8658A_DEFAULT_GYRO_ODR       QMI8658A_GYRO_ODR_224_2HZ
#define QMI8658A_CTRL9_TIMEOUT_MS       200
#define QMI8658A_COD_TIMEOUT_MS         1700

static void _QMI8658A_REG_WRITE(qmi8658a_t *dev, uint8_t bRegAddr, const uint8_t *pbDataBuf, uint8_t bLength)
{
    uint8_t i = 0;

    for (i = 0; i < bLength; i++)
        dev->interface->write_reg(dev->address, bRegAddr + i, pbDataBuf[i]);
}

static void _QMI8658A_REG_READ(qmi8658a_t *dev, uint8_t bRegAddr, uint8_t *pbDataBuf, uint8_t bLength)
{
    uint8_t i = 0;

    for (i = 0; i < bLength; i++)
        pbDataBuf[i] = dev->interface->read_reg(dev->address, bRegAddr + i);
}

static bool _QMI8658A_ValidDev(qmi8658a_t *dev)
{
    return (dev != 0) && (dev->interface != 0);
}

static int16_t _QMI8658A_S16(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static void _QMI8658A_U16_TO_CAL(uint8_t *cal, uint8_t low_index, uint16_t value)
{
    cal[low_index] = (uint8_t)(value & 0xFF);
    cal[low_index + 1] = (uint8_t)((value >> 8) & 0xFF);
}

static float _QMI8658A_AccelRangeG(qmi8658a_accel_range_t range)
{
    switch (range)
    {
        case QMI8658A_ACCEL_RANGE_4G:  return 4.0f;
        case QMI8658A_ACCEL_RANGE_8G:  return 8.0f;
        case QMI8658A_ACCEL_RANGE_16G: return 16.0f;
        case QMI8658A_ACCEL_RANGE_2G:
        default:                       return 2.0f;
    }
}

static float _QMI8658A_GyroRangeDps(qmi8658a_gyro_range_t range)
{
    switch (range)
    {
        case QMI8658A_GYRO_RANGE_32DPS:   return 32.0f;
        case QMI8658A_GYRO_RANGE_64DPS:   return 64.0f;
        case QMI8658A_GYRO_RANGE_128DPS:  return 128.0f;
        case QMI8658A_GYRO_RANGE_256DPS:  return 256.0f;
        case QMI8658A_GYRO_RANGE_512DPS:  return 512.0f;
        case QMI8658A_GYRO_RANGE_1024DPS: return 1024.0f;
        case QMI8658A_GYRO_RANGE_2048DPS: return 2048.0f;
        case QMI8658A_GYRO_RANGE_16DPS:
        default:                          return 16.0f;
    }
}

static bool _QMI8658A_WaitCtrl9Done(qmi8658a_t *dev, uint16_t timeout_ms)
{
    uint8_t status = 0;
    uint16_t elapsed = 0;

    while (elapsed < timeout_ms)
    {
        _QMI8658A_REG_READ(dev, QMI8658A_STATUSINT, &status, 1);
        if (QMI8658A_STATUSINT_CMD_DONE(status))
            return true;

        qmi8658a_Delay(1);
        elapsed++;
    }

    return false;
}

static bool _QMI8658A_WriteCalAndRun(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd, const uint8_t cal[QMI8658A_CAL_REG_NUM])
{
    if (!_QMI8658A_ValidDev(dev) || (cal == 0))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CAL1_L, cal, QMI8658A_CAL_REG_NUM);
    return QMI8658A_Ctrl9(dev, cmd);
}

qmi8658a_t *QMI8658A_Init(accelIO_t *io, uint8_t adr)
{
    uint8_t chip_id = 0;
    qmi8658a_t *dev = 0;

    if (io == 0)
        return 0;

    dev = qmi8658a_malloc(sizeof(qmi8658a_t));
    if (dev == 0)
        return 0;

    dev->address = adr;
    dev->interface = io;
    dev->accel_range = QMI8658A_DEFAULT_ACCEL_RANGE;
    dev->gyro_range = QMI8658A_DEFAULT_GYRO_RANGE;
    dev->enabled_sensors = QMI8658A_ACCEL_DISABLE | QMI8658A_GYRO_DISABLE;
    dev->interface->init(adr);

    _QMI8658A_REG_READ(dev, QMI8658A_WHO_AM_I, &chip_id, 1);
    if (chip_id != QMI8658A_WHO_AM_I_VALUE)
    {
        QMI_PRINTF("[%s] unexpected CHIP ID %X\r\n", __func__, chip_id);
    }

    QMI8658A_SetInterface(dev, (qmi8658a_ctrl1_t)(QMI8658A_ADDR_AUTO_INCREMENT |
                                                  QMI8658A_LITTLE_ENDIAN |
                                                  QMI8658A_INT1_DISABLE |
                                                  QMI8658A_INT2_DISABLE |
                                                  QMI8658A_OSC_ENABLE));
    qmi8658a_Delay(1);
    QMI8658A_SetAccelConfig(dev, QMI8658A_DEFAULT_ACCEL_RANGE, QMI8658A_DEFAULT_ACCEL_ODR, false);
    qmi8658a_Delay(1);
    QMI8658A_SetGyroConfig(dev, QMI8658A_DEFAULT_GYRO_RANGE, QMI8658A_DEFAULT_GYRO_ODR, false);
    qmi8658a_Delay(1);
    QMI8658A_SetFilter(dev, QMI8658A_LPF_MODE_2_66_PERCENT, false, QMI8658A_LPF_MODE_2_66_PERCENT, false);
    qmi8658a_Delay(1);

    return dev;
}

void QMI8658A_Deinit(qmi8658a_t *dev)
{
    if (dev != 0)
        qmi8658a_free(dev);
}

bool QMI8658A_ReadChipID(qmi8658a_t *dev, uint8_t *who_am_i, uint8_t *revision_id)
{
    if (!_QMI8658A_ValidDev(dev))
        return false;

    if (who_am_i != 0)
        _QMI8658A_REG_READ(dev, QMI8658A_WHO_AM_I, who_am_i, 1);
    if (revision_id != 0)
        _QMI8658A_REG_READ(dev, QMI8658A_REVISION_ID, revision_id, 1);

    return true;
}

bool QMI8658A_Reset(qmi8658a_t *dev)
{
    uint8_t reset = QMI8658A_RESET_CMD;

    if (!_QMI8658A_ValidDev(dev))
        return false;

    /* Rev A has one prose line that says 0x0B, while the reset-register table
     * specifies 0xB0.  QST sample drivers and the table use 0xB0. */
    _QMI8658A_REG_WRITE(dev, QMI8658A_RESET, &reset, 1);
    qmi8658a_Delay(15);
    dev->enabled_sensors = QMI8658A_ACCEL_DISABLE | QMI8658A_GYRO_DISABLE;
    return true;
}

bool QMI8658A_SetInterface(qmi8658a_t *dev, qmi8658a_ctrl1_t cfg)
{
    uint8_t reg = (uint8_t)cfg;

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_1, &reg, 1);
    return true;
}

bool QMI8658A_SetAccelConfig(qmi8658a_t *dev, qmi8658a_accel_range_t range, qmi8658a_accel_odr_t odr, bool self_test)
{
    uint8_t reg = QMI8658A_CTRL2_VALUE(range, odr, self_test);

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_2, &reg, 1);
    dev->accel_range = range;
    return true;
}

bool QMI8658A_SetGyroConfig(qmi8658a_t *dev, qmi8658a_gyro_range_t range, qmi8658a_gyro_odr_t odr, bool self_test)
{
    uint8_t reg = QMI8658A_CTRL3_VALUE(range, odr, self_test);

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_3, &reg, 1);
    dev->gyro_range = range;
    return true;
}

bool QMI8658A_SetFilter(qmi8658a_t *dev, qmi8658a_lpf_mode_t accel_lpf, bool accel_enable,
                        qmi8658a_lpf_mode_t gyro_lpf, bool gyro_enable)
{
    uint8_t reg = QMI8658A_CTRL5_VALUE(accel_lpf, accel_enable, gyro_lpf, gyro_enable);

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_5, &reg, 1);
    return true;
}

bool QMI8658A_EnableSensors(qmi8658a_t *dev, qmi8658a_ctrl7_t sensors)
{
    uint8_t reg = (uint8_t)sensors;

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_7, &reg, 1);
    dev->enabled_sensors = sensors;
    return true;
}

bool QMI8658A_SetActivityControl(qmi8658a_t *dev, qmi8658a_ctrl8_t cfg)
{
    uint8_t reg = (uint8_t)cfg;

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_8, &reg, 1);
    return true;
}

bool QMI8658A_ReadStatus(qmi8658a_t *dev, qmi8658a_status_t *status)
{
    uint8_t buf[3] = {0};

    if (!_QMI8658A_ValidDev(dev) || (status == 0))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_STATUSINT, buf, 3);
    status->statusint = buf[0];
    status->status0 = buf[1];
    status->status1 = buf[2];
    return true;
}

bool QMI8658A_ReadTimestamp(qmi8658a_t *dev, uint32_t *timestamp)
{
    uint8_t buf[3] = {0};

    if (!_QMI8658A_ValidDev(dev) || (timestamp == 0))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_TIMESTAMP_LOW, buf, 3);
    *timestamp = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
    return true;
}

bool QMI8658A_ReadTemperature(qmi8658a_t *dev, float *temperature_c)
{
    uint8_t buf[2] = {0};

    if (!_QMI8658A_ValidDev(dev) || (temperature_c == 0))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_TEMP_L, buf, 2);
    *temperature_c = (float)((int8_t)buf[1]) + ((float)buf[0] / 256.0f);
    return true;
}

bool QMI8658A_ReadRawAccel(qmi8658a_t *dev, qmi8658a_raw_axes_t *raw)
{
    uint8_t buf[6] = {0};

    if (!_QMI8658A_ValidDev(dev) || (raw == 0))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_AX_L, buf, 6);
    raw->x = _QMI8658A_S16(&buf[0]);
    raw->y = _QMI8658A_S16(&buf[2]);
    raw->z = _QMI8658A_S16(&buf[4]);
    return true;
}

bool QMI8658A_ReadRawGyro(qmi8658a_t *dev, qmi8658a_raw_axes_t *raw)
{
    uint8_t buf[6] = {0};

    if (!_QMI8658A_ValidDev(dev) || (raw == 0))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_GX_L, buf, 6);
    raw->x = _QMI8658A_S16(&buf[0]);
    raw->y = _QMI8658A_S16(&buf[2]);
    raw->z = _QMI8658A_S16(&buf[4]);
    return true;
}

bool QMI8658A_ReadAccel(qmi8658a_t *dev, qmi8658a_float_axes_t *accel_g)
{
    qmi8658a_raw_axes_t raw = {0};
    float scale = 0.0f;

    if (!_QMI8658A_ValidDev(dev) || (accel_g == 0))
        return false;

    if (!QMI8658A_ReadRawAccel(dev, &raw))
        return false;

    scale = _QMI8658A_AccelRangeG(dev->accel_range) / 32768.0f;
    accel_g->x = (float)raw.x * scale;
    accel_g->y = (float)raw.y * scale;
    accel_g->z = (float)raw.z * scale;
    return true;
}

bool QMI8658A_ReadGyro(qmi8658a_t *dev, qmi8658a_float_axes_t *gyro_dps)
{
    qmi8658a_raw_axes_t raw = {0};
    float scale = 0.0f;

    if (!_QMI8658A_ValidDev(dev) || (gyro_dps == 0))
        return false;

    if (!QMI8658A_ReadRawGyro(dev, &raw))
        return false;

    scale = _QMI8658A_GyroRangeDps(dev->gyro_range) / 32768.0f;
    gyro_dps->x = (float)raw.x * scale;
    gyro_dps->y = (float)raw.y * scale;
    gyro_dps->z = (float)raw.z * scale;
    return true;
}

bool QMI8658A_ReadSample(qmi8658a_t *dev, qmi8658a_sample_t *sample)
{
    if (!_QMI8658A_ValidDev(dev) || (sample == 0))
        return false;

    return QMI8658A_ReadTimestamp(dev, &sample->timestamp) &&
           QMI8658A_ReadTemperature(dev, &sample->temperature_c) &&
           QMI8658A_ReadAccel(dev, &sample->accel_g) &&
           QMI8658A_ReadGyro(dev, &sample->gyro_dps);
}

bool QMI8658A_Ctrl9(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd)
{
    uint8_t reg = (uint8_t)cmd;
    uint8_t ack = QMI8658A_CTRL_CMD_ACK;
    uint16_t timeout = (cmd == QMI8658A_CTRL_CMD_ON_DEMAND_CALIBRATION) ? QMI8658A_COD_TIMEOUT_MS : QMI8658A_CTRL9_TIMEOUT_MS;

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_9, &reg, 1);
    if (cmd == QMI8658A_CTRL_CMD_ACK)
        return true;

    if (!_QMI8658A_WaitCtrl9Done(dev, timeout))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_9, &ack, 1);
    return true;
}

bool QMI8658A_Ctrl9Write(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd, const uint8_t cal[QMI8658A_CAL_REG_NUM])
{
    return _QMI8658A_WriteCalAndRun(dev, cmd, cal);
}

bool QMI8658A_Ctrl9Read(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd, uint8_t cal[QMI8658A_CAL_REG_NUM])
{
    if (!_QMI8658A_ValidDev(dev) || (cal == 0))
        return false;

    if (!QMI8658A_Ctrl9(dev, cmd))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_CAL1_L, cal, QMI8658A_CAL_REG_NUM);
    return true;
}

bool QMI8658A_WriteAccelOffset(qmi8658a_t *dev, int16_t x, int16_t y, int16_t z)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    _QMI8658A_U16_TO_CAL(cal, 0, (uint16_t)x);
    _QMI8658A_U16_TO_CAL(cal, 2, (uint16_t)y);
    _QMI8658A_U16_TO_CAL(cal, 4, (uint16_t)z);
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_ACCEL_HOST_DELTA_OFFSET, cal);
}

bool QMI8658A_WriteGyroOffset(qmi8658a_t *dev, int16_t x, int16_t y, int16_t z)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    _QMI8658A_U16_TO_CAL(cal, 0, (uint16_t)x);
    _QMI8658A_U16_TO_CAL(cal, 2, (uint16_t)y);
    _QMI8658A_U16_TO_CAL(cal, 4, (uint16_t)z);
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_GYRO_HOST_DELTA_OFFSET, cal);
}

bool QMI8658A_SetPullups(qmi8658a_t *dev, uint8_t disable_mask)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    cal[0] = disable_mask & 0x0F;
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_SET_RPU, cal);
}

bool QMI8658A_SetAhbClockGating(qmi8658a_t *dev, bool enable)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    cal[0] = enable ? 0x01 : 0x00;
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_AHB_CLOCK_GATING, cal);
}

bool QMI8658A_ReadFwVersionUsid(qmi8658a_t *dev, qmi8658a_identity_t *identity)
{
    uint8_t fw[3] = {0};

    if (!_QMI8658A_ValidDev(dev) || (identity == 0))
        return false;

    if (!QMI8658A_Ctrl9(dev, QMI8658A_CTRL_CMD_COPY_USID))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_dQW_L, fw, 3);
    identity->fw_version[0] = fw[0];
    identity->fw_version[1] = fw[1];
    identity->fw_version[2] = fw[2];
    _QMI8658A_REG_READ(dev, QMI8658A_dVX_L, identity->usid, 6);
    return true;
}

bool QMI8658A_RunCOD(qmi8658a_t *dev, uint8_t *cod_status)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    if (!_QMI8658A_ValidDev(dev))
        return false;

    QMI8658A_EnableSensors(dev, QMI8658A_ACCEL_DISABLE | QMI8658A_GYRO_DISABLE);
    if (!QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_ON_DEMAND_CALIBRATION, cal))
        return false;

    if (cod_status != 0)
        _QMI8658A_REG_READ(dev, QMI8658A_COD_STATUS, cod_status, 1);

    return true;
}

bool QMI8658A_ApplyGyroGains(qmi8658a_t *dev, uint16_t gx, uint16_t gy, uint16_t gz)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    _QMI8658A_U16_TO_CAL(cal, 0, gx);
    _QMI8658A_U16_TO_CAL(cal, 2, gy);
    _QMI8658A_U16_TO_CAL(cal, 4, gz);
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_APPLY_GYRO_GAINS, cal);
}

bool QMI8658A_FIFOConfig(qmi8658a_t *dev, uint8_t watermark, qmi8658a_fifo_size_t size, qmi8658a_fifo_mode_t mode)
{
    uint8_t ctrl = QMI8658A_FIFO_CTRL_VALUE(size, mode);

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_WRITE(dev, QMI8658A_FIFO_WTM_TH, &watermark, 1);
    _QMI8658A_REG_WRITE(dev, QMI8658A_FIFO_CTRL, &ctrl, 1);
    return true;
}

bool QMI8658A_FIFOReset(qmi8658a_t *dev)
{
    return QMI8658A_Ctrl9(dev, QMI8658A_CTRL_CMD_RST_FIFO);
}

bool QMI8658A_FIFOGetStatus(qmi8658a_t *dev, qmi8658a_fifo_status_t *fifo_status)
{
    uint8_t buf[2] = {0};
    uint16_t words = 0;

    if (!_QMI8658A_ValidDev(dev) || (fifo_status == 0))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_FIFO_SMPL_CNT, buf, 2);
    words = (uint16_t)buf[0] | (uint16_t)((buf[1] & 0x03) << 8);
    fifo_status->full = QMI8658A_FIFO_STATUS_FULL(buf[1]);
    fifo_status->watermark = QMI8658A_FIFO_STATUS_WTM(buf[1]);
    fifo_status->overflow = QMI8658A_FIFO_STATUS_OVERFLOW(buf[1]);
    fifo_status->not_empty = QMI8658A_FIFO_STATUS_NOT_EMPTY(buf[1]);
    fifo_status->byte_count = (uint16_t)(words * 2U);
    return true;
}

bool QMI8658A_FIFORead(qmi8658a_t *dev, uint8_t *buffer, uint16_t max_len, uint16_t *read_len)
{
    qmi8658a_fifo_status_t status = {0};
    uint8_t ctrl = 0;
    uint16_t i = 0;
    uint16_t count = 0;

    if (!_QMI8658A_ValidDev(dev) || (buffer == 0) || (read_len == 0))
        return false;

    *read_len = 0;
    if (!QMI8658A_FIFOGetStatus(dev, &status))
        return false;

    count = (status.byte_count < max_len) ? status.byte_count : max_len;
    if (!QMI8658A_Ctrl9(dev, QMI8658A_CTRL_CMD_REQ_FIFO))
        return false;

    for (i = 0; i < count; i++)
        _QMI8658A_REG_READ(dev, QMI8658A_FIFO_DATA, &buffer[i], 1);

    _QMI8658A_REG_READ(dev, QMI8658A_FIFO_CTRL, &ctrl, 1);
    ctrl &= 0x7F;
    _QMI8658A_REG_WRITE(dev, QMI8658A_FIFO_CTRL, &ctrl, 1);
    *read_len = count;
    return true;
}

bool QMI8658A_ConfigureMotion(qmi8658a_t *dev, const qmi8658a_motion_config_t *cfg)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    if (!_QMI8658A_ValidDev(dev) || (cfg == 0))
        return false;

    cal[0] = cfg->any_x_thr;
    cal[1] = cfg->any_y_thr;
    cal[2] = cfg->any_z_thr;
    cal[3] = cfg->no_x_thr;
    cal[4] = cfg->no_y_thr;
    cal[5] = cfg->no_z_thr;
    cal[6] = cfg->mode_ctrl;
    cal[7] = 0x10;
    if (!QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_CONFIGURE_MOTION, cal))
        return false;

    cal[0] = cfg->any_window;
    cal[1] = cfg->no_window;
    _QMI8658A_U16_TO_CAL(cal, 2, cfg->sig_wait_window);
    _QMI8658A_U16_TO_CAL(cal, 4, cfg->sig_confirm_window);
    cal[6] = 0x00;
    cal[7] = 0x20;
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_CONFIGURE_MOTION, cal);
}

bool QMI8658A_EnableMotion(qmi8658a_t *dev, bool any_motion, bool no_motion, bool significant_motion)
{
    uint8_t ctrl8 = 0;

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_CTRL_8, &ctrl8, 1);
    ctrl8 &= (uint8_t)~(QMI8658A_ANY_MOTION_ENABLE | QMI8658A_NO_MOTION_ENABLE | QMI8658A_SIGNIFICANT_MOTION_ENABLE);
    if (any_motion)
        ctrl8 |= QMI8658A_ANY_MOTION_ENABLE;
    if (no_motion)
        ctrl8 |= QMI8658A_NO_MOTION_ENABLE;
    if (significant_motion)
        ctrl8 |= QMI8658A_SIGNIFICANT_MOTION_ENABLE;

    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_8, &ctrl8, 1);
    return true;
}

bool QMI8658A_ConfigureTap(qmi8658a_t *dev, const qmi8658a_tap_config_t *cfg)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    if (!_QMI8658A_ValidDev(dev) || (cfg == 0))
        return false;

    cal[0] = cfg->peak_window;
    cal[1] = cfg->priority & 0x07;
    _QMI8658A_U16_TO_CAL(cal, 2, cfg->tap_window);
    _QMI8658A_U16_TO_CAL(cal, 4, cfg->double_tap_window);
    cal[6] = 0x00;
    cal[7] = 0x10;
    if (!QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_CONFIGURE_TAP, cal))
        return false;

    cal[0] = cfg->alpha;
    cal[1] = cfg->gamma;
    _QMI8658A_U16_TO_CAL(cal, 2, cfg->peak_mag_thr);
    _QMI8658A_U16_TO_CAL(cal, 4, cfg->udm_thr);
    cal[6] = 0x00;
    cal[7] = 0x20;
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_CONFIGURE_TAP, cal);
}

bool QMI8658A_EnableTap(qmi8658a_t *dev, bool enable)
{
    uint8_t ctrl8 = 0;

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_CTRL_8, &ctrl8, 1);
    if (enable)
        ctrl8 |= QMI8658A_TAP_ENABLE;
    else
        ctrl8 &= (uint8_t)~QMI8658A_TAP_ENABLE;
    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_8, &ctrl8, 1);
    return true;
}

bool QMI8658A_ReadTapStatus(qmi8658a_t *dev, qmi8658a_tap_status_t *tap)
{
    uint8_t reg = 0;

    if (!_QMI8658A_ValidDev(dev) || (tap == 0))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_TAP_STATUS, &reg, 1);
    tap->polarity = QMI8658A_TAP_STATUS_POLARITY(reg);
    tap->axis = QMI8658A_TAP_STATUS_AXIS(reg);
    tap->count = QMI8658A_TAP_STATUS_COUNT(reg);
    return true;
}

bool QMI8658A_ConfigurePedometer(qmi8658a_t *dev, const qmi8658a_pedometer_config_t *cfg)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    if (!_QMI8658A_ValidDev(dev) || (cfg == 0))
        return false;

    _QMI8658A_U16_TO_CAL(cal, 0, cfg->sample_count);
    _QMI8658A_U16_TO_CAL(cal, 2, cfg->fix_peak2peak);
    _QMI8658A_U16_TO_CAL(cal, 4, cfg->fix_peak);
    cal[6] = 0x00;
    cal[7] = 0x10;
    if (!QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_CONFIGURE_PEDOMETER, cal))
        return false;

    _QMI8658A_U16_TO_CAL(cal, 0, cfg->time_up);
    cal[2] = cfg->time_low;
    cal[3] = cfg->cnt_entry;
    cal[4] = cfg->fix_precision;
    cal[5] = cfg->sig_count;
    cal[6] = 0x00;
    cal[7] = 0x20;
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_CONFIGURE_PEDOMETER, cal);
}

bool QMI8658A_EnablePedometer(qmi8658a_t *dev, bool enable)
{
    uint8_t ctrl8 = 0;

    if (!_QMI8658A_ValidDev(dev))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_CTRL_8, &ctrl8, 1);
    if (enable)
        ctrl8 |= QMI8658A_PEDOMETER_ENABLE;
    else
        ctrl8 &= (uint8_t)~QMI8658A_PEDOMETER_ENABLE;
    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_8, &ctrl8, 1);
    return true;
}

bool QMI8658A_ReadStepCount(qmi8658a_t *dev, uint32_t *steps)
{
    uint8_t buf[3] = {0};

    if (!_QMI8658A_ValidDev(dev) || (steps == 0))
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_STEP_CNT_LOW, buf, 3);
    *steps = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
    return true;
}

bool QMI8658A_ResetStepCount(qmi8658a_t *dev)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_RESET_PEDOMETER, cal);
}

bool QMI8658A_ConfigureWakeOnMotion(qmi8658a_t *dev, uint8_t threshold_mg,
                                    qmi8658a_wom_int_t int_cfg, uint8_t blanking_samples)
{
    uint8_t cal[QMI8658A_CAL_REG_NUM] = {0};

    if (!_QMI8658A_ValidDev(dev))
        return false;

    cal[0] = threshold_mg;
    cal[1] = (uint8_t)((((uint8_t)int_cfg & 0x03) << 6) | (blanking_samples & 0x3F));
    return QMI8658A_Ctrl9Write(dev, QMI8658A_CTRL_CMD_WRITE_WOM_SETTING, cal);
}

bool QMI8658A_DisableWakeOnMotion(qmi8658a_t *dev)
{
    return QMI8658A_ConfigureWakeOnMotion(dev, 0, QMI8658A_WOM_INT1_INITIAL_LOW, 0);
}

bool QMI8658A_RunAccelSelfTest(qmi8658a_t *dev, qmi8658a_raw_axes_t *delta_raw, bool *passed)
{
    uint8_t ctrl2 = 0;
    uint8_t status = 0;
    uint16_t elapsed = 0;
    uint8_t buf[6] = {0};

    if (!_QMI8658A_ValidDev(dev) || (delta_raw == 0) || (passed == 0))
        return false;

    QMI8658A_EnableSensors(dev, QMI8658A_ACCEL_DISABLE | QMI8658A_GYRO_DISABLE);
    _QMI8658A_REG_READ(dev, QMI8658A_CTRL_2, &ctrl2, 1);
    ctrl2 |= 0x80;
    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_2, &ctrl2, 1);

    do
    {
        _QMI8658A_REG_READ(dev, QMI8658A_STATUSINT, &status, 1);
        if (QMI8658A_STATUSINT_AVAILABLE(status))
            break;
        qmi8658a_Delay(1);
        elapsed++;
    } while (elapsed < QMI8658A_CTRL9_TIMEOUT_MS);

    ctrl2 &= 0x7F;
    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_2, &ctrl2, 1);
    if (elapsed >= QMI8658A_CTRL9_TIMEOUT_MS)
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_dVX_L, buf, 6);
    delta_raw->x = _QMI8658A_S16(&buf[0]);
    delta_raw->y = _QMI8658A_S16(&buf[2]);
    delta_raw->z = _QMI8658A_S16(&buf[4]);
    *passed = ((delta_raw->x > 409) || (delta_raw->x < -409)) &&
              ((delta_raw->y > 409) || (delta_raw->y < -409)) &&
              ((delta_raw->z > 409) || (delta_raw->z < -409));
    return true;
}

bool QMI8658A_RunGyroSelfTest(qmi8658a_t *dev, qmi8658a_raw_axes_t *delta_raw, bool *passed)
{
    uint8_t ctrl3 = 0;
    uint8_t status = 0;
    uint16_t elapsed = 0;
    uint8_t buf[6] = {0};

    if (!_QMI8658A_ValidDev(dev) || (delta_raw == 0) || (passed == 0))
        return false;

    QMI8658A_EnableSensors(dev, QMI8658A_ACCEL_DISABLE | QMI8658A_GYRO_DISABLE);
    _QMI8658A_REG_READ(dev, QMI8658A_CTRL_3, &ctrl3, 1);
    ctrl3 |= 0x80;
    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_3, &ctrl3, 1);

    do
    {
        _QMI8658A_REG_READ(dev, QMI8658A_STATUSINT, &status, 1);
        if (QMI8658A_STATUSINT_AVAILABLE(status))
            break;
        qmi8658a_Delay(1);
        elapsed++;
    } while (elapsed < QMI8658A_CTRL9_TIMEOUT_MS);

    ctrl3 &= 0x7F;
    _QMI8658A_REG_WRITE(dev, QMI8658A_CTRL_3, &ctrl3, 1);
    if (elapsed >= QMI8658A_CTRL9_TIMEOUT_MS)
        return false;

    _QMI8658A_REG_READ(dev, QMI8658A_dVX_L, buf, 6);
    delta_raw->x = _QMI8658A_S16(&buf[0]);
    delta_raw->y = _QMI8658A_S16(&buf[2]);
    delta_raw->z = _QMI8658A_S16(&buf[4]);
    *passed = ((delta_raw->x > 4800) || (delta_raw->x < -4800)) &&
              ((delta_raw->y > 4800) || (delta_raw->y < -4800)) &&
              ((delta_raw->z > 4800) || (delta_raw->z < -4800));
    return true;
}
