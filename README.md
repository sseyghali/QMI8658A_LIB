# QMI8658A Driver Library

This repository contains a C driver for the QST QMI8658A 6-axis inertial measurement unit.

The QMI8658A includes:

- 3-axis accelerometer with selectable full-scale ranges from +/-2 g to +/-16 g.
- 3-axis gyroscope with selectable full-scale ranges from +/-16 dps to +/-2048 dps.
- Temperature output.
- 24-bit sample timestamp.
- FIFO with bypass, FIFO, and stream modes.
- CTRL9 command engine for internal firmware functions.
- Motion detection, tap detection, pedometer, wake-on-motion, self-test, and calibration-on-demand support.

## File Layout

| File | Purpose |
| --- | --- |
| `QMI8658A.c` | Main driver implementation. Contains bus read/write helpers, initialization, configuration, data conversion, FIFO handling, CTRL9 command handling, motion engines, self-test, and COD routines. |
| `QMI8658A.h` | Public API. Defines the device object, output data structs, configuration structs, and all callable driver functions. |
| `QMI8658A_REG.h` | Register map and bit definitions. Contains register addresses, enum values, command IDs, status macros, and register-packing helper macros. |
| `QMI8658A_Reg.h` | Compatibility wrapper for projects that include the mixed-case register filename. It simply includes `QMI8658A_REG.h`. |
| `QMI8658A_PORTS.h` | Platform binding layer. Maps driver allocation, free, and delay calls to target functions. |

## External Dependencies

- `target_port.h`, providing:
  - `t_malloc(size_t)`
  - `t_free(void *)`
  - `t_sleep(x)`
- `io_driver/accel_io.h`, providing `accelIO_t`.

The expected bus interface is:

```c
typedef struct
{
    void (*init)(uint8_t address);
    void (*write_reg)(uint8_t address, uint8_t reg, uint8_t value);
    uint8_t (*read_reg)(uint8_t address, uint8_t reg);
} accelIO_t;
```

`QMI8658A_PORTS.h` maps:

| Driver macro | Platform function | Meaning |
| --- | --- | --- |
| `qmi8658a_malloc(x)` | `t_malloc(x)` | Allocates the `qmi8658a_t` device object. |
| `qmi8658a_free(x)` | `t_free(x)` | Frees the device object. |
| `qmi8658a_Delay(x)` | `t_sleep(x)` | Sleeps for the requested platform tick or millisecond interval, matching your `t_sleep` implementation. |

## Address Selection

The QMI8658A uses 7-bit I2C/I3C addresses:

| Macro | Address | Hardware state |
| --- | ---: | --- |
| `QMI8658A_ADDRESS_HIGH_LEVEL` | `0x6A` | SA0 high or floating. This is the recommended default because the chip has an internal weak pull-up during reset. |
| `QMI8658A_ADDRESS_LOW_LEVEL` | `0x6B` | SA0 pulled low. |

## Quick Start

```c
#include "QMI8658A.h"

extern accelIO_t imu_io;

void imu_example(void)
{
    qmi8658a_t *imu = QMI8658A_Init(&imu_io, QMI8658A_ADDRESS_HIGH_LEVEL);
    if (imu == 0)
        return;

    QMI8658A_SetAccelConfig(imu,
                            QMI8658A_ACCEL_RANGE_4G,
                            QMI8658A_ACCEL_ODR_250HZ,
                            false);

    QMI8658A_SetGyroConfig(imu,
                           QMI8658A_GYRO_RANGE_512DPS,
                           QMI8658A_GYRO_ODR_224_2HZ,
                           false);

    QMI8658A_SetFilter(imu,
                       QMI8658A_LPF_MODE_2_66_PERCENT, true,
                       QMI8658A_LPF_MODE_2_66_PERCENT, true);

    QMI8658A_EnableSensors(imu, (qmi8658a_ctrl7_t)(QMI8658A_ACCEL_ENABLE |
                                                   QMI8658A_GYRO_ENABLE));

    qmi8658a_sample_t sample;
    if (QMI8658A_ReadSample(imu, &sample))
    {
        /* sample.accel_g.x/y/z are in g.
         * sample.gyro_dps.x/y/z are in degrees per second.
         * sample.temperature_c is in degrees Celsius.
         */
    }
}
```

## Driver Object and Data Types

### `qmi8658a_t`

```c
typedef struct
{
    uint8_t    address;
    accelIO_t *interface;
    qmi8658a_accel_range_t accel_range;
    qmi8658a_gyro_range_t  gyro_range;
    qmi8658a_ctrl7_t       enabled_sensors;
} qmi8658a_t;
```

This is the device handle returned by `QMI8658A_Init`.

- `address` is the 7-bit bus address.
- `interface` points to the bus implementation.
- `accel_range` and `gyro_range` are cached so scaled reads can convert raw counts to physical units.
- `enabled_sensors` stores the last `CTRL7` sensor-enable value written by the driver.

### Axis and Sample Types

| Type | Description |
| --- | --- |
| `qmi8658a_raw_axes_t` | Signed 16-bit X/Y/Z raw register values. |
| `qmi8658a_float_axes_t` | Floating-point X/Y/Z values after scaling. Accelerometer values are in g; gyroscope values are in dps. |
| `qmi8658a_sample_t` | Combined accelerometer, gyroscope, temperature, and timestamp snapshot. |
| `qmi8658a_status_t` | Raw `STATUSINT`, `STATUS0`, and `STATUS1` bytes. |
| `qmi8658a_fifo_status_t` | Decoded FIFO flags and byte count. |
| `qmi8658a_tap_status_t` | Decoded tap polarity, axis, and tap count. |
| `qmi8658a_identity_t` | Firmware version and 6-byte unique sensor ID. |

## Register Map

The register map is declared in `qmi8658a_reg_t`.

| Register macro | Address | Access | Description |
| --- | ---: | --- | --- |
| `QMI8658A_WHO_AM_I` | `0x00` | R | Device identifier. Expected value is `QMI8658A_WHO_AM_I_VALUE` (`0x05`). |
| `QMI8658A_REVISION_ID` | `0x01` | R | Device revision ID. |
| `QMI8658A_CTRL_1` | `0x02` | RW | Serial interface, auto-increment, endian selection, INT1/INT2 output enables, FIFO interrupt route, oscillator disable. |
| `QMI8658A_CTRL_2` | `0x03` | RW | Accelerometer self-test, full-scale range, and ODR. |
| `QMI8658A_CTRL_3` | `0x04` | RW | Gyroscope self-test, full-scale range, and ODR. |
| `QMI8658A_CTRL_5` | `0x06` | RW | Accelerometer and gyroscope low-pass filter enable/mode. |
| `QMI8658A_CTRL_7` | `0x08` | RW | SyncSample, DRDY behavior, gyro snooze, accelerometer enable, gyroscope enable. |
| `QMI8658A_CTRL_8` | `0x09` | RW | CTRL9 handshake mode, activity interrupt route, pedometer, significant-motion, no-motion, any-motion, and tap enables. |
| `QMI8658A_CTRL_9` | `0x0A` | RW | Host command register for CTRL9 protocol. |
| `QMI8658A_CAL1_L` through `QMI8658A_CAL4_H` | `0x0B`-`0x12` | RW | Eight-byte parameter mailbox used by CTRL9 write/read commands. |
| `QMI8658A_FIFO_WTM_TH` | `0x13` | RW | FIFO watermark threshold in samples. |
| `QMI8658A_FIFO_CTRL` | `0x14` | RW | FIFO read mode, FIFO size, and FIFO operating mode. |
| `QMI8658A_FIFO_SMPL_CNT` | `0x15` | R | FIFO sample count low byte, in 2-byte words. |
| `QMI8658A_FIFO_STATUS` | `0x16` | R | FIFO full, watermark, overflow, not-empty, and sample count high bits. |
| `QMI8658A_FIFO_DATA` | `0x17` | R | FIFO byte output. Each read pops one FIFO byte in read mode. |
| `QMI8658A_STATUSINT` | `0x2D` | R | CTRL9 command done flag and SyncSample data-lock/data-available bits. |
| `QMI8658A_STATUS0` | `0x2E` | R | Accelerometer and gyroscope new-data-available bits. |
| `QMI8658A_STATUS1` | `0x2F` | R | Significant-motion, no-motion, any-motion, pedometer, WoM, and tap status bits. |
| `QMI8658A_TIMESTAMP_LOW/MID/HIGH` | `0x30`-`0x32` | R | 24-bit circular sample timestamp. |
| `QMI8658A_TEMP_L/H` | `0x33`-`0x34` | R | Temperature output. Driver converts as `TEMP_H + TEMP_L / 256`. |
| `QMI8658A_AX_L/H` | `0x35`-`0x36` | R | X-axis accelerometer output. |
| `QMI8658A_AY_L/H` | `0x37`-`0x38` | R | Y-axis accelerometer output. |
| `QMI8658A_AZ_L/H` | `0x39`-`0x3A` | R | Z-axis accelerometer output. |
| `QMI8658A_GX_L/H` | `0x3B`-`0x3C` | R | X-axis gyroscope output. |
| `QMI8658A_GY_L/H` | `0x3D`-`0x3E` | R | Y-axis gyroscope output. |
| `QMI8658A_GZ_L/H` | `0x3F`-`0x40` | R | Z-axis gyroscope output. |
| `QMI8658A_COD_STATUS` | `0x46` | R | Calibration-on-demand result bits. |
| `QMI8658A_dQW_L` through `QMI8658A_dVZ_H` | `0x49`-`0x56` | R | General-purpose/FW output registers used by CTRL9 read commands, self-test result output, and reset status. |
| `QMI8658A_TAP_STATUS` | `0x59` | R | Tap polarity, tap axis, and single/double tap result. |
| `QMI8658A_STEP_CNT_LOW/MIDL/HIGH` | `0x5A`-`0x5C` | R | 24-bit pedometer step count. |
| `QMI8658A_RESET` | `0x60` | W | Soft reset register. The driver writes `QMI8658A_RESET_CMD` (`0xB0`). |

## Important Register Bit Groups

### `CTRL1` Interface and Interrupt Control

Use `QMI8658A_SetInterface`.

| Bit field | Driver values | Meaning |
| --- | --- | --- |
| `SIM`, bit 7 | `QMI8658A_SPI_4_WIRE_ENABLE`, `QMI8658A_SPI_3_WIRE_ENABLE` | Selects 4-wire or 3-wire SPI mode. |
| `ADDR_AI`, bit 6 | `QMI8658A_ADDR_NON_INCREMENT`, `QMI8658A_ADDR_AUTO_INCREMENT` | Enables register address auto-increment for burst-style sequential reads. |
| `BE`, bit 5 | `QMI8658A_LITTLE_ENDIAN`, `QMI8658A_BIG_ENDIAN` | Selects serial read data endian mode. The driver reads low byte then high byte and initializes little-endian mode. |
| `INT2_EN`, bit 4 | `QMI8658A_INT2_DISABLE`, `QMI8658A_INT2_ENABLE` | Enables INT2 push-pull output. Disabled means high-Z. |
| `INT1_EN`, bit 3 | `QMI8658A_INT1_DISABLE`, `QMI8658A_INT1_ENABLE` | Enables INT1 push-pull output. Disabled means high-Z. |
| `FIFO_INT_SEL`, bit 2 | `QMI8658A_FIFO_INT_TO_INT2`, `QMI8658A_FIFO_INT_TO_INT1` | Selects FIFO interrupt destination. |
| `SensorDisable`, bit 0 | `QMI8658A_OSC_ENABLE`, `QMI8658A_OSC_DISABLE` | Controls internal high-speed oscillator. |

### `CTRL2` Accelerometer Configuration

Use `QMI8658A_SetAccelConfig`.

| Field | Values |
| --- | --- |
| Self-test bit | `self_test = true` sets bit 7. Normal operation uses `false`. |
| Range | `QMI8658A_ACCEL_RANGE_2G`, `QMI8658A_ACCEL_RANGE_4G`, `QMI8658A_ACCEL_RANGE_8G`, `QMI8658A_ACCEL_RANGE_16G`. |
| Normal ODR | `QMI8658A_ACCEL_ODR_1000HZ`, `500HZ`, `250HZ`, `125HZ`, `62_5HZ`, `31_25HZ`. |
| Low-power ODR | `QMI8658A_ACCEL_ODR_LOW_POWER_128HZ`, `21HZ`, `11HZ`, `3HZ`. Low-power accelerometer modes are intended for accelerometer-only use. |

The driver packs this register with `QMI8658A_CTRL2_VALUE(range, odr, st)`.

### `CTRL3` Gyroscope Configuration

Use `QMI8658A_SetGyroConfig`.

| Field | Values |
| --- | --- |
| Self-test bit | `self_test = true` sets bit 7. Normal operation uses `false`. |
| Range | `QMI8658A_GYRO_RANGE_16DPS`, `32DPS`, `64DPS`, `128DPS`, `256DPS`, `512DPS`, `1024DPS`, `2048DPS`. |
| ODR | `QMI8658A_GYRO_ODR_7174_4HZ`, `3587_2HZ`, `1793_6HZ`, `896_8HZ`, `448_4HZ`, `224_2HZ`, `112_1HZ`, `56_05HZ`, `28_025HZ`. |

The driver packs this register with `QMI8658A_CTRL3_VALUE(range, odr, st)`.

### `CTRL5` Low-Pass Filters

Use `QMI8658A_SetFilter`.

| Filter mode | Bandwidth as percentage of ODR |
| --- | ---: |
| `QMI8658A_LPF_MODE_2_66_PERCENT` | 2.66% |
| `QMI8658A_LPF_MODE_3_63_PERCENT` | 3.63% |
| `QMI8658A_LPF_MODE_5_39_PERCENT` | 5.39% |
| `QMI8658A_LPF_MODE_13_37_PERCENT` | 13.37% |

The function takes separate accelerometer and gyroscope mode/enable pairs.

### `CTRL7` Sensor Enable and Data Mode

Use `QMI8658A_EnableSensors`.

| Value | Meaning |
| --- | --- |
| `QMI8658A_SYNCSAMPLE_ENABLE` | Enables SyncSample locking mode. |
| `QMI8658A_DRDY_DISABLE` | Blocks DRDY from INT2. |
| `QMI8658A_GYRO_SNOOZE_MODE` | Enables gyroscope snooze while `gEN` is set. |
| `QMI8658A_GYRO_ENABLE` | Enables gyroscope. |
| `QMI8658A_ACCEL_ENABLE` | Enables accelerometer. |

Common examples:

```c
QMI8658A_EnableSensors(dev, QMI8658A_ACCEL_ENABLE);
QMI8658A_EnableSensors(dev, (qmi8658a_ctrl7_t)(QMI8658A_ACCEL_ENABLE | QMI8658A_GYRO_ENABLE));
QMI8658A_EnableSensors(dev, (qmi8658a_ctrl7_t)(QMI8658A_SYNCSAMPLE_ENABLE |
                                               QMI8658A_ACCEL_ENABLE |
                                               QMI8658A_GYRO_ENABLE));
```

### `CTRL8` Activity Engine Control

Use `QMI8658A_SetActivityControl`, or use the higher-level enable functions.

| Value | Meaning |
| --- | --- |
| `QMI8658A_CTRL9_HANDSHAKE_STATUSINT` | Poll `STATUSINT.bit7` for CTRL9 completion instead of using INT1. |
| `QMI8658A_ACTIVITY_INT_TO_INT1` | Routes activity interrupts to INT1. The default route is INT2. |
| `QMI8658A_PEDOMETER_ENABLE` | Enables pedometer engine. |
| `QMI8658A_SIGNIFICANT_MOTION_ENABLE` | Enables significant-motion engine. |
| `QMI8658A_NO_MOTION_ENABLE` | Enables no-motion engine. |
| `QMI8658A_ANY_MOTION_ENABLE` | Enables any-motion engine. |
| `QMI8658A_TAP_ENABLE` | Enables tap engine. |

## Public API Reference

All functions return `true` for successful driver-level execution and `false` for invalid arguments, missing device/interface pointers, or timeout conditions. Low-level bus functions in `accelIO_t` do not return errors, so hardware bus failure detection must be handled in the platform layer if required.

### Initialization and Lifetime

#### `QMI8658A_Init(accelIO_t *io, uint8_t adr)`

Allocates and initializes a `qmi8658a_t`.

Steps performed:

- Checks that `io` is not null.
- Allocates the device handle using `qmi8658a_malloc`.
- Stores the bus address and interface pointer.
- Calls `io->init(adr)`.
- Reads `WHO_AM_I` and prints a debug message if the value is not `0x05`.
- Configures default interface, accelerometer, gyroscope, and filter settings.

Default configuration:

- Accelerometer range: +/-2 g.
- Accelerometer ODR: 250 Hz.
- Gyroscope range: +/-256 dps.
- Gyroscope ODR: 224.2 Hz.
- Accelerometer and gyroscope LPFs disabled.
- Sensors are configured but not enabled by default.

#### `QMI8658A_Deinit(qmi8658a_t *dev)`

Frees a handle allocated by `QMI8658A_Init`.

#### `QMI8658A_ReadChipID(qmi8658a_t *dev, uint8_t *who_am_i, uint8_t *revision_id)`

Reads chip identity registers. Either output pointer may be null if that value is not needed.

#### `QMI8658A_Reset(qmi8658a_t *dev)`

Writes `0xB0` to the reset register and waits 15 ms. The driver then marks cached sensor enables as disabled.

Note: the Rev A datasheet contains one prose line saying `0x0B`, while the reset table specifies `0xB0`. This driver follows the reset table and common QST driver behavior.

### Sensor Configuration

#### `QMI8658A_SetInterface(qmi8658a_t *dev, qmi8658a_ctrl1_t cfg)`

Writes `CTRL1`. Combine `qmi8658a_ctrl1_t` values with bitwise OR.

Example:

```c
QMI8658A_SetInterface(dev, (qmi8658a_ctrl1_t)(QMI8658A_ADDR_AUTO_INCREMENT |
                                              QMI8658A_LITTLE_ENDIAN |
                                              QMI8658A_INT1_ENABLE |
                                              QMI8658A_INT2_ENABLE));
```

#### `QMI8658A_SetAccelConfig(qmi8658a_t *dev, qmi8658a_accel_range_t range, qmi8658a_accel_odr_t odr, bool self_test)`

Writes `CTRL2` and updates the cached accelerometer range. The cached range is used by `QMI8658A_ReadAccel` and `QMI8658A_ReadSample`.

#### `QMI8658A_SetGyroConfig(qmi8658a_t *dev, qmi8658a_gyro_range_t range, qmi8658a_gyro_odr_t odr, bool self_test)`

Writes `CTRL3` and updates the cached gyroscope range. The cached range is used by `QMI8658A_ReadGyro` and `QMI8658A_ReadSample`.

#### `QMI8658A_SetFilter(qmi8658a_t *dev, qmi8658a_lpf_mode_t accel_lpf, bool accel_enable, qmi8658a_lpf_mode_t gyro_lpf, bool gyro_enable)`

Writes `CTRL5`. Each sensor has its own LPF mode and enable bit.

#### `QMI8658A_EnableSensors(qmi8658a_t *dev, qmi8658a_ctrl7_t sensors)`

Writes `CTRL7` and updates the cached `enabled_sensors` field.

#### `QMI8658A_SetActivityControl(qmi8658a_t *dev, qmi8658a_ctrl8_t cfg)`

Writes `CTRL8` directly. Higher-level helpers such as `QMI8658A_EnableTap`, `QMI8658A_EnablePedometer`, and `QMI8658A_EnableMotion` preserve unrelated `CTRL8` bits while toggling their specific feature bits.

### Data and Status Reads

#### `QMI8658A_ReadStatus(qmi8658a_t *dev, qmi8658a_status_t *status)`

Reads `STATUSINT`, `STATUS0`, and `STATUS1` into one struct.

Useful helper macros:

- `QMI8658A_STATUSINT_CMD_DONE(status.statusint)`
- `QMI8658A_STATUSINT_LOCKED(status.statusint)`
- `QMI8658A_STATUSINT_AVAILABLE(status.statusint)`
- `QMI8658A_STATUS0_GYRO_AVAILABLE(status.status0)`
- `QMI8658A_STATUS0_ACCEL_AVAILABLE(status.status0)`
- `QMI8658A_STATUS1_SIG_MOTION(status.status1)`
- `QMI8658A_STATUS1_NO_MOTION(status.status1)`
- `QMI8658A_STATUS1_ANY_MOTION(status.status1)`
- `QMI8658A_STATUS1_PEDOMETER(status.status1)`
- `QMI8658A_STATUS1_WOM(status.status1)`
- `QMI8658A_STATUS1_TAP(status.status1)`

#### `QMI8658A_ReadTimestamp(qmi8658a_t *dev, uint32_t *timestamp)`

Reads the 24-bit timestamp from `TIMESTAMP_LOW`, `TIMESTAMP_MID`, and `TIMESTAMP_HIGH`. The value wraps at `0xFFFFFF`.

#### `QMI8658A_ReadTemperature(qmi8658a_t *dev, float *temperature_c)`

Reads `TEMP_L/H` and returns degrees Celsius using:

```c
temperature_c = (int8_t)TEMP_H + TEMP_L / 256.0f;
```

#### `QMI8658A_ReadRawAccel(qmi8658a_t *dev, qmi8658a_raw_axes_t *raw)`

Reads six accelerometer data bytes and returns signed 16-bit X/Y/Z counts.

#### `QMI8658A_ReadRawGyro(qmi8658a_t *dev, qmi8658a_raw_axes_t *raw)`

Reads six gyroscope data bytes and returns signed 16-bit X/Y/Z counts.

#### `QMI8658A_ReadAccel(qmi8658a_t *dev, qmi8658a_float_axes_t *accel_g)`

Reads raw acceleration and converts counts to g:

```c
g = raw * selected_full_scale_g / 32768.0f;
```

#### `QMI8658A_ReadGyro(qmi8658a_t *dev, qmi8658a_float_axes_t *gyro_dps)`

Reads raw angular rate and converts counts to degrees per second:

```c
dps = raw * selected_full_scale_dps / 32768.0f;
```

#### `QMI8658A_ReadSample(qmi8658a_t *dev, qmi8658a_sample_t *sample)`

Reads timestamp, temperature, scaled acceleration, and scaled gyroscope data. Returns `false` if any sub-read fails.

## CTRL9 Command System

The QMI8658A uses `CTRL9` as a command register for firmware-managed features. Some commands are simple command-only operations. Other commands use the eight CAL registers as a parameter mailbox.

The driver implements the command handshake:

1. Write command byte to `CTRL9`.
2. Wait for `STATUSINT.bit7` (`CmdDone`) to become 1.
3. Write `CTRL_CMD_ACK` (`0x00`) to `CTRL9`.

`QMI8658A_Ctrl9` uses a 200 ms timeout for ordinary commands and a 1700 ms timeout for calibration-on-demand.

### CTRL9 Commands

| Command macro | Value | Driver function(s) | Description |
| --- | ---: | --- | --- |
| `QMI8658A_CTRL_CMD_ACK` | `0x00` | Internal/`QMI8658A_Ctrl9` | Acknowledges command completion. |
| `QMI8658A_CTRL_CMD_RST_FIFO` | `0x04` | `QMI8658A_FIFOReset` | Clears FIFO data, count, and flags. |
| `QMI8658A_CTRL_CMD_REQ_FIFO` | `0x05` | `QMI8658A_FIFORead` | Puts FIFO into read mode. |
| `QMI8658A_CTRL_CMD_WRITE_WOM_SETTING` | `0x08` | `QMI8658A_ConfigureWakeOnMotion`, `QMI8658A_DisableWakeOnMotion` | Configures WoM threshold and interrupt behavior. |
| `QMI8658A_CTRL_CMD_ACCEL_HOST_DELTA_OFFSET` | `0x09` | `QMI8658A_WriteAccelOffset` | Applies volatile accelerometer offset deltas. |
| `QMI8658A_CTRL_CMD_GYRO_HOST_DELTA_OFFSET` | `0x0A` | `QMI8658A_WriteGyroOffset` | Applies volatile gyroscope offset deltas. |
| `QMI8658A_CTRL_CMD_CONFIGURE_TAP` | `0x0C` | `QMI8658A_ConfigureTap` | Loads tap engine parameters. |
| `QMI8658A_CTRL_CMD_CONFIGURE_PEDOMETER` | `0x0D` | `QMI8658A_ConfigurePedometer` | Loads pedometer engine parameters. |
| `QMI8658A_CTRL_CMD_CONFIGURE_MOTION` | `0x0E` | `QMI8658A_ConfigureMotion` | Loads any/no/significant motion parameters. |
| `QMI8658A_CTRL_CMD_RESET_PEDOMETER` | `0x0F` | `QMI8658A_ResetStepCount` | Clears step count. |
| `QMI8658A_CTRL_CMD_COPY_USID` | `0x10` | `QMI8658A_ReadFwVersionUsid` | Copies firmware version and USID to output registers. |
| `QMI8658A_CTRL_CMD_SET_RPU` | `0x11` | `QMI8658A_SetPullups` | Enables/disables internal pull-up resistor groups. |
| `QMI8658A_CTRL_CMD_AHB_CLOCK_GATING` | `0x12` | `QMI8658A_SetAhbClockGating` | Controls internal AHB clock gating. |
| `QMI8658A_CTRL_CMD_ON_DEMAND_CALIBRATION` | `0xA2` | `QMI8658A_RunCOD` | Runs gyroscope calibration-on-demand. |
| `QMI8658A_CTRL_CMD_APPLY_GYRO_GAINS` | `0xAA` | `QMI8658A_ApplyGyroGains` | Restores saved gyroscope gain values. |

### Generic CTRL9 Helpers

#### `QMI8658A_Ctrl9(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd)`

Runs a command that needs no caller-supplied CAL data.

#### `QMI8658A_Ctrl9Write(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd, const uint8_t cal[8])`

Writes all eight CAL registers, then runs the requested command.

#### `QMI8658A_Ctrl9Read(qmi8658a_t *dev, qmi8658a_ctrl9_cmd_t cmd, uint8_t cal[8])`

Runs the requested command, then reads the eight CAL registers.

## Offset, Pull-Up, Identity, COD, and Gain Functions

#### `QMI8658A_WriteAccelOffset(qmi8658a_t *dev, int16_t x, int16_t y, int16_t z)`

Writes volatile accelerometer delta offsets through CTRL9. The QMI8658A expects signed 4.12 values in the CAL registers.

#### `QMI8658A_WriteGyroOffset(qmi8658a_t *dev, int16_t x, int16_t y, int16_t z)`

Writes volatile gyroscope delta offsets through CTRL9. The QMI8658A expects signed 11.5 values in the CAL registers.

#### `QMI8658A_SetPullups(qmi8658a_t *dev, uint8_t disable_mask)`

Configures internal pull-up resistor groups with `CTRL_CMD_SET_RPU`. Only bits 0-3 are used.

| Bit | Meaning when set to 1 |
| --- | --- |
| 0 | Disable AUX pull-up group. |
| 1 | Disable ICM/SDx pull-up. |
| 2 | Disable CS pull-up. |
| 3 | Disable I2C SCL/SDA pull-ups. |

#### `QMI8658A_SetAhbClockGating(qmi8658a_t *dev, bool enable)`

Writes `CAL1_L = 1` to enable AHB clock gating or `CAL1_L = 0` to disable it, then runs `CTRL_CMD_AHB_CLOCK_GATING`. Disable this when using SyncSample locking if your application needs strict sample alignment.

#### `QMI8658A_ReadFwVersionUsid(qmi8658a_t *dev, qmi8658a_identity_t *identity)`

Runs `CTRL_CMD_COPY_USID`, then reads:

- Firmware version from `dQW_L`, `dQW_H`, and `dQX_L`.
- USID bytes from `dVX_L` through `dVZ_H`.

#### `QMI8658A_RunCOD(qmi8658a_t *dev, uint8_t *cod_status)`

Disables accelerometer and gyroscope, runs calibration-on-demand, and optionally reads `COD_STATUS`.

`COD_STATUS` bit meanings:

| Bit | Meaning when set to 1 |
| --- | --- |
| 7 | X-axis gyroscope low sensitivity limit failed. |
| 6 | X-axis gyroscope high sensitivity limit failed. |
| 5 | Y-axis gyroscope low sensitivity limit failed. |
| 4 | Y-axis gyroscope high sensitivity limit failed. |
| 3 | Accelerometer check failed because significant vibration happened during COD. |
| 2 | Gyroscope startup failed during COD. |
| 1 | COD was called while gyroscope was enabled. |
| 0 | COD failed; new correction was not applied. |

`0x00` means COD succeeded.

#### `QMI8658A_ApplyGyroGains(qmi8658a_t *dev, uint16_t gx, uint16_t gy, uint16_t gz)`

Writes saved gyroscope gain values to CAL1/CAL2/CAL3 and runs `CTRL_CMD_APPLY_GYRO_GAINS`.

## FIFO

The QMI8658A FIFO stores sensor output bytes. When one sensor is enabled, each sample contributes 6 bytes. When accelerometer and gyroscope are both enabled, each combined sample contributes 12 bytes.

### FIFO Modes and Sizes

| Size enum | Capacity |
| --- | ---: |
| `QMI8658A_FIFO_SIZE_16_SAMPLES` | 16 samples |
| `QMI8658A_FIFO_SIZE_32_SAMPLES` | 32 samples |
| `QMI8658A_FIFO_SIZE_64_SAMPLES` | 64 samples |
| `QMI8658A_FIFO_SIZE_128_SAMPLES` | 128 samples |

| Mode enum | Behavior |
| --- | --- |
| `QMI8658A_FIFO_MODE_BYPASS` | FIFO disabled. DRDY mode is used instead. |
| `QMI8658A_FIFO_MODE_FIFO` | Filling stops when FIFO is full; new data is discarded until data is read. |
| `QMI8658A_FIFO_MODE_STREAM` | Filling continues when full; oldest data is discarded. |

#### `QMI8658A_FIFOConfig(qmi8658a_t *dev, uint8_t watermark, qmi8658a_fifo_size_t size, qmi8658a_fifo_mode_t mode)`

Writes `FIFO_WTM_TH` and `FIFO_CTRL`.

#### `QMI8658A_FIFOReset(qmi8658a_t *dev)`

Runs `CTRL_CMD_RST_FIFO`.

#### `QMI8658A_FIFOGetStatus(qmi8658a_t *dev, qmi8658a_fifo_status_t *fifo_status)`

Reads `FIFO_SMPL_CNT` and `FIFO_STATUS`. The driver reports `byte_count` as:

```c
byte_count = 2 * ((FIFO_STATUS[1:0] << 8) | FIFO_SMPL_CNT);
```

Decoded flags:

- `full`
- `watermark`
- `overflow`
- `not_empty`

#### `QMI8658A_FIFORead(qmi8658a_t *dev, uint8_t *buffer, uint16_t max_len, uint16_t *read_len)`

Implements the datasheet FIFO read sequence:

1. Read FIFO status and byte count.
2. Run `CTRL_CMD_REQ_FIFO` to enter FIFO read mode.
3. Read up to `max_len` bytes from `FIFO_DATA`.
4. Clear `FIFO_CTRL.bit7` to leave FIFO read mode.
5. Return the actual byte count in `read_len`.

## Motion Detection

Motion detection operates on accelerometer slope data and must be configured through CTRL9 before enabling the corresponding engines in `CTRL8`.

### `qmi8658a_motion_config_t`

| Field | CAL/register meaning |
| --- | --- |
| `any_x_thr`, `any_y_thr`, `any_z_thr` | Any-motion X/Y/Z slope thresholds. Resolution is 1/32 g. |
| `no_x_thr`, `no_y_thr`, `no_z_thr` | No-motion X/Y/Z slope thresholds. Resolution is 1/32 g. |
| `mode_ctrl` | Packed axis enable and OR/AND logic byte. |
| `any_window` | Minimum consecutive samples above threshold for any-motion. |
| `no_window` | Minimum consecutive samples below threshold for no-motion. |
| `sig_wait_window` | Idle/wait samples after first any-motion event before significant-motion confirmation. |
| `sig_confirm_window` | Maximum samples to detect a second any-motion event for significant-motion. |

`mode_ctrl` layout:

| Bit | Meaning |
| --- | --- |
| 7 | No-motion axis logic: 0 = OR, 1 = AND. |
| 6 | Enable Z for no-motion. |
| 5 | Enable Y for no-motion. |
| 4 | Enable X for no-motion. |
| 3 | Any-motion axis logic: 0 = OR, 1 = AND. |
| 2 | Enable Z for any-motion. |
| 1 | Enable Y for any-motion. |
| 0 | Enable X for any-motion. |

#### `QMI8658A_ConfigureMotion(qmi8658a_t *dev, const qmi8658a_motion_config_t *cfg)`

Loads the motion engine in two CTRL9 command sets, matching the datasheet CAL register layout.

#### `QMI8658A_EnableMotion(qmi8658a_t *dev, bool any_motion, bool no_motion, bool significant_motion)`

Sets or clears the `CTRL8` bits for any-motion, no-motion, and significant-motion while preserving unrelated `CTRL8` settings.

## Tap Detection

Tap detection reports single tap or double tap, tap axis, and tap polarity through `TAP_STATUS`.

### `qmi8658a_tap_config_t`

| Field | Meaning |
| --- | --- |
| `peak_window` | Maximum valid peak duration in samples. |
| `priority` | Axis priority, lower 3 bits only. |
| `tap_window` | Minimum quiet time before the second tap. |
| `double_tap_window` | Maximum time for the second tap. |
| `alpha` | Acceleration averaging ratio, 1/128 resolution. |
| `gamma` | Movement magnitude averaging ratio, 1/128 resolution. |
| `peak_mag_thr` | Peak detection threshold, 1/1024 g^2 resolution. |
| `udm_thr` | Undefined motion threshold, 1/1024 g resolution. |

Tap priority values:

| Value | Priority order |
| ---: | --- |
| 0 | X > Y > Z |
| 1 | X > Z > Y |
| 2 | Y > X > Z |
| 3 | Y > Z > X |
| 4 | Z > X > Y |
| 5 | Z > Y > X |
| 6, 7 | Same as 0 |

#### `QMI8658A_ConfigureTap(qmi8658a_t *dev, const qmi8658a_tap_config_t *cfg)`

Loads tap parameters in two CTRL9 command sets.

#### `QMI8658A_EnableTap(qmi8658a_t *dev, bool enable)`

Sets or clears `CTRL8.bit0`.

#### `QMI8658A_ReadTapStatus(qmi8658a_t *dev, qmi8658a_tap_status_t *tap)`

Reads and decodes `TAP_STATUS`.

| Output field | Meaning |
| --- | --- |
| `polarity` | 0 = positive direction, 1 = negative direction. |
| `axis` | 0 = none, 1 = X, 2 = Y, 3 = Z. |
| `count` | 0 = none, 1 = single tap, 2 = double tap. |

## Pedometer

The pedometer engine counts steps and reports a 24-bit value through the step-count registers.

### `qmi8658a_pedometer_config_t`

| Field | Meaning |
| --- | --- |
| `sample_count` | Sample batch/window count. |
| `fix_peak2peak` | Valid peak-to-peak threshold. Resolution is approximately 1 mg. |
| `fix_peak` | Peak threshold versus average. Resolution is approximately 1 mg. |
| `time_up` | Maximum step duration in samples. |
| `time_low` | Minimum step duration in samples. |
| `cnt_entry` | Minimum continuous steps before valid counting starts. |
| `fix_precision` | Precision parameter. The datasheet recommends 0. |
| `sig_count` | Number of valid steps between pedometer register updates/interrupts. |

#### `QMI8658A_ConfigurePedometer(qmi8658a_t *dev, const qmi8658a_pedometer_config_t *cfg)`

Loads pedometer parameters in two CTRL9 command sets.

#### `QMI8658A_EnablePedometer(qmi8658a_t *dev, bool enable)`

Sets or clears `CTRL8.bit4`. Enabling the pedometer from disabled state resets the step count according to the datasheet.

#### `QMI8658A_ReadStepCount(qmi8658a_t *dev, uint32_t *steps)`

Reads the 24-bit step count from `STEP_CNT_LOW`, `STEP_CNT_MIDL`, and `STEP_CNT_HIGH`.

#### `QMI8658A_ResetStepCount(qmi8658a_t *dev)`

Runs `CTRL_CMD_RESET_PEDOMETER` to clear the step count without resetting the whole device.

## Wake On Motion

Wake-on-motion is a low-power accelerometer mode that toggles a selected interrupt pin when motion is detected.

### WoM Interrupt Selection

| Enum | Selected pin | Initial pin state |
| --- | --- | --- |
| `QMI8658A_WOM_INT1_INITIAL_LOW` | INT1 | Low |
| `QMI8658A_WOM_INT2_INITIAL_LOW` | INT2 | Low |
| `QMI8658A_WOM_INT1_INITIAL_HIGH` | INT1 | High |
| `QMI8658A_WOM_INT2_INITIAL_HIGH` | INT2 | High |

#### `QMI8658A_ConfigureWakeOnMotion(qmi8658a_t *dev, uint8_t threshold_mg, qmi8658a_wom_int_t int_cfg, uint8_t blanking_samples)`

Writes:

- `CAL1_L = threshold_mg`
- `CAL1_H[7:6] = int_cfg`
- `CAL1_H[5:0] = blanking_samples`

Then runs `CTRL_CMD_WRITE_WOM_SETTING`.

Use threshold `0` to disable WoM.

#### `QMI8658A_DisableWakeOnMotion(qmi8658a_t *dev)`

Convenience wrapper that writes a zero WoM threshold.

## Self-Test

#### `QMI8658A_RunAccelSelfTest(qmi8658a_t *dev, qmi8658a_raw_axes_t *delta_raw, bool *passed)`

Disables sensors, sets `CTRL2.aST`, waits for `STATUSINT.Avail`, clears `aST`, then reads self-test output from `dVX_L` through `dVZ_H`.

The driver marks the test passed if all axes exceed the internal threshold equivalent used in the implementation.

#### `QMI8658A_RunGyroSelfTest(qmi8658a_t *dev, qmi8658a_raw_axes_t *delta_raw, bool *passed)`

Disables sensors, sets `CTRL3.gST`, waits for `STATUSINT.Avail`, clears `gST`, then reads self-test output from `dVX_L` through `dVZ_H`.

The driver marks the test passed if all axes exceed the internal threshold equivalent used in the implementation.

## Example Configurations

### Basic Accel + Gyro

```c
QMI8658A_SetAccelConfig(dev, QMI8658A_ACCEL_RANGE_4G, QMI8658A_ACCEL_ODR_250HZ, false);
QMI8658A_SetGyroConfig(dev, QMI8658A_GYRO_RANGE_512DPS, QMI8658A_GYRO_ODR_224_2HZ, false);
QMI8658A_SetFilter(dev,
                   QMI8658A_LPF_MODE_3_63_PERCENT, true,
                   QMI8658A_LPF_MODE_3_63_PERCENT, true);
QMI8658A_EnableSensors(dev, (qmi8658a_ctrl7_t)(QMI8658A_ACCEL_ENABLE | QMI8658A_GYRO_ENABLE));
```

### FIFO Stream Mode

```c
QMI8658A_FIFOReset(dev);
QMI8658A_FIFOConfig(dev, 16, QMI8658A_FIFO_SIZE_64_SAMPLES, QMI8658A_FIFO_MODE_STREAM);
QMI8658A_EnableSensors(dev, (qmi8658a_ctrl7_t)(QMI8658A_ACCEL_ENABLE | QMI8658A_GYRO_ENABLE));

uint8_t fifo_buf[256];
uint16_t read_len = 0;
if (QMI8658A_FIFORead(dev, fifo_buf, sizeof(fifo_buf), &read_len))
{
    /* Parse fifo_buf according to enabled sensors:
     * accel only: AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H, ...
     * gyro only:  GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H, ...
     * both:       accel 6 bytes, gyro 6 bytes, repeat.
     */
}
```

### Tap Detection

```c
qmi8658a_tap_config_t tap_cfg = {
    .peak_window = 20,
    .priority = 0,
    .tap_window = 50,
    .double_tap_window = 250,
    .alpha = 8,
    .gamma = 32,
    .peak_mag_thr = 0x0320,
    .udm_thr = 0x0190,
};

QMI8658A_EnableSensors(dev, QMI8658A_ACCEL_DISABLE);
QMI8658A_ConfigureTap(dev, &tap_cfg);
QMI8658A_EnableTap(dev, true);
QMI8658A_EnableSensors(dev, QMI8658A_ACCEL_ENABLE);
```

### Pedometer Example

```c
qmi8658a_pedometer_config_t ped_cfg = {
    .sample_count = 50,
    .fix_peak2peak = 0x00CC,
    .fix_peak = 0x0066,
    .time_up = 200,
    .time_low = 20,
    .cnt_entry = 10,
    .fix_precision = 0,
    .sig_count = 4,
};

QMI8658A_EnableSensors(dev, QMI8658A_ACCEL_DISABLE);
QMI8658A_ConfigurePedometer(dev, &ped_cfg);
QMI8658A_EnablePedometer(dev, true);
QMI8658A_EnableSensors(dev, QMI8658A_ACCEL_ENABLE);
```

## Notes and Limitations

- The driver performs sequential byte reads/writes through `read_reg` and `write_reg`. It enables auto-increment in `CTRL1`, but the interface abstraction does not expose a bulk transaction function.
- Activity engines should be configured while accelerometer and gyroscope are disabled, matching the datasheet guidance.
- The scaled output functions depend on the cached full-scale ranges. If application code writes `CTRL2` or `CTRL3` directly outside the driver, scaled values may be wrong until `QMI8658A_SetAccelConfig` or `QMI8658A_SetGyroConfig` is called again.
- FIFO parsing is left to the caller because the byte pattern depends on which sensors are enabled.
- Hardware bus errors cannot be detected by this driver unless the `accelIO_t` abstraction is extended to return status from low-level reads and writes.

## Internal Implementation Components

These symbols are not part of the public API, but they explain how the driver is assembled internally.

| Component | Location | Purpose |
| --- | --- | --- |
| `_QMI8658A_REG_WRITE` | `QMI8658A.c` | Writes one or more sequential registers using `dev->interface->write_reg`. |
| `_QMI8658A_REG_READ` | `QMI8658A.c` | Reads one or more sequential registers using `dev->interface->read_reg`. |
| `_QMI8658A_ValidDev` | `QMI8658A.c` | Central null check for device and interface pointers. |
| `_QMI8658A_S16` | `QMI8658A.c` | Combines little-endian low/high bytes into a signed 16-bit value. |
| `_QMI8658A_U16_TO_CAL` | `QMI8658A.c` | Packs a 16-bit value into the CAL mailbox buffer. |
| `_QMI8658A_AccelRangeG` | `QMI8658A.c` | Converts an accelerometer range enum into full-scale g. |
| `_QMI8658A_GyroRangeDps` | `QMI8658A.c` | Converts a gyroscope range enum into full-scale dps. |
| `_QMI8658A_WaitCtrl9Done` | `QMI8658A.c` | Polls `STATUSINT.bit7` until a CTRL9 command completes or times out. |
| `_QMI8658A_WriteCalAndRun` | `QMI8658A.c` | Shared helper for commands that write all eight CAL registers before executing CTRL9. |

Important constants:

| Macro | Value | Meaning |
| --- | ---: | --- |
| `QMI8658A_AXIS_X/Y/Z` | `0/1/2` | Axis indexes for array-style code. |
| `QMI8658A_AXES_NUM` | `3` | Number of axes. |
| `QMI8658A_CAL_REG_NUM` | `8` | Number of CAL mailbox registers. |
| `QMI8658A_WHO_AM_I_VALUE` | `0x05` | Expected chip ID. |
| `QMI8658A_RESET_CMD` | `0xB0` | Soft-reset command byte. |
| `QMI8658A_GRAVITY_EARTH` | `9.80665f` | Earth gravity constant for callers that want m/s^2 conversion. |
| `QMI8658A_CTRL9_TIMEOUT_MS` | `200` | Internal timeout used for ordinary CTRL9 commands. |
| `QMI8658A_COD_TIMEOUT_MS` | `1700` | Internal timeout used for calibration-on-demand. |

## Build Check

A syntax-only check can be run in a project that provides `target_port.h` and `io_driver/accel_io.h`:

```sh
gcc -std=c99 -Wall -Wextra -Werror -I. -fsyntax-only QMI8658A.c
```
