/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#ifndef __MLX9039X_H__
#define __MLX9039X_H__

#include <rtthread.h>

#define PI  3.1415926

#define MLX90392ELQ_AAA_010 0
#define MLX90392ELQ_AAA_011 1
#define MLX90392ELQ_AAA_013 2

#define MLX90397RLQ_AAA_000 3

#define MLX9039x    MLX90397RLQ_AAA_000
//#define MLX9039x    MLX90392ELQ_AAA_011

#if MLX9039x == MLX90392ELQ_AAA_010
#define MLX9039x_I2C_ADDRESS                    (0x0C)        // address pin A0,A1 low (GND), default for MLX90392
#define MAGNETIC_SENSITIVITY_XY   0.15    //uT/LSB magnetic flux resolution
#define MAGNETIC_SENSITIVITY_Z    0.15    //uT/LSB
#elif MLX9039x == MLX90392ELQ_AAA_011
#define MLX9039x_I2C_ADDRESS                    (0x0C)        // address pin A0,A1 low (GND), default for MLX90392
#define MAGNETIC_SENSITIVITY_XY   1.5    //uT/LSB magnetic flux resolution
#define MAGNETIC_SENSITIVITY_Z    1.5    //uT/LSB
#elif MLX9039x == MLX90392ELQ_AAA_013
#define MLX9039x_I2C_ADDRESS                    (0x3C)        // address pin A0,A1 low (GND), default for MLX90392
#define MAGNETIC_SENSITIVITY_XY   1.5    //uT/LSB magnetic flux resolution
#define MAGNETIC_SENSITIVITY_Z    1.5    //uT/LSB
#elif MLX9039x == MLX90397RLQ_AAA_000
#define MLX9039x_I2C_ADDRESS                    (0x0D)        //default for MLX90397
#define MAGNETIC_SENSITIVITY_XY   1.5    //uT/LSB magnetic flux resolution
#define MAGNETIC_SENSITIVITY_Z    1.5    //uT/LSB
#endif

#define TEMPERATURE_RES           50.0

#define MEM_ADDRESS_STAT1   0x0
#define MEM_ADDRESS_X       0x1
#define MEM_ADDRESS_Y       0x3
#define MEM_ADDRESS_Z       0x5
#define MEM_ADDRESS_STAT2   0x7
#define MEM_ADDRESS_T       0x8
#define MEM_ADDRESS_CID     0xA
#define MEM_ADDRESS_DID     0xB

#if MLX9039x == MLX90397RLQ_AAA_000
#define MEM_ADDRESS_CTRL    0xE
#else
#define MEM_ADDRESS_CTRL    0x10
#endif

union mlx9039x_stat1
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t drdy     : 1;    //BIT0
        rt_uint8_t stat1_1  : 1;
        rt_uint8_t stat1_2  : 1;
        rt_uint8_t rt       : 1;
        rt_uint8_t stat1_4  : 1;
        rt_uint8_t stat1_5  : 1;
        rt_uint8_t stat1_6  : 1;
        rt_uint8_t stat1_7  : 1;
    };
};

#if MLX9039x == MLX90397RLQ_AAA_000
union mlx9039x_stat2
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t hovf_x   : 1;    //BIT0
        rt_uint8_t hovf_y   : 1;
        rt_uint8_t hovf_z   : 1;
        rt_uint8_t dor      : 1;
        rt_uint8_t stat2_4  : 1;
        rt_uint8_t stat2_5  : 1;
        rt_uint8_t stat2_6  : 1;
        rt_uint8_t stat2_7  : 1;
    };
};
#else
union mlx9039x_stat2
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t hovf     : 1;    //BIT0
        rt_uint8_t dor      : 1;
        rt_uint8_t stat2_2  : 1;
        rt_uint8_t stat2_3  : 1;
        rt_uint8_t stat2_4  : 1;
        rt_uint8_t stat2_5  : 1;
        rt_uint8_t stat2_6  : 1;
        rt_uint8_t stat2_7  : 1;
    };
};
#endif

/* 3-axis data structure */
struct mlx9039x_xyz
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};

/* 3-axis data structure */
struct mlx9039x_xyz_flux
{
    float x;
    float y;
    float z;
};

enum mlx9039x_mode
{
    POWER_DOWN_MODE                     = 0x0,
    SINGLE_MEASUREMENT_MODE             = 0x1,
    CONTINUOUS_MEASUREMENT_MODE_10HZ    = 0x2,
    CONTINUOUS_MEASUREMENT_MODE_20HZ    = 0x3,
    CONTINUOUS_MEASUREMENT_MODE_50HZ    = 0x4,
    CONTINUOUS_MEASUREMENT_MODE_100HZ   = 0x5,
//    SELF_TEST_MODE                      = 0x6,
//    POWER_DOWN_MODE                     = 0x7,
//    POWER_DOWN_MODE                     = 0x8,
//    SINGLE_MEASUREMENT_MODE             = 0x9,
    CONTINUOUS_MEASUREMENT_MODE_200HZ   = 0xA,
    CONTINUOUS_MEASUREMENT_MODE_500HZ   = 0xB,
    CONTINUOUS_MEASUREMENT_MODE_700HZ   = 0xC,
    CONTINUOUS_MEASUREMENT_MODE_1400HZ  = 0xD,
//    SELF_TEST_MODE                      = 0xE,
//    POWER_DOWN_MODE                     = 0xF
};

enum mlx9039x_range_sel
{
    MAGNETIC_RANGE_XYZ_25MT_25MT_25MT   = 0x0,
    MAGNETIC_RANGE_XYZ_50MT_50MT_25MT   = 0x1,
    MAGNETIC_RANGE_XYZ_25MT_25MT_50MT   = 0x2,
    MAGNETIC_RANGE_XYZ_50MT_50MT_50MT   = 0x3,
    MAGNETIC_RANGE_XYZ_25MT_25MT_100MT  = 0x4,
    MAGNETIC_RANGE_XYZ_50MT_50MT_100MT  = 0x5,
    MAGNETIC_RANGE_XYZ_25MT_25MT_200MT  = 0x6,
    MAGNETIC_RANGE_XYZ_50MT_50MT_200MT  = 0x7,
};

union mlx9039x_osr_dig_filt
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t dig_filt_temp     : 3;
        rt_uint8_t dig_filt_hall_xy  : 3;
        rt_uint8_t osr_temp          : 1;
        rt_uint8_t osr_hall          : 1;    //BIT7
    };
};

union mlx9039x_cust_ctrl
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t dig_filt_hall_z  : 3;
        rt_uint8_t cust_ctrl3       : 1;
        rt_uint8_t dnc3_1           : 1;
        rt_uint8_t t_comp_en        : 1;
        rt_uint8_t dnc2_0           : 1;
        rt_uint8_t dnc1_1           : 1;    //BIT7
    };
};

union mlx9039x_cust_ctrl2
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t range_sel        : 3;
        rt_uint8_t cust_ctrl2_3     : 1;
        rt_uint8_t cust_ctrl2_4     : 1;
        rt_uint8_t cust_ctrl2_5     : 1;
        rt_uint8_t cust_ctrl2_6     : 1;
        rt_uint8_t cust_ctrl2_7     : 1;    //BIT7
    };
};

enum cmd
{
    CMD_NOP               = 0x00,
    CMD_EXIT              = 0x80,
    CMD_START_BURST       = 0x10,
    CMD_WAKE_ON_CHANGE    = 0x20,
    CMD_START_MEASUREMENT = 0x30,
    CMD_READ_MEASUREMENT  = 0x40,
    CMD_READ_REGISTER     = 0x50,
    CMD_WRITE_REGISTER    = 0x60,
    CMD_MEMORY_RECALL     = 0xd0,
    CMD_MEMORY_STORE      = 0xe0,
    CMD_RESET             = 0xf0
};

typedef enum
{
    Z_FLAG = 0x8,
    Y_FLAG = 0x4,
    X_FLAG = 0x2,
    T_FLAG = 0x1
} axis_flag_t;

/* Supported configuration items */
enum mlx9039x_cmd
{
    MPU6XXX_GYRO_RANGE,  /* Gyroscope full scale range */
    MPU6XXX_ACCEL_RANGE, /* Accelerometer full scale range */
    MPU6XXX_DLPF_CONFIG, /* Digital Low Pass Filter */
    MPU6XXX_SAMPLE_RATE, /* Sample Rate —— 16-bit unsigned value.
                            Sample Rate = [1000 -  4]HZ when dlpf is enable
                            Sample Rate = [8000 - 32]HZ when dlpf is disable */
    MPU6XXX_SLEEP        /* Sleep mode */
};

/** HALLCONF settings for CONF1 register. */
typedef enum mlx9039x_hallconf
{
    MLX90392_HALLCONF_0 = (0x0),
    MLX90392_HALLCONF_C = (0xC),
} mlx9039x_hallconf_t;

/** Gain settings for CONF1 register. */
typedef enum mlx9039x_gain
{
    MLX90392_GAIN_5X = (0x00),
    MLX90392_GAIN_4X,
    MLX90392_GAIN_3X,
    MLX90392_GAIN_2_5X,
    MLX90392_GAIN_2X,
    MLX90392_GAIN_1_67X,
    MLX90392_GAIN_1_33X,
    MLX90392_GAIN_1X
} mlx9039x_gain_t;

/** Resolution settings for CONF3 register. */
typedef enum mlx9039x_resolution
{
    MLX90392_RES_16,
    MLX90392_RES_17,
    MLX90392_RES_18,
    MLX90392_RES_19,
} mlx9039x_resolution_t;

/** Digital filter settings for CONF3 register. */
typedef enum mlx9039x_filter
{
    MLX90392_FILTER_0,
    MLX90392_FILTER_1,
    MLX90392_FILTER_2,
    MLX90392_FILTER_3,
    MLX90392_FILTER_4,
    MLX90392_FILTER_5,
    MLX90392_FILTER_6,
    MLX90392_FILTER_7,
} mlx9039x_filter_t;

/** Oversampling settings for CONF3 register. */
typedef enum mlx9039x_oversampling
{
    MLX90392_OSR_0,
    MLX90392_OSR_1,
    MLX90392_OSR_2,
    MLX90392_OSR_3,
} mlx9039x_oversampling_t;

union mlx9039x_status
{
    struct
    {
    /** D1-D0: indicates the number of bytes (2D[1:0]) to follow the status byte after a read measurement
     * or a read register command has been sent.
     */
    rt_uint8_t d0 : 1;
    /** D1-D0: indicates the number of bytes (2D[1:0]) to follow the status byte after a read measurement
     * or a read register command has been sent.
     */
    rt_uint8_t d1 : 1;

    /** RS: indicates that the device has been reset successfully by a reset command.
     */
    rt_uint8_t rs : 1;

    /** SED: indicates that a single bit error has been corrected by the NVRAM
     */
    rt_uint8_t sed : 1;

    /** ERROR: indicates an error.
     * Can be set when reading out a measurement while the measurement is not yet completed or
     * when reading out the same measurement twice.
     */
    rt_uint8_t error : 1;

    /** SM_mode: if set, the IC is executing a measurement sequence in polling mode.
     * It can be initiated by a SM command or a pulse on the TRIG input.
     */
    rt_uint8_t sm_mode : 1;

    /** WOC_mode: if set, the IC is in wake-up-on-change mode.
     */
    rt_uint8_t woc_mode : 1;

    /** Burst_mode: if set, the IC is working in burst mode.
     */
    rt_uint8_t burst_mode : 1;
    };

    rt_uint8_t byte_val;
};

/* mlx9039x config structure */
struct mlx9039x_config
{
    rt_uint8_t magnetic_range;
    rt_uint8_t dig_filt_hall_xy;
    rt_uint8_t dig_filt_hall_z;
    rt_uint8_t dig_filt_hall_temperature;
};

/* mlx9039x device structure */
struct mlx9039x_device
{
    rt_device_t bus;

    rt_uint8_t id;
    rt_uint8_t i2c_addr;

    float magnetic_sensitivity_xy;
    float magnetic_sensitivity_z;

    struct mlx9039x_config config;
};

/**
 * This function initialize the mlx9039x device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct mlx9039x_device *mlx9039x_init(const char *dev_name, rt_uint8_t param);

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx9039x_deinit(struct mlx9039x_device *dev);

/**
 * This function gets the data of the temperature, unit: Centigrade
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mlx9039x_get_temp(struct mlx9039x_device *dev, float *temp);


rt_err_t mlx9039x_nop(struct mlx9039x_device *dev);
rt_err_t mlx9039x_exit(struct mlx9039x_device *dev);
rt_err_t mlx9039x_reset(struct mlx9039x_device *dev);

rt_err_t mlx9039x_get_gain_sel(struct mlx9039x_device *dev, mlx9039x_gain_t *gain);
rt_err_t mlx9039x_get_resolution(struct mlx9039x_device *dev, mlx9039x_resolution_t *res_x, mlx9039x_resolution_t *res_y, mlx9039x_resolution_t *res_z);


rt_err_t mlx9039x_get_xyz(struct mlx9039x_device *dev, struct mlx9039x_xyz *xyz);
#endif
