/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#ifndef __MELEXIS_MLX90392_SENSOR_V1_H__
#define __MELEXIS_MLX90392_SENSOR_V1_H__

#include <rtdevice.h>

#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif
#endif

#include "mlx9039x.h"

#define MLX90392_I2C_ADDRESS                    0x19 // address pin A0/A1 low(GND), default for Melexis MLX90393

#define MLX90392_CTRL_NOP                       0x10
#define MLX90392_CTRL_RESET                     0x11

int rt_hw_mlx9039x_init(const char *name, struct rt_sensor_config *cfg);

#endif
