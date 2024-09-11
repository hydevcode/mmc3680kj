/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-16     zhourq       the first version
 */
#ifndef APPLICATIONS_MMC3680KJ_MMC3680KJ_H_
#define APPLICATIONS_MMC3680KJ_MMC3680KJ_H_

#include <rtdevice.h>

#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif

#endif
#include "mmc3680kj.h"

#define MMC3680KJ_I2CBUS_NAME "i2c4"
#define MMC3680KJ_ADDR_DEFAULT 0x30

int rt_hw_mmc3680kj_init(const char *name, struct rt_sensor_config *cfg);


#endif /* APPLICATIONS_MMC3680KJ_MMC3680KJ_H_ */
