/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-12-13     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG

#include <rtdbg.h>

int main(void)
{
    int count = 1;
//    re_pin_mode(GET_PIN(F, 9), PIN_MODE_OUTPUT);
    while (count++)
    {
//        rt_pin_write(GET_PIN(F, 9), PIN_LOW);
        LOG_D("Hello RT-Thread!");
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}
