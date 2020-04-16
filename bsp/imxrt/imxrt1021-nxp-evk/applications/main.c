/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-06     tyustli      first version
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"
#include <board.h>
/* GPIO1_IO05 */
#define LED0_PIN               GET_PIN(1,5)
#define LED0_KEY               GET_PIN(5,0)
//int main(void)
//{
//#ifndef PHY_USING_KSZ8081
//    /* set LED0 pin mode to output */
//    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
////	rt_pin_mode(LED0_KEY, PIN_MODE_INPUT_PULLUP);

////    while (1)
////    {
//        rt_pin_write(LED0_PIN, PIN_HIGH);
//        rt_thread_mdelay(500);
//        rt_pin_write(LED0_PIN, PIN_LOW);
//        rt_thread_mdelay(500);


//	
//					/* 打开 MicroPython 命令交互界面 */
//		extern void mpy_main(const char *filename);
//		mpy_main(NULL);
////	}
//#endif	
//}



int main(void)
{
#ifndef PHY_USING_KSZ8081
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	BOARD_InitPins();

#endif
}
