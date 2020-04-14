/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-04-29     tyustli      first version
 */

#include "MIMXRT1062.h"
#include <rtdevice.h>
#include "drv_gpio.h"
#include "core_cm7.h"

/* defined the LED pin: GPIO1_IO9 */
#define LED0_PIN               GET_PIN(1, 9)
#define CDC_UART_NAME	"vcom"

struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};
static rt_device_t serial;
static struct rt_messagequeue rx_mq;
static char msg_pool[256];
static char rx_buffer[RT_SERIAL_RB_BUFSZ + 1];

static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
	struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
    if ( result == -RT_EFULL)
    {
        rt_kprintf("message queue full!\n");
    }
    return result;
}

static void usb_cdc_test_entry(void *parameter)
{
	
	char str[] = "hello RT-Thread!\r\n";
	struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
	
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	
	serial = rt_device_find(CDC_UART_NAME);
	if (!serial)
	{
		rt_kprintf("find %s failed!\n", CDC_UART_NAME);
		return;
	}
	
    rt_mq_init(&rx_mq, "rx_mq",
               msg_pool,                 
               sizeof(struct rx_msg),    
               sizeof(msg_pool),         
               RT_IPC_FLAG_FIFO);        

    result = rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
	if (result != RT_EOK)
	{
		rt_kprintf("Open vcom failed:%d\n",result);
		return;
	}
    rt_device_set_rx_indicate(serial, uart_input);

    rt_device_write(serial, 0, str, (sizeof(str) - 1));
	
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
            rx_buffer[rx_length] = '\0';
            rt_device_write(serial, 0, rx_buffer, rx_length);
            rt_kprintf("%s\n",rx_buffer);
        }
    }
}

int usb_cdc_test(void)
{
	int ret =0;
    rt_thread_t thread = rt_thread_create("serial", usb_cdc_test_entry, RT_NULL, 1024, 25, 10);
    
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }
	
	return ret;
}

MSH_CMD_EXPORT(usb_cdc_test, usb_cdc vcom demo)