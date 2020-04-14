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
#include "fsl_device_registers.h"

#include "fsl_lpspi.h"
#include "board.h"

#include "fsl_common.h"
#include "pin_mux.h"
#if ((defined FSL_FEATURE_SOC_INTMUX_COUNT) && (FSL_FEATURE_SOC_INTMUX_COUNT))
#include "fsl_intmux.h"
#endif
/* GPIO1_IO09 */
#define LED0_PIN               GET_PIN(1,9)
#define SPI1_CS               GET_PIN(3,13)
#define EXAMPLE_LPSPI_CLOCK_SOURCE_SELECT (1U)
#define EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER (7U)
#define LPSPI_MASTER_CLK_FREQ (CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER + 1U))
#define TRANSFER_SIZE 64U  
uint8_t masterRxData[TRANSFER_SIZE] = {0U};
uint8_t masterTxData[TRANSFER_SIZE] = {0U};



#define W25Q_SPI_DEVICE_NAME     "spi10"

void spi_w25q_sample(void)
{
    struct rt_spi_device *spi_dev_w25q;
    char name[RT_NAME_MAX];
    rt_uint8_t w25x_read_id[64] = {1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5};
    rt_uint8_t id[64] = {0,6,7,8,9,0,6,7,8,9,0,6,7,8,9,0,6,7,8,9,0,6,7,8,9,0,6,7,8,9,0,6,7,8,9,0,6,7,8,9};
    BOARD_InitPins();
//    if (argc == 2)
//    {
//        rt_strncpy(name, argv[1], RT_NAME_MAX);
//    }
//    else
//    {
//        rt_strncpy(name, W25Q_SPI_DEVICE_NAME, RT_NAME_MAX);
//    }

    /* ?? spi ???????? */
    spi_dev_w25q = (struct rt_spi_device *)rt_device_find(W25Q_SPI_DEVICE_NAME);
    if (!spi_dev_w25q)
    {
        rt_kprintf("spi sample run failed! can't find %s device!\n", W25Q_SPI_DEVICE_NAME);
    }
    else
    {
			    struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz = 50 * 1000 *1000;                           /* 20M */

        rt_spi_configure(spi_dev_w25q, &cfg);
        /* ??1:?? rt_spi_send_then_recv()??????ID */
//        rt_spi_send_then_recv(spi_dev_w25q, w25x_read_id, 64, id, 64);
//        rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:%x%x\n", id[3], id[4]);

//        /* ??2:?? rt_spi_transfer_message()??????ID */
//        struct rt_spi_message msg1, msg2;

//        msg1.send_buf   = w25x_read_id;
//        msg1.recv_buf   = RT_NULL;
//        msg1.length     = 1;
//        msg1.cs_take    = 1;
//        msg1.cs_release = 0;
//        msg1.next       = &msg2;

//        msg2.send_buf   = RT_NULL;
//        msg2.recv_buf   = id;
//        msg2.length     = 5;
//        msg2.cs_take    = 0;
//        msg2.cs_release = 1;
//        msg2.next       = RT_NULL;

//        rt_spi_transfer_message(spi_dev_w25q, &msg1);
        rt_kprintf("use rt_spi_transfer_message() read w25q ID is:%x%x\n", id[3], id[4]);

    }
}
int main(void)
{
#ifndef PHY_USING_KSZ8081
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	spi_w25q_sample();
	BOARD_InitPins();
//	BOARD_BootClockRUN();
//    while (1)
//    {
//        rt_pin_write(LED0_PIN, PIN_HIGH);
//        rt_thread_mdelay(500);
//        rt_pin_write(LED0_PIN, PIN_LOW);
//        rt_thread_mdelay(500);
//    }
	
	
	
//	uint8_t g_masterRxWatermark;
//uint8_t g_masterFifoSize;
//		CLOCK_SetMux(kCLOCK_LpspiMux, EXAMPLE_LPSPI_CLOCK_SOURCE_SELECT);
//    CLOCK_SetDiv(kCLOCK_LpspiDiv, EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER);
//	  uint32_t srcClock_Hz;
//    uint32_t errorCount;

//    uint32_t whichPcs;
//    uint8_t txWatermark;
//    lpspi_master_config_t masterConfig;
//bool isMasterTransferCompleted = false;
//		volatile uint32_t masterTxCount;
//volatile uint32_t masterRxCount;
//    /*Master config*/
//    masterConfig.baudRate     = 500000U;
//    masterConfig.bitsPerFrame = 8;
//    masterConfig.cpol         = kLPSPI_ClockPolarityActiveHigh;
//    masterConfig.cpha         = kLPSPI_ClockPhaseFirstEdge;
//    masterConfig.direction    = kLPSPI_MsbFirst;

//    masterConfig.pcsToSckDelayInNanoSec        = 1000000000 / masterConfig.baudRate;
//    masterConfig.lastSckToPcsDelayInNanoSec    = 1000000000 / masterConfig.baudRate;
//    masterConfig.betweenTransferDelayInNanoSec = 1000000000 / masterConfig.baudRate;

//    masterConfig.whichPcs           = kLPSPI_Pcs0;
//    masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

//    masterConfig.pinCfg        = kLPSPI_SdiInSdoOut;
//    masterConfig.dataOutConfig = kLpspiDataOutRetained;

//    srcClock_Hz = LPSPI_MASTER_CLK_FREQ;
//    LPSPI_MasterInit(LPSPI1, &masterConfig, srcClock_Hz);
//    int i=0;
//    /******************Set up master transfer******************/
//    /*Set up the transfer data*/
//    for (i = 0; i < TRANSFER_SIZE; i++)
//    {
//        masterTxData[i] = i % 256;
//        masterRxData[i] = 0;
//    }

//  //  g_masterFifoSize = LPSPI_GetRxFifoSize(LPSPI1);

//    isMasterTransferCompleted = false;
//    masterTxCount             = 0;
//    masterRxCount             = 0;
//    whichPcs                  = kLPSPI_Pcs0;

//    /*The TX and RX FIFO sizes are always the same*/
//    g_masterFifoSize = LPSPI_GetRxFifoSize(LPSPI1);

//    /*Set the RX and TX watermarks to reduce the ISR times.*/
//    if (g_masterFifoSize > 1)
//    {
//        txWatermark         = 1;
//        g_masterRxWatermark = g_masterFifoSize - 2;
//    }
//    else
//    {
//        txWatermark         = 0;
//        g_masterRxWatermark = 0;
//    }

//    LPSPI_SetFifoWatermarks(LPSPI1, txWatermark, g_masterRxWatermark);

//  //  LPSPI_Enable(LPSPI1, false);
//    LPSPI1->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
//    ///LPSPI_Enable(LPSPI1, true);

//    /*Flush FIFO , clear status , disable all the inerrupts.*/
//    LPSPI_FlushFifo(LPSPI1, true, true);
//    LPSPI_ClearStatusFlags(LPSPI1, kLPSPI_AllStatusFlag);
//    LPSPI_DisableInterrupts(LPSPI1, kLPSPI_AllInterruptEnable);

//    LPSPI1->TCR =
//        (LPSPI1->TCR &
//         ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK)) |
//        LPSPI_TCR_CONT(0) | LPSPI_TCR_CONTC(0) | LPSPI_TCR_RXMSK(0) | LPSPI_TCR_TXMSK(0) | LPSPI_TCR_PCS(whichPcs);

//		    rt_pin_mode(SPI1_CS, PIN_MODE_OUTPUT);
//    rt_pin_write(SPI1_CS, PIN_LOW);
//    lpspi_transfer_t transfer;
//   transfer.dataSize = TRANSFER_SIZE;
//    transfer.rxData   = (uint8_t*)(masterRxData);
//    transfer.txData   = (uint8_t*)(masterTxData);
//    /*Fill up the TX data in FIFO */
//    LPSPI_MasterTransferBlocking(LPSPI1, &transfer);
//    rt_pin_write(SPI1_CS, PIN_HIGH);
#endif
}

