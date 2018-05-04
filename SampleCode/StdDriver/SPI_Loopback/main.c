/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 7 $
 * $Date: 15/07/06 9:49a $
 * @brief    Demonstrate SPI function by connect MOSI (P0.5) with MISO (P0.6)
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000
#define TEST_COUNT             64

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);
void SpiLoopbackTest(void);

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                       SPI Driver Sample Code                         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf(" Please Connect:\n");
    printf(" P0.5 MOSI <--> P0.6 MISO \n");

    SpiLoopbackTest();

    while(1);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Read User Config to select internal high speed RC */
    SystemInit();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC */
    CLK->PWRCTL =  CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPI_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART0SEL_HIRC,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Setup SPI multi-function pin */
    SYS->P0_MFP |= SYS_MFP_P04_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK;

    /* Lock protected registers */
    SYS_LockReg();

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI, SPI_SS, SPI_SS_ACTIVE_LOW);
}

void SpiLoopbackTest(void)
{
    uint32_t u32DataCount, u32TestCount, u32Err;

    printf("\nSPI Loopback test ");

    /* Clear Tx register of SPI to avoid send non-zero data to Master. Just for safe. */
    SPI_WRITE_TX(SPI, 0);

    u32Err = 0;
    for(u32TestCount=0; u32TestCount<100; u32TestCount++)
    {
        /* set the source data and clear the destination buffer */
        for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++)
        {
            g_au32SourceData[u32DataCount] = u32DataCount;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount=0;

        if((u32TestCount&0x1FF) == 0)
        {
            putchar('.');
        }

        while(1)
        {
            SPI_WRITE_TX(SPI, g_au32SourceData[u32DataCount]);
            SPI_TRIGGER(SPI);
            while(SPI_IS_BUSY(SPI));

            g_au32DestinationData[u32DataCount] = SPI_READ_RX(SPI);
            u32DataCount++;
            if(u32DataCount > TEST_COUNT)    break;
        }

        /*  Check the received data */
        for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++)
        {
            if(g_au32DestinationData[u32DataCount]!=g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    return ;
}


