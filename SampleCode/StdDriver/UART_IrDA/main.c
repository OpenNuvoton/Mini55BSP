/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 6 $
 * $Date: 15/07/08 4:05p $
 * @brief    Transmit and receive UART data in UART IrDA mode.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

#include "uart.h"


#define RXBUFSIZE 1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] = {0};
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;
volatile int32_t g_i32pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);

extern void IrDA_FunctionTest(void);


int32_t SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Read User Config to select internal high speed RC */
    if (SystemInit() < 0)
        return -1;

    /* Set P5 multi-function pins for crystal output/input */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL &= ~CLK_PWRCTL_XTLEN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_XTLEN_Pos); // XTAL12M (HXT) Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_XTLSTB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_XTAL;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0CKEN_Msk; // UART Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART0SEL_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART1 RXD and TXD  */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_RXD | SYS_MFP_P13_TXD);

    /* Set P0 multi-function pins for UART1 RTS and CTS */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_CTS | SYS_MFP_P01_RTS);

    /* Lock protected registers */
    SYS_LockReg();
    return 0;
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    int32_t retval;

    /* Init System, IP clock and multi-function I/O */
    retval = SYS_Init();
    /* Init UART for printf */
    UART_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+------------------------+\n");
    printf("| IrDA function test |\n");
    printf("+------------------------+\n");

    IrDA_FunctionTest();

    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{

}





