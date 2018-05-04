/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Date: 15/06/29 11:16a $
 * @brief    Demonstrate how to switch HCLK between HIRC and HXT.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Read User Config to select internal high speed RC */
    SystemInit();

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable HIRC, and HXT */
    CLK->PWRCTL &=~(CLK_PWRCTL_XTLEN_Msk | CLK_PWRCTL_HIRCEN_Msk);
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_XTL12M;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_XTLSTB_Msk)) !=
            (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_XTLSTB_Msk));

    /* Enable UART clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Set P3 multi-function pins for Clock Output */
    SYS->P3_MFP = SYS_MFP_P36_CKO;

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS->REGLCTL = 0;
}

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART0->LINE = UART_LINE_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (0xBE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+----------------------------------------+\n");
    printf("|         CLK Driver Sample Code         |\n");
    printf("+----------------------------------------+\n");

    /*---------------------------------------------------------------------------------------------------------*/
    /* Switch HCLK function test                                                                               */
    /* Measure CLKO(P3.6) to check switch HCLK between HIRC and HXT                                            */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable CKO and output frequency = system clock / 4 */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | 1 | 0<<CLK_CLKOCTL_DIV1EN_Pos;
    CLK->APBCLK |= CLK_APBCLK_CLKOCKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_FDIVSEL_Msk)) | CLK_CLKSEL2_FDIVSEL_HCLK;

    /* Unlock protected registers */
    while(SYS->REGLCTL != SYS_REGLCTL_REGPROTDIS_Msk)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    while(1)
    {
        printf("Switch HCLK clock source to HIRC\n");
        /* Switch HCLK clock source to HIRC */
        CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
        CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | CLK_CLKDIV_HCLK(1);

        /* Delay (~1sec) */
        CLK_SysTickDelay(500000);
        CLK_SysTickDelay(500000);

        printf("Switch HCLK clock source to HXT\n");
        /* Switch HCLK clock source to HXT */
        CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_XTAL;
        CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | CLK_CLKDIV_HCLK(1);

        /* Delay (~1sec) */
        CLK_SysTickDelay(500000);
        CLK_SysTickDelay(500000);
    }

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


