/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/07/07 3:51p $
 * @brief    Show how to user divider API and how to use hardware divider
 *           by control registers
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
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

    /*  Read User Config to select internal high speed RC  */
    SystemInit();

    /* Enable HIRC */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Lock protected registers */
    SYS->REGLCTL = 0;
}


void UART0_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART0->LINE = UART_LINE_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (0xBE);
}

void HDIV_Init(void)
{
    /* Enable Hardware Divider Clock */
    CLK->AHBCLK |= CLK_AHBCLK_HDIVCKEN_Msk;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init Divider */
    HDIV_Init();

    printf("+----------------------------------------------+\n");
    printf("|    Mini55 Divider Sample Code                |\n");
    printf("+----------------------------------------------+\n");
    printf("\n");

    printf("12341 / 123 = %d\n", HDIV_Div(12341, 123));
    printf("12341 %% 123 = %d\n", HDIV_Mod(12341, 123));

    HDIV->DIVIDEND = 12341;
    HDIV->DIVISOR = 123;
    printf("12341 / 123 = %d, remainder = %d\n", HDIV->QUOTIENT, HDIV->REM);

    printf("Done\n");

    /* Disable Hardware Divider Clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_HDIVCKEN_Msk;

    while(SYS->PDID);
    return 0;
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
