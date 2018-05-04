/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/07/07 3:34p $
 * @brief    Show how to user divider API and how to use hardware divider
 *           by control registers
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Register write-protection disabled */
    SYS_UnlockReg();

    /* Read User Config to select internal high speed RC  */
    SystemInit();

    /* Enable external 12MHz XTAL (UART), HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_XTAL,CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;
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
    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Enable Hardware Divider Clock */
    HDIV_Init();

    printf("+----------------------------------------------+\n");
    printf("|    Mini55 Divider Sample Code                |\n");
    printf("+----------------------------------------------+\n");
    printf("\n");

    printf("12341 / 123 = %d\n", HDIV_Div(12341, 123));
    printf("12341 %% 123 = %d\n", HDIV_Mod(12341, 123));

    /* Lock protected registers */
    SYS_LockReg();

    printf("Done\n");

    /* Disabl Hardware Divider Clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_HDIVCKEN_Msk;

    while(SYS->PDID);
    return 0;
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
