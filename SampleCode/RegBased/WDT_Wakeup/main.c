/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/07/03 4:42p $
 * @brief    Use WDT to wake up system from Power-down mode periodically.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

void WDT_IRQHandler(void)
{

    // Check WDT wake up flag
    if(WDT->CTL & WDT_CTL_WKF_Msk)
    {
        printf("Wake up by WDT\n");
    }

    // Clear WDT interrupt and wake up flag
    WDT->CTL |= WDT_CTL_WKF_Msk | WDT_CTL_IF_Msk;
}

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

    /* Enable HIRC, and LIRC (fro WDT) */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk)) !=
            (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk));


    /* Enable UART and WDT clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_WDTCKEN_Msk;

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

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART0->LINE = UART_LINE_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (0xBE);
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Init();

    printf("\nThis sample code demonstrate using WDT to wake system up from power down mode\n");

    // WDT register is locked, so it is necessary to unlock protect register before configure WDT
    while(SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    // WDT timeout every 2^14 WDT clock, enable system wake up
    WDT->CTL = WDT_TIMEOUT_2POW14 | WDT_CTL_WDTEN_Msk | WDT_CTL_INTEN_Msk | WDT_CTL_WKEN_Msk;

    NVIC_EnableIRQ(WDT_IRQn);

    while(1)
    {
        // Wait 'til UART FIFO empty to get a cleaner console out
        while(!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));
        // Enable sleep deep mode
        SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
        // Enable power down mode
        CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk);
        // Power down
        __WFI();
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


