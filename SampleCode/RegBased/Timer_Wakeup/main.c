/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/07/03 4:59p $
 * @brief    Use timer to wake up system from Power-down mode periodically.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"


void TMR0_IRQHandler(void)
{

    // clear timer interrupt flag and wake up flag
    TIMER0->INTSTS = TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk;

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

    /* Enable HIRC */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk)) != (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk));

    /* Enable UART and Timer 0 clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_TMR0CKEN_Msk;

    /* Select Timer clock source from LIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_LIRC;

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
    UART0->BAUD = UART_BAUD_BAUDM0_Msk | UART_BAUD_BAUDM1_Msk | (0xBE);
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Init();

    printf("\nThis sample code use timer to wake up system every 1 second \n");

    // To generate interrupt every second, we set prescale = 1, so timer clock is 10k / (1 + 1)
    // And set compare value to 10000 / 2, timer will generate time out interrupt every 1 second
    TIMER0->CMP = (10000 / 2);
    // Enable timer counter , enable timer wake up, enable timer interrupt, set timer operating in periodic mode, and set prescale to 1
    TIMER0->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_WKEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_PERIODIC_MODE | (1);
    // Enable timer interrupt
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Unlock protected registers to control PWRCTL */
    while(SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    while(1)
    {
        printf("Sleep 1 second\n");
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


