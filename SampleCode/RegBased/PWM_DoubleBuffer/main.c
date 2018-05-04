/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/07/07 3:51p $
 * @brief    Demonstrate the PWM double buffer feature.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

void PWM_IRQHandler(void)
{
    static int toggle = 0;

    // Update PWM channel 0 period and duty
    if(toggle == 0)
    {
        PWM->PERIOD[0] = 110;
        PWM->CMPDAT[0] = 50;
    }
    else
    {
        PWM->PERIOD[0] = 200;
        PWM->CMPDAT[0] = 100;
    }
    toggle ^= 1;
    // Clear channel 0 period interrupt flag;
    PWM->INTSTS |= PWM_INTSTS_ZIF0_Msk;
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

    /*  Read User Config to select internal high speed RC  */
    SystemInit();

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | 0;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_PWMCH01CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P2 multi-function pins for PWM Channel */
    SYS->P2_MFP = SYS_MFP_P22_PWM0;


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

    printf("\nThis sample code will output PWM channel 0 to output waveform\n");
    printf("using double buffer feature.\n");

    /*
        PWM channel 0 wave form of this sample shown below:

        |<-        CNR + 1  clk     ->|  CNR + 1 = 199 + 1 CLKs
                       |<-CMR+1 clk ->|  CMR + 1 = 99 + 1 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<CMR+1>|  CMR + 1 = 39 + 1 CLKs
         ______________                ________         _____
      __|      100     |_____100______|  60    |__40___|     PWM waveform

    */



    // Set channel 0 prescaler to 2. Actual value fill into register needs to minus 1.
    PWM->CLKPSC = 0x1;
    // Set channel 0 clock divider to 1
    PWM->CLKDIV = PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV0_Pos;
    // Enable PWM channel 0 auto-reload mode
    PWM->CTL = PWM_CTL_CNTMODE0_Msk;
    /*
      Configure PWM channel 0 init period and duty.
      Period is HCLK / (prescaler * clock divider * (CNR + 1))
      Duty ratio = (CMR + 1) / (CNR + 1)
      Period = 22.1184 MHz / (2 * 1 * (199 + 1)) =  55296 Hz
      Duty ratio = (99 + 1) / (199 + 1) = 50%
    */
    PWM->CMPDAT[0] = 99;
    PWM->PERIOD[0] = 199;


    // Enable PWM channel 0 output
    PWM->POEN = PWM_POEN_POEN0_Msk;

    // Enable PWM channel 0 period interrupt
    PWM->INTEN = PWM_INTEN_ZIEN0_Msk;
    NVIC_EnableIRQ(PWM_IRQn);

    // Start
    PWM->CTL |= PWM_CTL_CNTEN0_Msk;

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


