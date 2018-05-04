/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/07/07 3:51p $
 * @brief    Demonstrate the dead-zone feature with PWM.
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

    /* Set P2 multi-function pins for PWM Channel 0 and 1 */
    SYS->P2_MFP = SYS_MFP_P22_PWM0 | SYS_MFP_P23_PWM1;


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

    printf("\nThis sample code will output PWM channel 0 and 1 with\n");
    printf("1.2kHz 50%% duty waveform, and 10us deadzone\n");


    // Set channel 0 prescaler to 2. Actual value fill into register needs to minus 1.
    PWM->CLKPSC = 0x1;
    // Set channel 0 clock divider to 1
    PWM->CLKDIV = (PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV0_Pos);
    // Enable PWM channel 0 auto-reload mode, enable Dead Zone.
    // No need to configure channel 1 because it's the inverse of channel 0 output here
    PWM->CTL = PWM_CTL_CNTMODE0_Msk | PWM_CTL_DTCNT01_Msk;
    /*
      Configure PWM channel 0 init period and duty.
      Period is HCLK / (prescaler * clock divider * (CNR + 1))
      Duty ratio = (CMR + 1) / (CNR + 1)
      Period = 22.1184 MHz / (2 * 1 * (9215 + 1)) =  1200 Hz
      Duty ratio = (4607 + 1) / (9215 + 1) = 50%
    */
    PWM->CMPDAT[0] = 4607;
    PWM->PERIOD[0] = 9215;

    // Configure dead zone duration to 10us.
    // 22118400(clk) / 2(perscaler) * 10 * 10^-6 ~= 111
    PWM->DTCTL = 111;

    // Enable PWM channel 0 and 1 output
    PWM->POEN = PWM_POEN_POEN0_Msk | PWM_POEN_POEN1_Msk;

    // Start
    PWM->CTL |= PWM_CTL_CNTEN0_Msk;

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


