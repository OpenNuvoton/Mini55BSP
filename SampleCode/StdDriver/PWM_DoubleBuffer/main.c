/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/07/07 3:41p $
 * @brief    Demonstrate the PWM double buffer feature.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

uint32_t duty30, duty60;
void PWM_IRQHandler(void)
{
    static int toggle = 0;  // First two already fill into PWM, so start from 30%

    // Update PWM channel 0 duty
    if(toggle == 0)
    {
        PWM_SET_CMR(PWM, 0, duty30);
    }
    else
    {
        PWM_SET_CMR(PWM, 0, duty60);
    }
    toggle ^= 1;
    // Clear channel 0 period interrupt flag
    PWM_ClearPeriodIntFlag(PWM, 0);
}



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /*  Read User Config to select internal high speed RC  */
    SystemInit();

    /* Enable external 12MHz XTAL (UART), HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_XTAL,CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_PWMCH01CKEN_Msk | CLK_APBCLK_PWMCH23CKEN_Msk | CLK_APBCLK_PWMCH45CKEN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_XTAL;

    /* Select PWM clock source from external crystal*/
    CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWMCH01SEL_HCLK, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P2 multi-function pin for PWM Channel 0  */
    SYS->P2_MFP = SYS_MFP_P22_PWM0;


    /* Lock protected registers */
    SYS_LockReg();
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
    UART_Open(UART0, 115200);

    /*
        PWM channel 0 wave form of this sample changed between 30% and 60% duty ratio
    */
    PWM_ConfigOutputChannel(PWM, PWM_CH0, 1000, 30);

    // Save 30% duty setting
    duty30 = PWM->CMPDAT[PWM_CH0];
    // Calculate 60% duty setting. CMPDAT store the actual value minus 1.
    duty60 = (duty30 + 1) * 2 - 1;

    // Enable output of all PWM channel 0
    PWM_EnableOutput(PWM, 1);

    // Enable PWM channel 0 period interrupt
    PWM_EnablePeriodInt(PWM, PWM_CH0);
    NVIC_EnableIRQ(PWM_IRQn);

    // Start
    PWM_Start(PWM, 0x1);

    // Fill second duty setting immediately after PWM start
    PWM_SET_CMR(PWM, PWM_CH0, duty60);

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


