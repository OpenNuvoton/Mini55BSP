/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/07/07 3:28p $
 * @brief    Demonstrate ADC conversion and comparison function by
 *           monitoring the conversion result of channel 0.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    // Get ADC comparator interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_CMP0_INT | ADC_CMP1_INT);

    if(u32Flag & ADC_CMP0_INT)
        printf("Channel 0 input < 0x200\n");
    if(u32Flag & ADC_CMP1_INT)
        printf("Channel 0 input >= 0x200\n");

    ADC_CLR_INT_FLAG(ADC, u32Flag);
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

    /* Enable external 12MHz XTAL, HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_XTAL,CLK_CLKDIV_HCLK(1));

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_XTAL);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART0SEL_XTAL,CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(ADC_MODULE,CLK_CLKSEL1_ADCSEL_XTAL,CLK_CLKDIV_ADC(6));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Set P5.3 to ADC channel 0 input pin */
    SYS->P5_MFP = SYS_MFP_P53_AIN0;

    /* Analog pin OFFD to prevent leakage */
    P5->DINOFF |= GP_DINOFF_DINOFF3_Msk;

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

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

    printf("\nThis sample code demonstrate ADC conversion and comparison function\n");
    printf("by monitoring the conversion result of channel 0 (P5.3)\n");

    // Enable channel 0
    ADC_Open(ADC, 0, 0, 0x01);

    // Power on ADC
    ADC_POWER_ON(ADC);

    // Configure and enable Comparator 0 to monitor channel 0 input less than 0x200
    ADC_ENABLE_CMP0(ADC, 0, ADC_CMP_LESS_THAN, 0x200, 16);
    // Configure and enable Comparator 1 to monitor channel 0 input greater or equal to 0x200
    ADC_ENABLE_CMP1(ADC, 0, ADC_CMP_GREATER_OR_EQUAL_TO, 0x200, 16);

    // Enable ADC comparator 0 and 1 interrupt
    ADC_EnableInt(ADC, ADC_CMP0_INT);
    ADC_EnableInt(ADC, ADC_CMP1_INT);
    NVIC_EnableIRQ(ADC_IRQn);

    while(1)
    {
        // Trigger ADC conversion if it is idle
        if(!ADC_IS_BUSY(ADC))
        {
            ADC_START_CONV(ADC);
        }
    }

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


