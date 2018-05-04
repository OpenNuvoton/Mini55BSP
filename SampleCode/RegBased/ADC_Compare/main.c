/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/07/07 5:04p $
 * @brief    Demonstrate ADC conversion and comparison function by monitoring
 *           the conversion result of channel 5.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"


void ADC_IRQHandler(void)
{
    uint32_t reg = ADC->STATUS;
    // Get ADC Comapre result
    if(reg & ADC_CMP0_INT)
        printf("Channel 5 input < 0x200\n");
    if(reg & ADC_CMP1_INT)
        printf("Channel 5 input >= 0x200\n");

    // Clear ADC interrupt flag
    ADC->STATUS |= reg;
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

    /* Enable HIRC */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Enable UART and ADC clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_ADCCKEN_Msk;

    /* ADC clock source is HIRC, set divider to (3 + 1), ADC clock is HIRC/4 MHz */
    CLK->CLKDIV |= (3 << CLK_CLKDIV_ADCDIV_Pos);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P1.5 to ADC channel 5 input pin */
    SYS->P1_MFP = SYS_MFP_P15_AIN5;
    /* Analog pin OFFD to prevent leakage */
    P1->DINOFF |= GP_DINOFF_DINOFF5_Msk;

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

    printf("\nThis sample code demonstrate ADC conversion and comparison function\n");
    printf("by monitoring the conversion result of channel 5 (P1.5)\n");

    // Enable channel 5
    ADC->CHEN = 1 << 5;
    // Turn on ADC power and enable covert complete interrupt
    ADC->CTL = ADC_CTL_ADCEN_Msk | ADC_CTL_ADCIEN_Msk;

    // Configure and enable Comparator 0 to monitor channel 0 input less than 0x200
    ADC->CMP[0] = ADC_CMP_ADCMPEN_Msk |
                  ADC_CMP_ADCMPIE_Msk |
                  ADC_CMP_LESS_THAN |
                  (5 << ADC_CMP_CMPCH_Pos) |
                  (0xF << ADC_CMP_CMPMCNT_Pos) |
                  (0x200 << ADC_CMP_CMPDAT_Pos);
    // Configure and enable Comparator 1 to monitor channel 0 input greater or equal to 0x200
    ADC->CMP[1] = ADC_CMP_ADCMPEN_Msk |
                  ADC_CMP_ADCMPIE_Msk |
                  ADC_CMP_GREATER_OR_EQUAL_TO |
                  (5 << ADC_CMP_CMPCH_Pos) |
                  (0xF << ADC_CMP_CMPMCNT_Pos) |
                  (0x200 << ADC_CMP_CMPDAT_Pos);

    // Enable ADC interrupt
    NVIC_EnableIRQ(ADC_IRQn);

    while(1)
    {
        // Check if ADC is busy
        if(!(ADC->STATUS & ADC_STATUS_BUSY_Msk))
        {
            // Trigger ADC conversion
            ADC->CTL |= ADC_CTL_SWTRG_Msk;
        }
    }

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


