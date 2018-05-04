/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/07/07 3:47p $
 * @brief    Demonstrate ADC function by repeatedly convert the input of
 *           ADC channel 0 (P5.3) and shows the result on UART console.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"


void ADC_IRQHandler(void)
{
    // Get ADC convert result
    printf("Convert result is %x\n", (uint32_t)(ADC->DAT & ADC_DAT_RESULT_Msk));

    // Clear ADC convert complete flag
    ADC->STATUS |= ADC_STATUS_ADIF_Msk;
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

    /* ADC clock source is HIRC, set divider to (3 + 1), ADC clock is HIRC */
    CLK->CLKDIV |= (3 << CLK_CLKDIV_ADCDIV_Pos);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P5.3 to ADC channel 0 input pin */
    SYS->P5_MFP = SYS_MFP_P53_AIN0;
    /* Analog pin OFFD to prevent leakage */
    P5->DINOFF |= GP_DINOFF_DINOFF3_Msk;

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

    printf("\nThis sample code demonstrate ADC channel 0 (P5.3)conversion function\n");

    // Enable channel 0
    ADC->CHEN = 1;
    // Turn on ADC power and enable covert complete interrupt
    ADC->CTL = ADC_CTL_ADCEN_Msk | ADC_CTL_ADCIEN_Msk;



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


