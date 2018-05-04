/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 15/07/09 8:55a $
 * @brief    Demonstrate Analog comparator (ACMP) comparison by
 *           comparing CPP0 (P1.5) with VBG voltage and shows the result
 *           on UART console.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"


void ACMP_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    u32Cnt++;
    /* Clear ACMP 0 interrupt flag */
    ACMP->STATUS = ACMP_STATUS_ACMPIF0_Msk;

    /* Check Comparator 0 Output Status */
    if (ACMP->STATUS & ACMP_STATUS_ACMPO0_Msk)
        printf("CPP0 > CPN0 (%d)\n", u32Cnt);
    else
        printf("CPP0 <= CPN0 (%d)\n", u32Cnt);
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

    /* Enable UART and ACMP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_ACMPCKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P1 multi-function pins for ACMP CPP0*/
    SYS->P1_MFP = SYS_MFP_P15_CPP0;

    /* Set P3.6 to ACMP CPO0 function */
    SYS->P3_MFP = SYS_MFP_P36_CPO0;

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

    printf("\nThis sample code demonstrate ACMP0 function. Using CPP0 (P1.5) as ACMP0\n");
    printf("positive input and internal bandgap voltage as the negative input\n");
    printf("The compare result reflects on CPO0 (P3.6)\n");

    /* Configure ACMP0 Comparator 0. Enable ACMP0, enable interrupt and select internal reference voltage as negative input */
    ACMP->CTL[0] = ACMP_CTL_ACMPEN_Msk | ACMP_CTL_ACMPIE_Msk | ACMP_CTL_NEGSEL_Msk;

    // Enable ADC interrupt
    NVIC_EnableIRQ(ACMP_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


