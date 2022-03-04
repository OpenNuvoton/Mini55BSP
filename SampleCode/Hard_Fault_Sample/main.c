/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/07/08 3:49p $
 * @brief    Show hard fault information when hard fault happened.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Mini55Series.h"


int32_t SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Read User Config to select internal high speed RC  */
    if (SystemInit() < 0)
        return -1;

    /* Enable External XTAL and HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE); // UART Clock Enable

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P1 multi-function pins for UART0 RXD and TXD  */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_RXD | SYS_MFP_P13_TXD);

    /* Lock protected registers */
    SYS_LockReg();
    return 0;
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    int32_t retval;
    char *tmp = (char *)0x4;

    /* Init System, IP clock and multi-function I/O */
    retval = SYS_Init();
    /* Init UART for printf */
    UART_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    strcpy(tmp,"HardFaultTest");

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


