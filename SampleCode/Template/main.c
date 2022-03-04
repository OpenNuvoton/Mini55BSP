/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/06/29 11:16a $
 * @brief    A project template for Mini55 MCU.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"


int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Read User Config to select internal high speed RC  */
    if (SystemInit() < 0)
        return -1;

    // Enable UART clock
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Lock protected registers */
    SYS_LockReg();
    return 0;
}


int main()
{
    int32_t retval;

    retval = SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    printf("Hello World\n");


    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
