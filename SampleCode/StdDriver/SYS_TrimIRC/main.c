/**************************************************************************//**
* @file     main.c
* @version  V1.00
* $Revision: 1 $
* $Date: 15/07/03 2:55p $
* @brief    Demonstrate how to use LXT to trim HIRC
*
* @note
* Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Mini55Series.h"
/*---------------------------------------------------------------------------------------------------------*/
/*  IRCTrim IRQ Handler                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void HIRC_IRQHandler()
{
    uint32_t status;
    status=SYS_GET_IRCTRIM_INT_FLAG();
    if(status & BIT1)
    {
        printf("Trim Failure Interrupt\n");
        SYS_CLEAR_IRCTRIM_INT_FLAG(SYS_IRCTISTS_TFAILIF_Msk);
    }
    if(status & BIT2)
    {
        SYS_CLEAR_IRCTRIM_INT_FLAG(SYS_IRCTISTS_CLKERRIF_Msk);
        printf("LXT Clock Error Lock\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*  Read User Config to select internal high speed RC  */
    SystemInit();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC,CLK_CLKDIV_HCLK(1));

    /* STCLK to HIRC */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_HCLKSEL_HIRC);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART0SEL_HIRC,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Set P3 multi-function pins for Clock Output */
    SYS->P3_MFP = SYS_MFP_P36_CKO;

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

int32_t main (void)
{
    uint32_t status;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);


    printf("+----------------------------------------+\n");
    printf("|      Mni55 Trim IRC Sample Code        |\n");
    printf("+----------------------------------------+\n");

    /* Enable Interrupt */
    NVIC_EnableIRQ(HIRC_IRQn);

    /*  Enable IRC Trim, set HIRC clock to 22.1184Mhz and enable interrupt */
    SYS_EnableIRCTrim(SYS_IRCTCTL_TRIM_22_1184M,SYS_IRCTIEN_CLKEIEN_Msk|SYS_IRCTIEN_TFAILIEN_Msk);

    /* Waiting for HIRC Frequency Lock */
    CLK_SysTickDelay(2000);

    status=SYS_GET_IRCTRIM_INT_FLAG();
    if(status & BIT0)
        printf("HIRC Frequency Lock\n");

    /* Enable CKO and output frequency = HIRC / 2 */
    CLK_EnableCKO(CLK_CLKSEL2_FDIVSEL_HIRC,0,0);
    printf("Press any key to disable IRC Trim Funciton\n");
    getchar();

    /* Disable IRC Trim */
    SYS_DisableIRCTrim();
    printf("Disable IRC Trim\n");
    while(1);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
