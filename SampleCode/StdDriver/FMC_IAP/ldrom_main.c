/**************************************************************************//**
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/07/09 8:59a $
 * @brief    This sample code includes LDROM image (fmc_ld_iap)
 *           and APROM image (fmc_ap_main).
 *           It shows how to branch between APROM and LDROM. To run
 *           this sample code, the boot mode must be "Boot from APROM
 *           with IAP".
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"
#include "uart.h"
#include "fmc.h"

typedef void (FUNC_PTR)(void);

FUNC_PTR    *func;
uint32_t    sp;

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*  Read User Config to select internal high speed RC  */
    SystemInit();

    /* Set P5 multi-function pins for crystal output/input */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC and XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK->PWRCTL = (CLK->PWRCTL & ~CLK_PWRCTL_XTLEN_Msk) | CLK_PWRCTL_HXT;

    /* Waiting for clock ready */
    while ((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_XTLSTB_Msk)) !=
            (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_XTLSTB_Msk)) ;

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_XTAL;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init()
{
    SYS_ResetModule(UART0_RST);
    UART0->BAUD = 0x30000066;
    UART0->LINE = 0x3;
}

void print_char(int ch)
{
    while(UART->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
    UART->DAT = ch;
    if(ch == '\n')
    {
        while(UART->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        UART->DAT = '\r';
    }
}

void print_msg(char *str)
{
    for ( ; *str ; str++)
        print_char(*str);
}


#ifdef __ARMCC_VERSION
__asm __set_SP(uint32_t _sp)
{
    MSR MSP, r0
    BX lr
}
#endif

int main()
{
    FUNC_PTR    *func;

    SYS_Init();
    UART_Init();

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    print_msg("\n\n");
    print_msg("MINI55 FMC IAP Sample Code [LDROM code]\n");

    print_msg("\n\nPress any key to branch to APROM...\n");
    while ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk));

    print_msg("\n\nChange VECMAP and branch to APROM...\n");
    while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk));

    sp = FMC_Read(FMC_APROM_BASE);
    func = (FUNC_PTR *)FMC_Read(FMC_APROM_BASE + 4);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) /* for GNU C compiler */
    asm("msr msp, %0" : : "r" (sp));
#else
    __set_SP(sp);
#endif

    /*  NOTE!
     *     Before change VECMAP, user MUST disable all interrupts.
     */
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    func();

    while (1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
