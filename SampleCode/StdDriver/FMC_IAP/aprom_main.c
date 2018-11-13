/**************************************************************************//**
 * @file     APROM_main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/07/07 5:47p $
 * @brief    This sample code includes LDROM image (fmc_ld_iap)
 *           and APROM image (fmc_ap_main).
 *           It shows how to branch between APROM and LDROM. To run
 *           this sample code, the boot mode must be "Boot from APROM
 *           with IAP".
 *
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"
#include "uart.h"
#include "fmc.h"

typedef void (FUNC_PTR)(void);

extern uint32_t  loaderImage1Base, loaderImage1Limit;

FUNC_PTR    *func;
uint32_t    sp;


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));

    /* Set P5 multi-function pins for crystal output/input */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_XTAL, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}


static int  set_IAP_boot_mode(void)
{
    uint32_t  au32Config[2];

    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");
        return -1;
    }

    if (au32Config[0] & 0x40)
    {
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] &= ~0x40;
        FMC_WriteConfig(au32Config, 2);

        // Perform chip reset to make new User Config take effect
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    }
    return 0;
}


#ifdef __ARMCC_VERSION
__asm __set_SP(uint32_t _sp)
{
    MSR MSP, r0
    BX lr
}
#endif


static int  load_image_to_flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = max_size;

    printf("Program image to flash address 0x%x...", flash_addr);
    pu32Loader = (uint32_t *)image_base;
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(flash_addr + i);
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC_Write(flash_addr + i + j, pu32Loader[(i + j) / 4]);
        }
    }
    printf("OK.\n");

    printf("Verify ...");

    /* Verify loader */
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(flash_addr + i + j);

            if (u32Data != pu32Loader[(i+j)/4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
                return -1;
            }

            if (i + j >= u32ImageSize)
                break;
        }
    }
    printf("OK.\n");
    return 0;
}


int main()
{
    uint8_t     u8Item;
    uint32_t    u32Data;
    FUNC_PTR    *func;

    SYS_Init();
    UART_Init();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|     MINI55 FMC IAP Sample Code         |\n");
    printf("|           [APROM code]                 |\n");
    printf("+----------------------------------------+\n");

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    if (set_IAP_boot_mode() < 0)
    {
        printf("Failed to set IAP boot mode!\n");
        goto lexit;
    }

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if (FMC_GetBootSource() == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }

    u32Data = FMC_ReadCID();
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));

    do
    {
        printf("\n\n\n");
        printf("+----------------------------------------+\n");
        printf("|               Select                   |\n");
        printf("+----------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM             |\n");
        printf("| [1] Run IAP program (in LDROM)         |\n");
        printf("+----------------------------------------+\n");
        printf("Please select...");
        u8Item = getchar();
        printf("%c\n", u8Item);

        switch (u8Item)
        {
        case '0':
            FMC_ENABLE_LD_UPDATE();
            if (load_image_to_flash((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                                    FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
            {
                printf("Load image to LDROM failed!\n");
                goto lexit;
            }
            FMC_DISABLE_LD_UPDATE();
            break;

        case '1':
            func = (FUNC_PTR *)FMC_Read(FMC_LDROM_BASE + 4);
            sp = FMC_Read(FMC_LDROM_BASE);

            printf("\n\nChange VECMAP and branch to LDROM...\n");
            while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk));

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) /* for GNU C compiler */
            asm("msr msp, %0" : : "r" (sp));
#else
            __set_SP(sp);
#endif

            /*  NOTE!
             *     Before change VECMAP, user MUST disable all interrupts.
             *     The following code CANNOT locate in address 0x0 ~ 0x200.
             */

            /* FMC_SetVectorPageAddr(FMC_LDROM_BASE) */
            FMC->ISPCMD = FMC_ISPCMD_VECMAP;
            FMC->ISPADDR = FMC_LDROM_BASE;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

            func();
            break;

        default :
            continue;
        }
    }
    while (1);


lexit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
