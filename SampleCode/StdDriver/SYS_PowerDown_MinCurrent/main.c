/**************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @note
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini55series.h"
#include "GPIO.h"

#define GPIO_P0_TO_P15              0xFFFF

/**
 * @brief       Port0/Port1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port0/Port1 default IRQ, declared in startup_Mini55.s.
 */
void GPIO01_IRQHandler(void)
{
    uint32_t reg;
    /* To check if P1.5 interrupt occurred */
    if (P1->INTSRC & BIT5)
    {
        P1->INTSRC = BIT5;
        printf("P1.5 INT occurred. \n");

    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT0, PORT1 interrupts */
        reg = P0->INTSRC;
        P0->INTSRC = reg;
        reg = P1->INTSRC;
        P1->INTSRC = reg;
        printf("Un-expected interrupts. \n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

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
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

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

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(SYS_IPRST1_UART0RST_Msk);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by P1.5 Sample Code   |\n");
    printf("+-------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------------+\n");
    printf("+ Operating sequence                                                      |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                               |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                      |\n");
    printf("|  3. Enter to Power-Down                                                 |\n");
    printf("|  4. Wait for P1.5 rising-edge interrupt event to wakeup the MCU         |\n");
    printf("+-------------------------------------------------------------------------+\n\n");


    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Configure all GPIO as Quasi-bidirectional Mode*/
    GPIO_SetMode(P0, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(P1, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(P2, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(P3, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(P4, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(P5, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Configure P1.5 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(P1, BIT5, GPIO_MODE_INPUT);
    GPIO_EnableInt(P1, 5, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPIO01_IRQn);;

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK, GPIO_DBCTL_DBCLKSEL_1);
    GPIO_ENABLE_DEBOUNCE(P1, BIT5);

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    printf("Enter to Power-Down ......\n");
    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Waiting for P1.5 rising-edge interrupt event */
    printf("System waken-up done.\n\n");

    while(1);
}

