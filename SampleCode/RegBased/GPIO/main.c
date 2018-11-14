/**************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * $Date: 15/07/09 9:49a $
 * @brief    Use GPIO driver to control the GPIO pin direction, control their
 *           high/low state, and how to use GPIO interrupts.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"


/**
 * @brief       Port0/Port1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port0/Port1 default IRQ, declared in startup_Mini55Series.s.
 */
void GPIO01_IRQHandler(void)
{
    uint32_t volatile reg;
    /* To check if P1.5 interrupt occurred */
    if (P1->INTSRC & BIT5)
    {
        P1->INTSRC = BIT5;
        P30 = P30 ^ 1;
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


/**
 * @brief       Port2/Port3/Port4 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port2/Port3/Port4 default IRQ, declared in startup_Mini55Series.s.
 */
void GPIO234_IRQHandler(void)
{
    uint32_t volatile reg;
    /* To check if P2.2 interrupt occurred */
    if (P2->INTSRC & BIT2)
    {
        P2->INTSRC = BIT2;
        P30 = P30 ^ 1;
        printf("P2.2 INT occurred. \n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT2, PORT3 and PORT4 interrupts */
        reg = P2->INTSRC;
        P2->INTSRC = reg;
        reg = P3->INTSRC;
        P3->INTSRC = reg;
        reg = P4->INTSRC;
        P4->INTSRC = reg;
        printf("Un-expected interrupts. \n");
    }
}


/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0(P3.2) default IRQ, declared in startup_Mini51.s.
 */
void EINT0_IRQHandler(void)
{
    /* For P3.2, clear the INT flag */
    P3->INTSRC = BIT2;
    P30 = P30 ^ 1;
    printf("P3.2 EINT0 occurred. \n");
}


/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1(P5.2) default IRQ, declared in startup_Mini55Series.s.
 */
void EINT1_IRQHandler(void)
{
    /* For P5.2, clear the INT flag */
    P5->INTSRC = BIT2;
    P30 = P30 ^ 1;
    printf("P5.2 EINT1 occurred. \n");
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

    /* Read User Config to select internal high speed RC */
    SystemInit();

    /* Enable HIRC, and LIRC (fro WDT) */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk)) !=
            (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk));


    /* Enable UART and WDT clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_WDTCKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

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

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    int32_t i32Err;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|    MINI55 GPIO Driver Sample Code  | \n");
    printf("+-------------------------------------+ \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect P1.0 and P3.4 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure P1.0 as Output mode and P3.4 as Input mode then close it */
    P1->MODE = (P1->MODE & ~0x3) | GPIO_MODE_OUTPUT;
    P3->MODE = (P3->MODE & ~0x300) | (GPIO_MODE_INPUT << 8);

    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    P10 = 0;
    if (P34 != 0)
    {
        i32Err = 1;
    }

    P10 = 1;
    if (P34 != 1)
    {
        i32Err = 1;
    }

    if ( i32Err )
    {
        printf("  [FAIL] --- Please make sure P1.0 and P3.4 are connected. \n");
    }
    else
    {
        printf("  [OK] \n");
    }

    /* Configure P1.0 and P3.4 to default Quasi-bidirectional mode */
    P1->MODE = (P1->MODE & ~0x3) | GPIO_MODE_QUASI;
    P3->MODE = (P3->MODE & ~0x300) | (GPIO_MODE_QUASI << 8);

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("\n  P15, P22, P32(INT0) and P52(INT1) are used to test interrupt\n  and control LEDs(P30)\n");

    /*Configure P30 for LED control */
    P3->MODE = (P3->MODE & ~0x3) | GPIO_MODE_OUTPUT;

    /* Configure P1.5 as Input mode and enable interrupt by rising edge trigger */
    P1->MODE = (P1->MODE & ~0xC00) | (GPIO_MODE_INPUT << 10);
    P1->INTTYPE &= ~0x20;
    P1->INTEN |= 0x200000;
    NVIC_EnableIRQ(GPIO01_IRQn);

    /*  Configure P2.2 as Quasi-bidirectional mode and enable interrupt by falling edge trigger */
    P2->MODE = (P2->MODE & ~0x30) | (GPIO_MODE_QUASI << 4);
    P2->INTTYPE &= ~0x4;
    P2->INTEN |= 0x04;
    NVIC_EnableIRQ(GPIO234_IRQn);

    /* Configure P3.2 as EINT0 pin and enable interrupt by falling edge trigger */
    P3->MODE = (P3->MODE & ~0x30) | (GPIO_MODE_INPUT << 4);
    P3->INTTYPE &= ~0x4;
    P3->INTEN |= 0x04;
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure P5.2 as EINT1 pin and enable interrupt by rising and falling edge trigger */
    P5->MODE = (P5->MODE & ~0x30) | (GPIO_MODE_INPUT << 4);
    P5->INTTYPE &= ~0x4;
    P5->INTEN |= 0x040004;
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO->DBCTL = (GP_DBCTL_ICLKON_Msk | GPIO_DBCTL_DBCLKSRC_HCLK | GPIO_DBCTL_DBCLKSEL_1);
    P1->DBEN |= 0x20;
    P2->DBEN |= 0x04;
    P3->DBEN |= 0x04;
    P5->DBEN |= 0x04;

    /* Waiting for interrupts */
    while (1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


