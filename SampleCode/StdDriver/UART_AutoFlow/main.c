/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 7 $
 * $Date: 15/07/08 4:04p $
 * @brief    Transmit and receive data using auto flow control.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

#include "uart.h"


#define RXBUFSIZE 10

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] = {0};
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;
volatile int32_t g_i32pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_TEST_HANDLE(void);
void AutoFlow_FunctionTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Read User Config to select internal high speed RC */
    SystemInit();

    /* Set P5 multi-function pins for crystal output/input */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL &= ~CLK_PWRCTL_XTLEN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_XTLEN_Pos); // XTAL12M (HXT) Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_XTLSTB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_XTAL;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0CKEN_Msk; // UART Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART0SEL_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART0 RXD and TXD  */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_RXD | SYS_MFP_P13_TXD);

    /* Set P0 multi-function pins for UART0 RTS and CTS */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_CTS | SYS_MFP_P01_RTS);

    /* Lock protected registers */
    SYS_LockReg();

}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to Hyper-Terminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART for printf */
    UART_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+------------------------+\n");
    printf("| Auto-Flow function test |\n");
    printf("+------------------------+\n");

    AutoFlow_FunctionTest();

    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
    uint8_t u8InChar=0xFF;
    uint32_t u32IntSts= UART0->INTSTS;

    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        printf("\nInput:");

        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0))
        {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);           /* Rx trigger level is 1 byte*/

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE-1)) ? 0 : (g_u32comRtail+1);
                g_u32comRbytes++;
                g_i32pointer++;
            }
        }
        printf("\nReceive %d bytes data", g_i32pointer);
    }

    if(u32IntSts & UART_INTSTS_THREINT_Msk)
    {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];
            UART_WRITE(UART0,u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
            g_u32comRbytes--;
        }
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest()
{
    uint8_t u8Item;
    uint32_t u32i;
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  _______                                      _______     |\n");
    printf("| |       |                                    |       |    |\n");
    printf("| |Master |---TXD0        <====> RXD0       ---| Slave |    |\n");
    printf("| |       |---RTS0        <====> CTS0       ---|       |    |\n");
    printf("| |_______|---CTS0        <====> RTS0       ---|_______|    |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Set RTS Trigger Level */
    UART_EnableFlowCtrl(UART0);
    UART0->FIFO = (UART0->FIFO &~ UART_FIFO_RTSTRGLV_Msk) | UART_FIFO_RTSTRGLV_14BYTES;

    /* Enable RTS and CTS autoflow control */
    UART0->INTEN |= UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk;

    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave.Slave will check if received data is correct  |\n");
    printf("|    after getting 1k bytes data.                           |\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n\n");
    u8Item = getchar();


    if(u8Item=='0')
    {
        for(u32i=0; u32i<10; u32i++)
        {
            UART_WRITE(UART0,((u32i+(0x30))&0xFF));

            /* Slave will control RTS pin*/
            while(UART0->MODEM & UART_MODEM_RTSSTS_Msk);
        }
        printf("\n Transmit Done\n");
    }
    else
    {
        g_i32pointer = 0;

        /* Enable RDA\RLS\RTO Interrupt  */
        UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));

        /* Set RX Trigger Level = 1 */
        UART0->FIFO = (UART0->FIFO &~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_1BYTE;

        /* Set Timeout time 0x3E bit-time */
        UART_SetTimeoutCnt(UART0,0x3E);

        NVIC_EnableIRQ(UART0_IRQn);

        printf("Starting to receive %d bytes data...\n", RXBUFSIZE);

        while(g_i32pointer<(RXBUFSIZE));

        /* Compare Data */
        for(u32i=0; u32i!=(RXBUFSIZE-1); u32i++)
        {
            if(g_u8RecData[u32i] != ((u32i+(0x30))&0xFF) )
            {
                printf("Compare Data Failed\n");
                while(1);
            }
        }
        printf("\n Receive OK & Check OK\n");
    }

    NVIC_DisableIRQ(UART0_IRQn);

    UART_DISABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
