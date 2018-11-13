/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 7 $
 * $Date: 15/07/08 3:54p $
 * @brief    Show how to transmit and receive data using auto flow control.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"


#define RXBUFSIZE 1024

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
    SYS->REGLCTL = 0x59;
    SYS->REGLCTL = 0x16;
    SYS->REGLCTL = 0x88;

    /* Read User Config to select internal high speed RC */
    SystemInit();

    /* Set P5 multi-function pins for crystal output/input */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL &= ~CLK_PWRCTL_XTLEN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_XTLEN_Pos); // XTAL12M (HXT) Enabled

    /* Waiting for 12MHz clock ready */
    while(!(CLK->STATUS & CLK_STATUS_XTLSTB_Msk));


    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;

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
    /* Set P1 multi-function pins for UART1 RXD and TXD  */
    SYS->P1_MFP &= ~(0x00000404 | 0x00000808);
    SYS->P1_MFP |= (0x00000400 | 0x00000800);

    /* Set P0 multi-function pins for UART1 RTS and CTS */
    SYS->P0_MFP &= ~(0x01000101 | 0x02000202);
    SYS->P0_MFP |= (0x00000100 | 0x00000200);

    /* Lock protected registers */
    SYS->REGLCTL = 0;

}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk |
                  (((__XTAL + (115200/2)) / 115200)-2);

    UART0->LINE = 0x3 | (0x0 << UART_LINE_PBE_Pos) | (0x0 << UART_LINE_NSB_Pos) ;
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to Hyper Terminal.                                                            */
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

    printf("+-------------------------+\n");
    printf("| Auto-Flow function test  |\n");
    printf("+-------------------------+\n");

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
        while(UART0->INTSTS & UART_INTSTS_RDAIF_Msk)
        {
            /* Get the character from UART Buffer */
            u8InChar = UART0->DAT;

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* En-queue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE-1)) ? 0 : (g_u32comRtail+1);
                g_u32comRbytes++;
            }
        }
        printf("\nReceive Count: %d", g_i32pointer);
    }

    if(u32IntSts & UART_INTSTS_THREINT_Msk)
    {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];
            UART0->DAT = u8InChar;
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
    printf("| |Master |---TXD0(pin46) <====> RXD0(pin45)---| Slave |    |\n");
    printf("| |       |---RTS0(pin37) <====> CTS0(pin38)---|       |    |\n");
    printf("| |_______|---CTS0(pin38) <====> RTS0(pin37)---|_______|    |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Set RTS Trigger Level */
    UART0->MODEM = (UART0->MODEM & ~UART_MODEM_RTSACTLV_Msk     | (0x1 << UART_MODEM_RTSACTLV_Pos)); /* Set RTS is High Level Trigger */
    UART0->FIFO = (UART0->FIFO & ~UART_FIFO_RTSTRGLV_Msk | (0x3 << UART_FIFO_RTSTRGLV_Pos)); /* set RTS Trigger Level to 14 bits */

    /* Enable RTS and CTS auto flow control */
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
        for(u32i=0; u32i<(RXBUFSIZE-1); u32i++)
        {
            UART0->DAT = ((u32i+1)&0xFF);

            /* Slave will control RTS pin*/
            while(UART0->MODEM & UART_MODEM_RTSSTS_Msk);
        }
        printf("\n Transmit Done\n");
    }
    else
    {
        g_i32pointer = 0;

        /* Enable RDA\RLS\RTO Interrupt  */
        UART0->INTEN |= (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);

        /* Set RX Trigger Level = 1 */
        UART0->FIFO = (UART0->FIFO & ~UART_FIFO_RFITL_Msk) | (0x0 << UART_FIFO_RFITL_Pos);

        /* Set Timeout time 0x3E bit-time */
        UART0->TOUT = (UART0->TOUT & ~UART_TOUT_TOIC_Msk) | (0x3E);
        UART0->INTEN |= UART_INTEN_TOCNTEN_Msk;

        NVIC_EnableIRQ(UART0_IRQn);

        printf("Starting to receive %d bytes data...\n", RXBUFSIZE);

        while(g_i32pointer<(RXBUFSIZE-1));

        /* Compare Data */
        for(u32i=0; u32i!=(RXBUFSIZE-1); u32i++)
        {
            if(g_u8RecData[u32i] != ((u32i+1)&0xFF) )
            {
                printf("Compare Data Failed\n");
                while(1);
            }
        }
        printf("\n Receive OK & Check OK\n");
    }

    NVIC_DisableIRQ(UART0_IRQn);

    UART0->INTEN &= ~(UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk);
}
