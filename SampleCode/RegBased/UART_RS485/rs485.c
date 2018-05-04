/**************************************************************************//**
 * @file     RS485.c
 * @version  V2.00
 * $Date: 15/07/08 4:00p $
 * @brief    Transmit and receive data in UART RS485 mode.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

#define IS_USE_RS485NMM   1      //1:Select NMM_Mode , 0:Select AAD_Mode
#define MATCH_ADDRSS1     0xC0
#define MATCH_ADDRSS2     0xA2
#define UNMATCH_ADDRSS1   0xB1
#define UNMATCH_ADDRSS2   0xD3

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE(void);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void RS485_9bitModeSlave(void);
void RS485_FunctionTest(void);

/*---------------------------------------------------------------------------------------------------------*/
/* RS485 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE()
{
    volatile uint32_t addr=0;
    volatile uint32_t regRX=0xFF;
    volatile uint32_t u32IntSts = UART0->INTSTS;;

    if((u32IntSts & UART_INTSTS_RLSINT_Msk)&&(u32IntSts & UART_INTSTS_RDAINT_Msk))           /* RLS INT & RDA INT */  //For RS485 Detect Address
    {
        if(UART0->FIFOSTS & UART_FIFOSTS_ADDRDETF_Msk)   /* ADD_IF, RS485 mode */
        {
            addr = UART0->DAT;
            UART0->FIFOSTS  = UART_FIFOSTS_ADDRDETF_Msk;    /* clear ADD_IF flag */
            printf("\nAddr=0x%x,Get:",addr);

#if (IS_USE_RS485NMM ==1) //RS485_NMM
            /* if address match, enable RX to receive data, otherwise to disable RX. */
            /* In NMM mode,user can decide multi-address filter. In AAD mode,only one address can set */
            if (( addr == MATCH_ADDRSS1)||( addr == MATCH_ADDRSS2))
            {
                UART0->FIFO &= ~ UART_FIFO_RXOFF_Msk;  /* Enable RS485 RX */
            }
            else
            {
                printf("\n");
                UART0->FIFO |= UART_FIFO_RXOFF_Msk;      /* Disable RS485 RX */
                UART0->FIFO |= UART_FIFO_RXRST_Msk;       /* Clear data from RX FIFO */
            }
#endif
        }
    }
    else if((u32IntSts & UART_INTSTS_RDAINT_Msk) || (u32IntSts & UART_INTSTS_RXTOINT_Msk) )     /* Rx Ready or Time-out INT*/
    {
        /* Handle received data */
        printf("%2d,",UART0->DAT);
    }

    else if(u32IntSts & UART_INTSTS_BUFERRINT_Msk)     /* Buffer Error INT */
    {
        printf("\nBuffer Error...\n");
        UART0->FIFOSTS = (UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control  (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                        */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk));
    while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));  /* Wait Tx empty */

    UART0->LINE = (0x3 | (0x5 << UART_LINE_PBE_Pos) | (0x0 << UART_LINE_NSB_Pos));
    UART0->DAT = u8data;
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;

    UART0->LINE = (0x3 | (0x7 << UART_LINE_PBE_Pos) | (0x0 << UART_LINE_NSB_Pos));
    for (u32Count=0; u32Count != u32WriteBytes; u32Count++)
    {
        while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk));
        while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));  /* Wait Tx empty */

        UART0->DAT = pu8TxBuf[u32Count]; /* Send UART Data from buffer */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeMaster()
{
    int32_t i32;
    uint8_t g_u8SendDataGroup1[10] = {0};
    uint8_t g_u8SendDataGroup2[10] = {0};
    uint8_t g_u8SendDataGroup3[10] = {0};
    uint8_t g_u8SendDataGroup4[10] = {0};

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               RS485 9-bit Master Test                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function will send different address with 10 data bytes|\n");
    printf("| to test RS485 9-bit mode. Please connect TX/RX to another |\n");
    printf("| board and wait its ready to receive.                      |\n");
    printf("| Press any key to start...                                 |\n");
    printf("+-----------------------------------------------------------+\n\n");
    GetChar();

    UART0->MODEM = 0x0;
    /* Set RS485-Master as AUD mode*/
    UART0->FUNCSEL = (0x3 << UART_FUNCSEL_FUNCSEL_Pos);
    UART0->ALTCTL = UART_ALTCTL_RS485AUD_Msk;   /* Enable AUD to HW control RTS pin automatically */
    /* You also can use GPIO to control RTS pin for replacing AUD mode*/
    /* Prepare Data to transmit*/
    for(i32=0; i32<10; i32++)
    {
        g_u8SendDataGroup1[i32] = i32;
        g_u8SendDataGroup2[i32] = i32+10;
        g_u8SendDataGroup3[i32] = i32+20;
        g_u8SendDataGroup4[i32] = i32+30;
    }
    /* Send For different Address and data for test */
    printf("Send Address %x and data 0~9\n",MATCH_ADDRSS1);
    //Send Address Byte
    RS485_SendAddressByte(MATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup1, 10);

    printf("Send Address %x and data 10~19\n",UNMATCH_ADDRSS1);
    RS485_SendAddressByte( UNMATCH_ADDRSS1 );
    RS485_SendDataByte(g_u8SendDataGroup2,10);

    printf("Send Address %x and data 20~29\n",MATCH_ADDRSS2);
    RS485_SendAddressByte( MATCH_ADDRSS2 );
    RS485_SendDataByte(g_u8SendDataGroup3,10);

    printf("Send Address %x and data 30~39\n",UNMATCH_ADDRSS2);
    RS485_SendAddressByte( UNMATCH_ADDRSS2 );
    RS485_SendDataByte(g_u8SendDataGroup4,10);
    printf("Transfer Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Receive Test  (IS_USE_RS485NMM: 0:AAD  1:NMM)                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeSlave()
{

    /* Set RS485 Function */
    UART0->FUNCSEL = (0x3 << UART_FUNCSEL_FUNCSEL_Pos);


    UART0->MODEM = 0x0;
    /* Set Data Format*/ /* Only need parity enable whenever parity ODD/EVEN */
    UART0->LINE = (0x3 | (0x3 << UART_LINE_PBE_Pos) | (0x0 << UART_LINE_NSB_Pos));

    /* Set RX Trigger Level = 1 */
    UART0->FIFO &= ~UART_FIFO_RFITL_Msk;
    UART0->FIFO |= UART_FIFO_RFITL_1BYTE;

#if(IS_USE_RS485NMM == 1)
    printf("+-----------------------------------------------------------+\n");
    printf("|    Normal Multi-drop Operation Mode                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("| Only Address %2x and %2x,data can receive                  |\n",MATCH_ADDRSS1,MATCH_ADDRSS2);
    printf("+-----------------------------------------------------------+\n");

    /* Set RX_DIS enable before set RS485-NMM mode */
    UART0->FIFO |= UART_FIFO_RXOFF_Msk;

    /* Set RS485-NMM Mode */
    UART0->ALTCTL = UART_ALTCTL_RS485NMM_Msk | UART_ALTCTL_RS485AUD_Msk |
                    UART_ALTCTL_ADDRDEN_Msk ;
#else
    printf("Auto Address Match Operation Mode\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("|    Auto Address Match Operation Mode                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|Only Address %2x,data can receive                          |\n",MATCH_ADDRSS1);
    printf("+-----------------------------------------------------------+\n");

    /* Set RS485-AAD Mode and address match is 0xC0 */
    UART->ALTCTL = UART_ALTCTL_RS485AAD_Msk    |
                   UART_ALTCTL_ADDRDEN_Msk |     //ADD_EN is option.
                   UART_ALTCTL_RS485AUD_Msk |
                   ((uint32_t)(MATCH_ADDRSS1)<< UART_ALTCTL_ADDRMV_Pos) ;
#endif

    /* Enable RDA\RLS\Time-out Interrupt  */
    UART0->INTEN |= UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk;
    NVIC_EnableIRQ(UART0_IRQn);

    printf("Ready to receive data...(Press any key to stop test)\n");
    GetChar();

    /* Flush FIFO */
    UART0->FIFO |= UART_FIFO_TXRST_Msk|UART_FIFO_RXRST_Msk;

    UART0->INTEN &= ~(UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk);

    /* Set UART Function */
    UART0->FUNCSEL = (0x0 << UART_FUNCSEL_FUNCSEL_Pos);
    printf("\n\nEnd test\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest()
{
    uint32_t u32Item;
    printf("\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|            IO Setting                                       |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                      _______        |\n");
    printf("| |      |                                    |       |       |\n");
    printf("| |Master|---TXD0        <====> RXD0       ---|Slave  |       |\n");
    printf("| |      |---RTS0        <====> RTS0       ---|       |       |\n");
    printf("| |______|                                    |_______|       |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|       RS485 Function Test                                   |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Please enable semihosted to show messages on debug session.|\n");
    printf("|  Keil users must define DEBUG_ENABLE_SEMIHOST in both C/C++ |\n");
    printf("|  and Asm preprocessor symbols.                              |\n");
    printf("|  IAR users must define DEBUG_ENABLE_SEMIHOST in both C/C++  |\n");
    printf("|  Compiler and Assembler preprocessor symbols.               |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n\n");
    u32Item = GetChar();

    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.
        Master:
            1.Set AUD mode and HW will control RTS pin. LEV_RTS is set to '0'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'. (Set UA_LCR = 0x2B)
            4.Data bytes : the parity bit should be '0'. (Set UA_LCR = 0x3B)
            5.RTS pin is low in idle state. When master is sending,
              RTS pin will be pull high.

        Slave:
            1.Set AAD and AUD mode firstly. LEV_RTS is set to '0'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".  (Default)
            4.AAD: The slave will ignore any data until ADDRESS match ADDR_MATCH value.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check if RS485_ADD_DETF is set and read UA_RBR to clear ADDRESS stored in rx_fifo.

              NMM: The slave will ignore data byte until disable RX_DIS.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check the ADDRESS is match or not by user in UART_IRQHandler.
              If the ADDRESS is match,clear RX_DIS bit to receive data byte.
              If the ADDRESS is not match,set RX_DIS bit to avoid data byte stored in FIFO.
    */

    if(u32Item =='0')
        RS485_9bitModeMaster();
    else
        RS485_9bitModeSlave();
}

