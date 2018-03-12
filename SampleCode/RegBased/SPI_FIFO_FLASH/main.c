/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/06/29 11:16a $
 * @brief    Access SPI Flash using FIFO mode.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

#define TEST_NUMBER 1   /* page numbers */
#define TEST_LENGTH 256 /* length */

#define SPI_FLASH_PORT  SPI0
uint8_t SrcArray[TEST_LENGTH];
uint8_t DestArray[TEST_LENGTH];

uint16_t SpiFlash_ReadMidDid(void)
{
    uint8_t u8TxData[6]= {0x90, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t u8RxData[6]= {0};
    uint8_t u8IDCnt = 0, u8TxCnt = 0;

    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0x90, Read Manufacturer/Device ID
    while(u8TxCnt < 6)
    {
        if((SPI_FLASH_PORT->STATUS & SPI_STATUS_TXFULL_Msk) != SPI_STATUS_TXFULL_Msk)
        {
            SPI_FLASH_PORT->TX = u8TxData[u8TxCnt ++];
            while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);
        }

        while(((SPI_FLASH_PORT->STATUS & SPI_STATUS_RXEMPTY_Msk) != SPI_STATUS_RXEMPTY_Msk) && (u8IDCnt < 6))
            u8RxData[u8IDCnt ++] = SPI_FLASH_PORT->RX;
    }


    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;

    return ( (u8RxData[4]<<8) | u8RxData[5] );
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0x06, Write enable
    SPI_FLASH_PORT->TX = 0x06;

    // wait tx finish
    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);

    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;

    //////////////////////////////////////////

    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0xC7, Chip Erase
    SPI_FLASH_PORT->TX = 0xC7;

    // wait tx finish
    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);

    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;

    SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk;
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    volatile uint32_t data;

    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0x05, Read status register
    SPI_FLASH_PORT->TX = 0x05;

    // read status
    SPI_FLASH_PORT->TX = 0x00;

    // wait tx finish
    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);

    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;

    // skip first rx data
    data = SPI_FLASH_PORT->RX;
    data = SPI_FLASH_PORT->RX;

    return (data & 0xff);
}

void SpiFlash_WriteStatusReg(uint8_t u8Value)
{
    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0x06, Write enable
    SPI_FLASH_PORT->TX = 0x06;

    // wait tx finish
    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);

    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;

    ///////////////////////////////////////

    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0x01, Write status register
    SPI_FLASH_PORT->TX = 0x01;

    // write status
    SPI_FLASH_PORT->TX = u8Value;

    // wait tx finish
    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);

    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;
}

void SpiFlash_WaitReady(void)
{
    uint8_t ReturnValue;

    do
    {
        ReturnValue = SpiFlash_ReadStatusReg();
        ReturnValue = ReturnValue & 1;
    }
    while(ReturnValue!=0);   // check the BUSY bit
}

void SpiFlash_NormalPageProgram(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t i = 0;

    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0x06, Write enable
    SPI_FLASH_PORT->TX = 0x06;

    // wait tx finish
    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);

    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;


    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0x02, Page program
    SPI_FLASH_PORT->TX = 0x02;

    // send 24-bit start address
    SPI_FLASH_PORT->TX = (StartAddress>>16) & 0xFF;
    SPI_FLASH_PORT->TX = (StartAddress>>8)  & 0xFF;
    SPI_FLASH_PORT->TX = StartAddress       & 0xFF;

    // write data
    while(1)
    {
        if((SPI_FLASH_PORT->STATUS & SPI_STATUS_TXFULL_Msk) != SPI_STATUS_TXFULL_Msk)
        {
            SPI_FLASH_PORT->TX = u8DataBuffer[i++];
            while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);
            if(i >= 255) break;
        }
    }

    // wait tx finish
    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);

    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;

    SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk;
}

void SpiFlash_NormalRead(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t i;

    // /CS: active
    SPI_FLASH_PORT->SSCTL |= SPI_SSCTL_SS_Msk;

    // send Command: 0x03, Read data
    SPI_FLASH_PORT->TX = 0x03;

    // send 24-bit start address
    SPI_FLASH_PORT->TX = (StartAddress>>16) & 0xFF;
    SPI_FLASH_PORT->TX = (StartAddress>>8)  & 0xFF;
    SPI_FLASH_PORT->TX = StartAddress       & 0xFF;

    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);
    // clear RX buffer
    SPI_FLASH_PORT->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk;

    // read data
    for(i=0; i<256; i++)
    {
        SPI_FLASH_PORT->TX = 0x00;
        while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);
        u8DataBuffer[i] = SPI_FLASH_PORT->RX;
    }

    // wait tx finish
    while((SPI_FLASH_PORT->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk);

    // /CS: de-active
    SPI_FLASH_PORT->SSCTL &= ~SPI_SSCTL_SS_Msk;
}

void SYS_Init(void)
{
    int32_t i32TimeOutCnt;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->REGLCTL != SYS_REGLCTL_REGWRPROT_Msk)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Read User Config to select internal high speed RC */
    SystemInit();

    /* Enable HIRC */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    i32TimeOutCnt = __HSI / 200; /* About 5ms */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
            break;
    }

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;
    CLK->CLKSEL1 |= (0x2 << CLK_CLKSEL1_UART0SEL_Pos);// Clock source from HIRC clock

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP = SYS_MFP_P01_RXD | SYS_MFP_P00_TXD;

    /* Setup SPI multi-function pin */
    SYS->P0_MFP |= SYS_MFP_P04_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK;

    /* Lock protected registers */
    SYS->REGLCTL = 0;
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk |
                  (((__HSI + (115200/2)) / 115200)-2);

    UART0->LINE = 0x3 | (0x0 << UART_LINE_PBE_Pos) | (0x0 << UART_LINE_NSB_Pos) ;
}


void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, falling clock edge Tx, rising edge Rx and 8-bit transaction */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    CLK->APBCLK |= CLK_APBCLK_SPICKEN_Msk;
    SPI->CTL = SPI_MASTER | SPI_MODE_0 | (8 << SPI_CTL_DWIDTH_Pos);
    SPI->CLKDIV = (((__HSI / 1000000) + 1) >> 1) - 1;

    /* Select the SS pin */
    SPI->SSCTL |= SPI_SS;

    /* Enable FIFO mode and configure threshold level to 2. */
    SPI->FIFOCTL = (SPI->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk) |
                    (2 << SPI_FIFOCTL_TXTH_Pos) |
                    (2 << SPI_FIFOCTL_RXTH_Pos));

    SPI->CTL |= SPI_CTL_FIFOEN_Msk;

    /* SS active low */
    SPI->SSCTL &= ~SPI_SSCTL_SSACTPOL_Msk;
}

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t nError = 0;
    uint16_t u16ID;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n+------------------------------------------------------------+\n");
    printf("|                SPI Flash with FIFO mode Sample             |\n");
    printf("+------------------------------------------------------------+\n");

    /* Wait ready */
    SpiFlash_WaitReady();

    if((u16ID = SpiFlash_ReadMidDid()) != 0xEF14)
    {
        printf("Wrong ID, 0x%x\n", u16ID);
        while(1);
    }
    else
        printf("Flash found: W25X16 ...\n");

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase();

    /* Wait ready */
    SpiFlash_WaitReady();

    printf("[OK]\n");

    /* init source data buffer */
    for(u32ByteCount=0; u32ByteCount<TEST_LENGTH; u32ByteCount++)
    {
        SrcArray[u32ByteCount] = u32ByteCount;
    }

    printf("Start to write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber=0; u32PageNumber<TEST_NUMBER; u32PageNumber++)
    {
        /* page program */
        SpiFlash_NormalPageProgram(u32FlashAddress, SrcArray);
        SpiFlash_WaitReady();
        u32FlashAddress += 0x100;
    }

    printf("[OK]\n");

    /* clear destination data buffer */
    for(u32ByteCount=0; u32ByteCount<TEST_LENGTH; u32ByteCount++)
    {
        DestArray[u32ByteCount] = 0;
    }

    printf("Read & Compare ...");

    /* Read SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber=0; u32PageNumber<TEST_NUMBER; u32PageNumber++)
    {
        /* page read */
        SpiFlash_NormalRead(u32FlashAddress, DestArray);
        u32FlashAddress += 0x100;

        for(u32ByteCount=0; u32ByteCount<TEST_LENGTH; u32ByteCount++)
        {
            if(DestArray[u32ByteCount] != SrcArray[u32ByteCount])
                nError ++;
        }
    }

    if(nError == 0)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

    while(1);
}


/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


