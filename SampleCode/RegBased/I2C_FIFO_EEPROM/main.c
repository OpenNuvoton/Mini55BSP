/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 2 $
 * $Date: 15/06/29 11:16a $
 * @brief    Read/write EEPROM via I2C interface using FIFO mode.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Mini55Series.h"

#define EEPROM_READ_ADDR      0xA1 /* Address of slave for read  */
#define EEPROM_WRITE_ADDR     0xA0 /* Address of slave for write */
uint8_t WBuf[3], RBuf[3];

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
    CLK->APBCLK = CLK_APBCLK_I2CCKEN_Msk | CLK_APBCLK_UART0CKEN_Msk;

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

    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_SDA | SYS_MFP_P35_SCL;

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

void I2C_Init(void)
{
    /* Reset I2C */
    SYS->IPRST1 |=  SYS_IPRST1_I2C_RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C_RST_Msk;

    /* Enable I2C Controller */
    I2C->CTL |= I2C_CTL_I2CEN_Msk;

    /* I2C clock divider, I2C Bus Clock = 100kHz */
    I2C->CLKDIV = 0x1D;

    /* Set I2C 4 Slave Addresses */
    I2C->ADDR0 = (0x15 << 1);
    I2C->ADDR1 = (0x35 << 1);
    I2C->ADDR2 = (0x55 << 1);
    I2C->ADDR3 = (0x75 << 1);
}


void ACK_Polling(void)
{
    uint32_t u32Status;

    /* Disable FIFO mode , don't need FIFO here */
    I2C->CTL1 &= ~I2C_CTL1_FIFOEN_Msk;

    do
    {
        /* Send start */
        I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;   // S
        while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), S

        /* Send control byte */
        I2C->DAT = EEPROM_WRITE_ADDR;                                // ConByte(W)
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), ConByte(W)
        u32Status = I2C->STATUS;
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;            // STOP
    }
    while( u32Status!= 0x18);

    /* Enable FIFO mode again */
    I2C->CTL1 |= I2C_CTL1_FIFOEN_Msk;
}

void EEPROM_Write(void)
{
    /* Send start */
    I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;   // S
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), S

    /* Send control byte */
    I2C->DAT = EEPROM_WRITE_ADDR;                       // (DATA), ConByte(W)
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    I2C->DAT = (0x00 >> 8) & 0xFFUL;                    // (DATA), Add-H
    while(!(I2C->CTL & I2C_CTL_SI_Msk));               // (INT), ConByte(W)

    I2C->DAT = 0x01 & 0xFFUL;                           // (DATA), Add-L
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));               // (INT), Add-H

    I2C->DAT = WBuf[0];                                 // (DATA), data0
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));               // (INT), Add-L

    I2C->DAT = WBuf[1];                                 // (DATA), data1
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));               // (INT), data0

    I2C->DAT = WBuf[2];                                 // (DATA), data2
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));               // (INT), data1

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;   // STOP
    while(!(I2C->CTL & I2C_CTL_SI_Msk));               // (INT), data2

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
}

void EEPROM_Read(void)
{
    /* Send start */
    I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;   // S
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), S

    /* Send control byte */
    I2C->DAT = EEPROM_WRITE_ADDR;                               // (DATA), ControlByte-Write
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    I2C->DAT = (0x00 >> 8) & 0xFFUL;                            // (DATA), Add-H
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), Con-W

    I2C->DAT = 0x01 & 0xFFUL;                                   // (DATA), Add-L
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), Add-H

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STA | I2C_SI;           // Sr
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), ADD-L

    I2C->DAT = EEPROM_READ_ADDR;                                // (DATA), ControlByte-Read
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), Sr

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), ConrtolByte-Read

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), data0
    RBuf[0] = I2C->DAT;                                         // (DATA), data0

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), data1
    RBuf[1] = I2C->DAT;                                         // (DATA), data1

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;            // STOP
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), data2
    RBuf[2] = I2C->DAT;                                         // (DATA), data2

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
}

int main(void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("+------------------------------------------------+\n");
    printf("|      I2C FIFO Sample Code with EEPROM 24LC64   |\n");
    printf("+------------------------------------------------+\n");

    /* Init I2C to access EEPROM */
    I2C_Init();

    /* Enable FIFO mode */
    I2C->CTL1 |= I2C_CTL1_FIFOEN_Msk;

    /* Write data to EEPROM */
    EEPROM_Write();

    /* Polling ACK from EEPROM */
    ACK_Polling();

    /* Read data from EEPROM*/
    EEPROM_Read();

    /* Check receive buffer */
    for(i=0; i<3; i++)
    {
        if(WBuf[i] != RBuf[i])
            printf("Data-%d Error!\n", i);
        else
            printf("Data-%d OK!\n", i);
    }

    while(1);
}
