/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 8 $
 * $Date: 15/07/06 9:48a $
 * @brief    Read/write EEPROM via I2C interface.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini55Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[3];
uint8_t g_u8RxData;
uint8_t g_u8DataLen;
uint8_t __IO g_u8EndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

I2C_FUNC __IO s_I2CHandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C);

    if (I2C_GET_TIMEOUT_FLAG(I2C))
    {
        /* Clear I2C Timeout Flag */
        I2C_ClearTimeoutFlag(I2C);
    }
    else
    {
        if (s_I2CHandlerFn != NULL)
            s_I2CHandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    //printf("RX:%d\n" , u32Status);
    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 2)
        {
            I2C_SET_DATA(I2C, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C, I2C_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C, I2C_STA | I2C_SI);
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        g_u8RxData = I2C_GET_DATA(I2C);
        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
        g_u8EndFlag = 1;
        P07 = 1;

    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    //printf("TX:%d\n" , u32Status);
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        I2C_SET_DATA(I2C, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 3)
        {
            I2C_SET_DATA(I2C, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C, I2C_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
        }

    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Read User Config to select internal high speed RC */
    if (SystemInit() < 0)
        return -1;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC */
    CLK->PWRCTL =  CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2C_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART0SEL_HIRC,CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(I2C_MODULE, 0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_SDA | SYS_MFP_P35_SCL;

    /* I2C pin enable schmitt trigger */
    SYS->P3_MFP |= SYS_MFP_TYPE_Msk(4) | SYS_MFP_TYPE_Msk(5);

    /* Lock protected registers */
    SYS_LockReg();

    /* Update System Core Clock */
    SystemCoreClockUpdate();
    return 0;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

void I2C_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C, 100000);

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C);
    NVIC_EnableIRQ(I2C_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    uint32_t i;
    int32_t retval;

    /* Init System, IP clock and multi-function I/O */
    retval = SYS_Init();

    /* Init UART for printf */
    UART_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    /*
        This sample code sets I2C bus clock to 100kHz. Then, accesses EEPROM 24LC64 with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|    Mini55 I2C Driver Sample Code with EEPROM 24LC64    |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C to access EEPROM */
    I2C_Init();

    g_u8DeviceAddr = 0x50;


    for (i = 0; i < 0x100; i++)
    {
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLen = 0;
        g_u8EndFlag = 0;

        /* I2C function to write data to slave */
        s_I2CHandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C as master sends START signal */
        I2C_START(I2C);

        /* Wait I2C Tx Finish */
        while (g_u8EndFlag == 0);
        while(I2C->CTL & I2C_CTL_STO_Msk);
        g_u8EndFlag = 0;

        /* I2C function to read data from slave */
        s_I2CHandlerFn = (I2C_FUNC)I2C_MasterRx;

        g_u8DataLen = 0;
        g_u8DeviceAddr = 0x50;

        /* Wait write finished */
        CLK_SysTickDelay(5000);

        I2C_START(I2C);

        /* Wait I2C Rx Finish */
        while (g_u8EndFlag == 0);
        while(I2C->CTL & I2C_CTL_STO_Msk);
        /* Compare data */
        if (g_u8RxData != g_au8TxData[2])
        {
            printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
            return -1;
        }
    }

    printf("I2C Access EEPROM Test OK\n");
    while(1);
//    return 0;
}



