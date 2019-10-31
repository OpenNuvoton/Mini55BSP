/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __TARGET_H__
#define __TARGET_H__

#include "Mini55Series.h"
#include "ISP_USER.h"
#include "uart_transfer.h"

/* rename for uart_transfer.c */
#define UART_N                          UART0
#define UART_N_IRQHandler       UART0_IRQHandler
#define UART_N_IRQn                 UART0_IRQn

#endif //__TARGET_H__
