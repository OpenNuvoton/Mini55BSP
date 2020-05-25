/**************************************************************************//**
 * @file     FMC.h
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/07/09 9:01a $
 * @brief    MINI55 series FMC driver header file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __FMC_H__
#define __FMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup MINI55_Device_Driver MINI55 Device Driver
  @{
*/

/** @addtogroup MINI55_FMC_Driver FMC Driver
  @{
*/

/** @addtogroup MINI55_FMC_EXPORTED_CONSTANTS FMC Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00000000UL    /*!< APROM Base Address          */
#define FMC_APROM_END           0x00004600UL    /*!< APROM End Address           */
#define FMC_LDROM_BASE          0x00100000UL    /*!< LDROM Base Address          */
#define FMC_LDROM_END           0x00100800UL    /*!< LDROM End Address           */
#define FMC_CONFIG_BASE         0x00300000UL    /*!< CONFIG Base Address         */

#define FMC_FLASH_PAGE_SIZE     0x200           /*!< Flash Page Size (512 Bytes) */
#define FMC_LDROM_SIZE          0x800           /*!< LDROM Size (2 Kbytes)       */


/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00            /*!< ISP Command: Read Flash        */
#define FMC_ISPCMD_PROGRAM      0x21            /*!< ISP Command: Program Flash     */
#define FMC_ISPCMD_PAGE_ERASE   0x22            /*!< ISP Command: Page Erase Flash  */
#define FMC_ISPCMD_READ_CID     0x0B            /*!< ISP Command: Read Company ID   */
#define FMC_ISPCMD_READ_PID     0x0C            /*!< ISP Command: Read Product ID   */
#define FMC_ISPCMD_READ_UID     0x04            /*!< ISP Command: Read Unique ID    */
#define FMC_ISPCMD_VECMAP       0x2E            /*!< ISP Command: Vector Page Remap */

#define IS_BOOT_FROM_APROM      0               /*!< Is booting from APROM                */
#define IS_BOOT_FROM_LDROM      1               /*!< Is booting from LDROM                */


/*@}*/ /* end of group MINI55_FMC_EXPORTED_CONSTANTS */

/** @addtogroup MINI55_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

#define FMC_SET_APROM_BOOT()        (FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk)         /*!< Select booting from APROM  */
#define FMC_SET_LDROM_BOOT()        (FMC->ISPCTL |= FMC_ISPCTL_BS_Msk)          /*!< Select booting from LDROM  */
#define FMC_ENABLE_AP_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk)      /*!< Enable APROM update  */
#define FMC_DISABLE_AP_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk)      /*!< Disable APROM update  */
#define FMC_ENABLE_LD_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)      /*!< Enable LDROM update  */
#define FMC_DISABLE_LD_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)      /*!< Disable LDROM update  */
#define FMC_ENABLE_CFG_UPDATE()     (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk)     /*!< Enable User Config update  */
#define FMC_DISABLE_CFG_UPDATE()    (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk)     /*!< Disable User Config update  */
#define FMC_ENABLE_ISP()            (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)      /*!< Enable ISP function  */
#define FMC_DISABLE_ISP()           (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)      /*!< Disable ISP function  */
#define FMC_GET_FAIL_FLAG()         ((FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) ? 1 : 0)  /*!< Get ISP fail flag  */
#define FMC_CLR_FAIL_FLAG()         (FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk)       /*!< Clear ISP fail flag        */

extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadPID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern uint32_t FMC_GetVectorPageAddr(void);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);


/*@}*/ /* end of group MINI55_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI55_FMC_Driver */

/*@}*/ /* end of group MINI55_Device_Driver */

#ifdef __cplusplus
}
#endif


#endif

