/**
 * \author Mr.Nobody
 * \file RamCfg_Reg.h
 * \ingroup RamCfg
 * \brief RamCfg module Reg component header file.
 *
 * This component contains functionality for register operations. It is private
 * to the module (the register access is not shared with other modules). Every
 * SRAM has its own set of RAMCFG registers (instance), the accessors take the
 * SRAM identification as the instance.
 *
 */

#ifndef RAMCFG_REG_RAMCFG_REG_H
#define RAMCFG_REG_RAMCFG_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "RamCfg_Types.h"                   /* Module types definition        */
#include "Stm32.h"                          /* MCU device header (SRAM2 size) */
/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Count of the write protection pages in one write protection register */
#define RAMCFG_REG_PAGES_PER_WPR            ( 32u )

/** Count of the write protection pages of SRAM2 of the device line (1 Kbyte pages, size of SRAM2 from the
 *  device header: 16 on STM32H503, 80 on STM32H523 ... STM32H553, 64 on STM32H562, STM32H563 and STM32H573,
 *  128 on STM32H5E4 ... STM32H5F5) */
#define RAMCFG_REG_WP_PAGE_CNT              ( SRAM2_SIZE / RAMCFG_PAGE_SIZE )

/** Count of the write protection registers of the device line (RAMCFG_WPR1 ... RAMCFG_WPR4) */
#define RAMCFG_REG_WPR_CNT                  ( ( RAMCFG_REG_WP_PAGE_CNT + RAMCFG_REG_PAGES_PER_WPR - 1u ) / \
                                              RAMCFG_REG_PAGES_PER_WPR )

/* ============================= TYPEDEFS =================================== */

/** \brief List of RAMCFG registers of one SRAM (the write protection registers exist for SRAM2 only, the
 *         ECC registers for the SRAMs with ECC only) */
typedef enum
{
    RAMCFG_REG_CR  = 0u, /**< Control register                      */
    RAMCFG_REG_IER     , /**< Interrupt enable register             */
    RAMCFG_REG_ISR     , /**< Interrupt status register             */
    RAMCFG_REG_SEAR    , /**< ECC single error address register     */
    RAMCFG_REG_DEAR    , /**< ECC double error address register     */
    RAMCFG_REG_ICR     , /**< Interrupt clear register              */
    RAMCFG_REG_WPR1    , /**< Write protection register 1 (pages 0 - 31)                  */
#if ( RAMCFG_REG_WPR_CNT > 1u )
    RAMCFG_REG_WPR2    , /**< Write protection register 2 (pages 32 - 63)                 */
#endif
#if ( RAMCFG_REG_WPR_CNT > 2u )
    RAMCFG_REG_WPR3    , /**< Write protection register 3 (pages 64 - 95)                 */
#endif
#if ( RAMCFG_REG_WPR_CNT > 3u )
    RAMCFG_REG_WPR4    , /**< Write protection register 4 (pages 96 - 127)                */
#endif
    RAMCFG_REG_ECCKEY  , /**< ECC key register                      */
    RAMCFG_REG_ERKEYR  , /**< Erase key register                    */
    RAMCFG_REG_CNT       /**< Count of available RAMCFG registers   */
}   ramCfg_RegId_t;

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                        RamCfg_Set_RegBit               ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t bitMask );
void                        RamCfg_Reset_RegBit             ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t bitMask );
uint32_t                    RamCfg_Get_RegBit               ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t bitMask );

void                        RamCfg_Set_RegVal               ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t regMask,
                                                              uint32_t regValue );
uint32_t                    RamCfg_Get_RegVal               ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t regMask );

void                        RamCfg_Set_RegRaw               ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t regValue );

/* ----------------------------- Register protocol -------------------------- */

ramCfg_RequestState_t       RamCfg_Reg_Set_Ecc              ( ramCfg_SramId_t sramId, ramCfg_FunctionState_t eccState );
ramCfg_FunctionState_t      RamCfg_Reg_Get_Ecc              ( ramCfg_SramId_t sramId );

ramCfg_RequestState_t       RamCfg_Reg_Set_Reporting        ( ramCfg_SramId_t sramId, ramCfg_ReportMode_t reportMode );
ramCfg_RequestState_t       RamCfg_Reg_Set_ReportingOff     ( ramCfg_SramId_t sramId );
ramCfg_RequestState_t       RamCfg_Reg_Get_Reporting        ( ramCfg_SramId_t sramId,
                                                              ramCfg_ReportMode_t * const reportMode );

ramCfg_RequestState_t       RamCfg_Reg_Set_WriteProtection  ( ramCfg_SramId_t sramId,
                                                              ramCfg_Page_t firstPage,
                                                              ramCfg_PageCnt_t pageCnt );
ramCfg_RequestState_t       RamCfg_Reg_Get_WriteProtection  ( ramCfg_SramId_t sramId,
                                                              ramCfg_Page_t firstPage,
                                                              ramCfg_PageCnt_t pageCnt,
                                                              ramCfg_FunctionState_t * const protection );

ramCfg_RequestState_t       RamCfg_Reg_Set_EraseStart       ( ramCfg_SramId_t sramId );
ramCfg_FlagState_t          RamCfg_Reg_Get_EraseBusy        ( ramCfg_SramId_t sramId );
ramCfg_RequestState_t       RamCfg_Reg_Wait_EraseEnd        ( ramCfg_SramId_t sramId );

ramCfg_FlagState_t          RamCfg_Reg_Get_SingleFlag       ( ramCfg_SramId_t sramId );
ramCfg_FlagState_t          RamCfg_Reg_Get_DoubleFlag       ( ramCfg_SramId_t sramId );
void                        RamCfg_Reg_Clear_SingleFlag     ( ramCfg_SramId_t sramId );
void                        RamCfg_Reg_Clear_DoubleFlag     ( ramCfg_SramId_t sramId );
ramCfg_Address_t            RamCfg_Reg_Get_SingleAddress    ( ramCfg_SramId_t sramId );
ramCfg_Address_t            RamCfg_Reg_Get_DoubleAddress    ( ramCfg_SramId_t sramId );

#ifdef __cplusplus
}
#endif

#endif /* RAMCFG_REG_RAMCFG_REG_H */
