/**
 * \author Mr.Nobody
 * \file RamCfg.h
 * \ingroup RamCfg
 * \brief RamCfg module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef RAMCFG_RAMCFG_H
#define RAMCFG_RAMCFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "RamCfg_Types.h"          /* Module types definitions       */
#include "RamCfg_Reg.h"            /* Register identifiers           */
#include "Rcc_Types.h"             /* Clock identification           */
/* ============================= TYPEDEFS =================================== */

/** \brief Features of an SRAM of the device line (one item of the table of the SRAMs) */
typedef struct
{
    ramCfg_SramId_t        SramId;    /**< SRAM (position of the item in the table)                              */
    ramCfg_Address_t       Base;      /**< Base address of the memory (non-secure alias)                         */
    ramCfg_Size_t          Size;      /**< Size of the memory in bytes                                           */
    ramCfg_FunctionState_t EccState;  /**< The ECC is controlled and reported by the RAMCFG registers            */
    ramCfg_PageCnt_t       WpPageCnt; /**< Count of the pages of the write protection, zero - no write protection */
    ramCfg_FunctionState_t ClkState;  /**< The clock of the memory has its own enable bit in the Rcc module      */
    rcc_PeriphId_t         ClkPeriph; /**< Clock identification of the memory in the Rcc module (not used when
                                           \ref ramCfg_SramInfo_t::ClkState is inactive)                          */
}   ramCfg_SramInfo_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

#ifdef __cplusplus
}
#endif

#endif /* RAMCFG_RAMCFG_H */
