/**
 * \author Mr.Nobody
 * \file RamCfg_Port.h
 * \ingroup RamCfg
 * \brief RamCfg module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef RAMCFG_RAMCFG_PORT_H
#define RAMCFG_RAMCFG_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "RamCfg_Types.h"                   /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

ramCfg_ModuleVersion_t      RamCfg_Get_ModuleVersion        ( void );

ramCfg_RequestState_t       RamCfg_Init                     ( const ramCfg_Config_t * const ramCfgConfig );
ramCfg_RequestState_t       RamCfg_Deinit                   ( void );
void                        RamCfg_Task                     ( void );

ramCfg_RequestState_t       RamCfg_Get_DefaultConfig        ( ramCfg_Config_t * const ramCfgConfig );

/* ------------------------------ Check and reporting ----------------------- */

ramCfg_RequestState_t       RamCfg_Set_CheckMode            ( ramCfg_SramId_t sramId, ramCfg_CheckMode_t checkMode );
ramCfg_RequestState_t       RamCfg_Get_CheckMode            ( ramCfg_SramId_t sramId,
                                                              ramCfg_CheckMode_t * const checkMode );

ramCfg_RequestState_t       RamCfg_Set_ReportMode           ( ramCfg_SramId_t sramId, ramCfg_ReportMode_t reportMode );
ramCfg_RequestState_t       RamCfg_Get_ReportMode           ( ramCfg_SramId_t sramId,
                                                              ramCfg_ReportMode_t * const reportMode );

ramCfg_RequestState_t       RamCfg_Set_Scrubbing            ( ramCfg_SramId_t sramId,
                                                              ramCfg_FunctionState_t scrubbing );

/* ---------------------------------- Errors -------------------------------- */

ramCfg_RequestState_t       RamCfg_Get_ErrorStatistics      ( ramCfg_SramId_t sramId,
                                                              ramCfg_ErrorStatistics_t * const statistics );
ramCfg_RequestState_t       RamCfg_Set_ErrorStatisticsClear ( ramCfg_SramId_t sramId );
ramCfg_RequestState_t       RamCfg_Get_FailingAddress       ( ramCfg_SramId_t sramId,
                                                              ramCfg_Address_t * const failingAddress );

/* ---------------------------- Protection and erase ------------------------ */

ramCfg_RequestState_t       RamCfg_Set_WriteProtection      ( ramCfg_SramId_t sramId,
                                                              ramCfg_Page_t firstPage,
                                                              ramCfg_PageCnt_t pageCnt );
ramCfg_RequestState_t       RamCfg_Get_WriteProtection      ( ramCfg_SramId_t sramId,
                                                              ramCfg_Page_t firstPage,
                                                              ramCfg_PageCnt_t pageCnt,
                                                              ramCfg_FunctionState_t * const protection );

ramCfg_RequestState_t       RamCfg_Set_Erase                ( ramCfg_SramId_t sramId );
ramCfg_RequestState_t       RamCfg_Get_EraseState           ( ramCfg_SramId_t sramId,
                                                              ramCfg_EraseState_t * const eraseState );

/* ---------------------------------- Wait states --------------------------- */

ramCfg_RequestState_t       RamCfg_Set_WaitStates           ( ramCfg_SramId_t sramId, ramCfg_WaitStates_t waitStates );
ramCfg_RequestState_t       RamCfg_Get_WaitStates           ( ramCfg_SramId_t sramId,
                                                              ramCfg_WaitStates_t * const waitStates );

/* -------------------------------- Interrupt ------------------------------- */

ramCfg_RequestState_t       RamCfg_Set_IrqPriority          ( ramCfg_IrqPrio_t irqPrio );
ramCfg_RequestState_t       RamCfg_Get_IrqPriority          ( ramCfg_IrqPrio_t * const irqPrio );

#ifdef __cplusplus
}
#endif

#endif /* RAMCFG_RAMCFG_PORT_H */
