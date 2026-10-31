/**
 * \author Mr.Nobody
 * \file RamCfg_Types.h
 * \ingroup RamCfg
 * \brief RamCfg module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 * The list of the SRAMs (\ref ramCfg_SramId_t) contains exactly the memories of the
 * device line: an item exists only if the CMSIS device header defines the RAMCFG
 * instance of the memory (STM32H503 has SRAM1, SRAM2 and the backup SRAM, STM32H523 ...
 * STM32H573 add SRAM3, STM32H5E4 ... STM32H5F5 add SRAM3, SRAM4 and SRAM5).
 *
 */

/**
 * \defgroup RamCfg RamCfg
 * \brief RamCfg module - configuration of the internal SRAMs (RAMCFG)
 */

#ifndef RAMCFG_RAMCFG_TYPES_H
#define RAMCFG_RAMCFG_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Stm32.h"                          /* MCU device header (SRAMs)      */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define RAMCFG_NULL_PTR                      ( ( void* ) 0u )

/** Address value used to signal that no failing address is known (no error was recorded) */
#define RAMCFG_ADDRESS_UNUSED                ( 0xFFFFFFFFu )

/** Count of pages used to signal that the write protection is not requested */
#define RAMCFG_PAGE_CNT_UNUSED               ( 0u )

/** Size of one page of the write protection in bytes (1 Kbyte on all STM32H5 lines) */
#define RAMCFG_PAGE_SIZE                     ( 1024u )

/* ========================== EXPORTED MACROS =============================== */

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t ramCfg_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t ramCfg_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t ramCfg_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    ramCfg_MajorVersion_t Major; /**< Major version */
    ramCfg_MinorVersion_t Minor; /**< Minor version */
    ramCfg_PatchVersion_t Patch; /**< Patch version */
}   ramCfg_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    RAMCFG_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    RAMCFG_FUNCTION_ACTIVE         /**< Function status is active   */
}   ramCfg_FunctionState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    RAMCFG_REQUEST_ERROR = 0u, /**< Processing request failed  */
    RAMCFG_REQUEST_OK          /**< Processing request succeed */
}   ramCfg_RequestState_t;


/** Flag states enumeration */
typedef enum
{
    RAMCFG_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    RAMCFG_FLAG_ACTIVE         /**< Active flag state   */
}   ramCfg_FlagState_t;


/**
 * \brief Internal SRAMs of the device line (the list contains the memories of the line only).
 *
 * Features of the memories (STM32H5, reference manuals RM0481, RM0492, RM0517):
 * - SRAM1: software erase. On STM32H503 the reference manual lists the ECC of SRAM1, but the
 *          control and status registers of SRAM1 have no ECC bits (see the README of the module),
 *          the memory is handled as a memory without the RAMCFG control of the ECC.
 * - SRAM2: ECC, write protection in pages, software erase.
 * - SRAM3: ECC (not on STM32H523 and STM32H533), software erase.
 * - SRAM4, SRAM5: software erase (STM32H5E4, STM32H5E5, STM32H5F4, STM32H5F5).
 * - Backup SRAM: ECC, software erase.
 */
typedef enum
{
    RAMCFG_SRAM_1 = 0u,    /**< SRAM1                                                               */
    RAMCFG_SRAM_2,         /**< SRAM2 (ECC, write protection)                                       */
#if defined(RAMCFG_SRAM3)
    RAMCFG_SRAM_3,         /**< SRAM3 (ECC on the lines with the option bit FLASH_OPTSR2_SRAM3_ECC)  */
#endif
#if defined(RAMCFG_SRAM4)
    RAMCFG_SRAM_4,         /**< SRAM4                                                               */
#endif
#if defined(RAMCFG_SRAM5)
    RAMCFG_SRAM_5,         /**< SRAM5                                                               */
#endif
    RAMCFG_SRAM_BACKUP,    /**< Backup SRAM (ECC)                                                   */
    RAMCFG_SRAM_CNT,       /**< Count of the SRAMs of the line                                      */
    RAMCFG_SRAM_UNUSED     /**< No SRAM (no error was recorded)                                     */
}   ramCfg_SramId_t;


/**
 * \brief Check of the content of an SRAM.
 *
 * \note STM32H5 has the ECC only (no parity check on the SRAMs). \ref RAMCFG_CHECK_PARITY stays in the list
 *       for the common interface, the requests of it are refused. \ref RAMCFG_CHECK_UNCHANGED is not in the
 *       specification of the common interface: it keeps the state of the hardware (the ECC enabled by the
 *       option bytes stays enabled), it is the default of the configuration.
 */
typedef enum
{
    RAMCFG_CHECK_UNCHANGED = 0u, /**< The check mode of the hardware is kept (no register is written)         */
    RAMCFG_CHECK_NONE,           /**< No check (the ECC is disabled)                                          */
    RAMCFG_CHECK_PARITY,         /**< Parity check (not available on STM32H5, refused)                         */
    RAMCFG_CHECK_ECC,            /**< ECC - single error correction, double error detection                   */
    RAMCFG_CHECK_CNT             /**< Count of the check modes                                                */
}   ramCfg_CheckMode_t;


/**
 * \brief Reporting of the ECC errors of an SRAM.
 *
 * The failing address is stored by the hardware only while the interrupt of the error is enabled
 * (device errata ES0561 2.2.20), the statistics of the module therefore need a reporting mode
 * other than \ref RAMCFG_REPORT_NONE.
 */
typedef enum
{
    RAMCFG_REPORT_NONE = 0u, /**< No reporting: the flags are set by the hardware, the module does not count them     */
    RAMCFG_REPORT_IRQ,       /**< Single error and double error are reported by the RAMCFG interrupt                  */
    RAMCFG_REPORT_NMI,       /**< Single error by the RAMCFG interrupt, double error by the NMI (cannot be undone
                                  until the reset of the RAMCFG peripheral, the module takes over the NMI vector)     */
    RAMCFG_REPORT_CNT        /**< Count of the reporting modes                                                        */
}   ramCfg_ReportMode_t;


/** \brief State of the software erase of an SRAM */
typedef enum
{
    RAMCFG_ERASE_IDLE = 0u, /**< No erase was requested since the initialization and none is running   */
    RAMCFG_ERASE_BUSY,      /**< An erase is running (the hardware flag SRAMBUSY is set)                */
    RAMCFG_ERASE_DONE,      /**< The requested erase is finished (the content of the SRAM is zero)      */
    RAMCFG_ERASE_CNT        /**< Count of the erase states                                              */
}   ramCfg_EraseState_t;


/** \brief Address in the memory map of the device (address of the failing word, base address of an SRAM) */
typedef uint32_t ramCfg_Address_t;


/** \brief Size of a memory in bytes */
typedef uint32_t ramCfg_Size_t;


/** \brief Index of a write protection page (0 ... count of the pages of the SRAM - 1) */
typedef uint16_t ramCfg_Page_t;


/** \brief Count of the write protection pages */
typedef uint16_t ramCfg_PageCnt_t;


/** \brief Count of the errors */
typedef uint32_t ramCfg_ErrorCnt_t;


/** \brief Count of the wait states of an SRAM */
typedef uint8_t ramCfg_WaitStates_t;


/** \brief Interrupt priority type definition */
typedef uint32_t ramCfg_IrqPrio_t;


/**
 * \brief Single error callback.
 *
 * Called from the interrupt context after the single error was corrected by the hardware, counted
 * and (optionally) scrubbed. The interrupt flag is already acknowledged by the module.
 *
 * \param sramId  [in]: SRAM with the error
 * \param address [in]: Address of the failing word as reported by the hardware
 */
typedef void ( ramCfg_SingleErrorCallback_t )( ramCfg_SramId_t sramId, ramCfg_Address_t address );


/**
 * \brief Double error callback.
 *
 * Called from the interrupt context (or from the NMI in the mode \ref RAMCFG_REPORT_NMI, the callback
 * must be minimal there) after the double error was counted. The data at the address are lost. The flag
 * is already acknowledged by the module.
 *
 * \param sramId  [in]: SRAM with the error
 * \param address [in]: Address of the failing word as reported by the hardware
 */
typedef void ( ramCfg_DoubleErrorCallback_t )( ramCfg_SramId_t sramId, ramCfg_Address_t address );


/**
 * \brief Erase complete callback.
 *
 * Called by \ref RamCfg_Task when the erase started by \ref RamCfg_Set_Erase is finished.
 *
 * \param sramId [in]: SRAM which was erased
 */
typedef void ( ramCfg_EraseCompleteCallback_t )( ramCfg_SramId_t sramId );


/** \brief Statistics of the ECC errors of an SRAM (since the initialization or the last clear) */
typedef struct
{
    ramCfg_ErrorCnt_t SingleErrorCnt;   /**< Count of the corrected single errors                                  */
    ramCfg_ErrorCnt_t DoubleErrorCnt;   /**< Count of the double errors                                            */
    ramCfg_SramId_t   LastErrorSram;    /**< SRAM of the last error (the requested one), \ref RAMCFG_SRAM_UNUSED if
                                             no error was recorded                                                 */
    ramCfg_Address_t  LastErrorAddress; /**< Address of the last error (single or double), \ref RAMCFG_ADDRESS_UNUSED
                                             if no error was recorded                                              */
}   ramCfg_ErrorStatistics_t;


/** \brief Configuration of one SRAM */
typedef struct
{
    /** Check mode, \ref RAMCFG_CHECK_UNCHANGED keeps the state of the hardware. The ECC is not
     *  initialized by the module (see the README): a memory is erased before the first enabling. */
    ramCfg_CheckMode_t     CheckMode;

    /** Reporting of the errors, only \ref RAMCFG_REPORT_NONE is accepted for an SRAM without ECC */
    ramCfg_ReportMode_t    ReportMode;

    /** Scrubbing: the corrected word of a single error is written back (accepted for an SRAM with ECC) */
    ramCfg_FunctionState_t Scrubbing;

    /** First page of the write protection (used when \ref ramCfg_SramConfig_t::ProtectionPageCnt is not zero) */
    ramCfg_Page_t          ProtectionFirstPage;

    /** Count of pages of the write protection starting at the first page, \ref RAMCFG_PAGE_CNT_UNUSED does not
     *  protect any page. The protection is locked until the reset of the RAMCFG peripheral. */
    ramCfg_PageCnt_t       ProtectionPageCnt;
}   ramCfg_SramConfig_t;


/** \brief Type used to store RAMCFG configuration */
typedef struct
{
    /** Configuration of the SRAMs, indexed by \ref ramCfg_SramId_t */
    ramCfg_SramConfig_t             Sram[ RAMCFG_SRAM_CNT ];

    /** Single error callback, \ref RAMCFG_NULL_PTR - the errors are counted only */
    ramCfg_SingleErrorCallback_t   *SingleErrorCallback;

    /** Double error callback, \ref RAMCFG_NULL_PTR - the errors are counted only */
    ramCfg_DoubleErrorCallback_t   *DoubleErrorCallback;

    /** Erase complete callback, \ref RAMCFG_NULL_PTR - \ref RamCfg_Set_Erase waits for the end of the erase */
    ramCfg_EraseCompleteCallback_t *EraseCompleteCallback;

    /** Priority of the RAMCFG interrupt (0 - highest priority) */
    ramCfg_IrqPrio_t                IrqPriority;
}   ramCfg_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */


#endif /* RAMCFG_RAMCFG_TYPES_H */
