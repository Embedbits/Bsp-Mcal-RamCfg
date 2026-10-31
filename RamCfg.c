/**
 * \author Mr.Nobody
 * \file RamCfg.c
 * \ingroup RamCfg
 * \brief RamCfg module common functionality
 *
 * # Introduction #
 * The SRAM controller (RAMCFG) configures the features of the internal SRAMs of STM32H5: the
 * error correcting code (ECC: one bit error corrected, two bit error detected), the write protection
 * in pages of 1 Kbyte (SRAM2) and the software erase. Every SRAM has its own set of registers. The
 * module owns the controller, the other modules and the application do not touch the registers.
 *
 * # Features of the SRAMs #
 * The memories, their sizes and the features are a property of the device line (table
 * ramCfg_SramInfo, the list \ref ramCfg_SramId_t contains the memories of the line only):
 * SRAM2 and the backup SRAM have the ECC, SRAM3 has the ECC on the lines with the option bit
 * SRAM3_ECC (not on STM32H523 and STM32H533), SRAM1, SRAM4 and SRAM5 have none. The write protection
 * exists for SRAM2 (16 pages on STM32H503, 80 on STM32H523 ... STM32H553, 64 on STM32H562, STM32H563 and
 * STM32H573, 128 on STM32H5E4 ... STM32H5F5). All SRAMs can be erased. A request for a feature the
 * SRAM does not have returns \ref RAMCFG_REQUEST_ERROR, no register is written.
 *
 * # ECC errors #
 * The hardware corrects a single error on read and flags it (SEDC), a double error is flagged (DED).
 * The failing address is stored only while the address latch and the interrupt of the error are
 * enabled. The module therefore enables both when an SRAM is reported (modes IRQ and NMI). The interrupt
 * handler counts the errors, remembers the address, optionally scrubs the word (reads the corrected value
 * and writes it back), acknowledges the flag and calls the callback. In the mode NMI the double error is
 * reported by the NMI: the module takes over the NMI vector (the previous handler is restored by
 * \ref RamCfg_Deinit), the NMI redirection cannot be cleared by software (only a reset of the peripheral
 * does it), the mode of such an SRAM cannot be changed any more.
 *
 * # Erase and protection #
 * \ref RamCfg_Set_Erase writes the unlock keys and starts the erase, the content is zero after it. With an
 * erase complete callback the function returns at once and \ref RamCfg_Task reports the end, without the
 * callback it waits for the end. An erase of an SRAM which holds the stack of the caller (main stack pointer
 * or process stack pointer) is refused, the same holds for a write protection of pages which hold the stack.
 * The write protection is locked until the reset of the peripheral.
 *
 * \note  The ECC of a memory which was written with the ECC disabled has no valid code words, the module does
 *        not initialize the memory when it enables the ECC (the erase would destroy the data): the application
 *        erases the SRAM before the first enabling of the ECC.
 * \note  The runtime data of the module and the stack of the application are in an SRAM: the application does
 *        not erase that SRAM.
 *
 */
/* ============================== INCLUDES ================================== */
#include "RamCfg.h"                         /* Self include                   */
#include "RamCfg_Port.h"                    /* Own port file include          */
#include "RamCfg_Types.h"                   /* Module types definitions       */
#include "RamCfg_Reg.h"                     /* Register access                */
#include "Nvic_Port.h"                      /* NVIC handler functionality     */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
#include "Stm32.h"                          /* RAMCFG registers definition    */
/* ============================== TYPEDEFS ================================== */

/** \brief Runtime data of one SRAM */
typedef struct
{
    ramCfg_ReportMode_t    ReportMode;           /**< Reporting mode set by the module                           */
    ramCfg_FunctionState_t Scrubbing;            /**< The word of a single error is written back                 */
    ramCfg_ErrorCnt_t      SingleErrorCnt;       /**< Count of the corrected single errors                       */
    ramCfg_ErrorCnt_t      DoubleErrorCnt;       /**< Count of the double errors                                 */
    ramCfg_Address_t       LastErrorAddress;     /**< Address of the last error, \ref RAMCFG_ADDRESS_UNUSED none */
    ramCfg_FunctionState_t EraseStarted;         /**< An erase was started by \ref RamCfg_Set_Erase              */
    ramCfg_FunctionState_t EraseCallbackPending; /**< The end of the erase is reported by \ref RamCfg_Task       */
}   ramCfg_SramRuntime_t;


/** \brief Runtime data of the module */
typedef struct
{
    ramCfg_FunctionState_t          InitState;               /**< Module initialized by \ref RamCfg_Init             */
    ramCfg_SramRuntime_t            Sram[ RAMCFG_SRAM_CNT ]; /**< Runtime data of the SRAMs                          */
    ramCfg_SingleErrorCallback_t   *SingleErrorCallback;     /**< Single error callback                              */
    ramCfg_DoubleErrorCallback_t   *DoubleErrorCallback;     /**< Double error callback                              */
    ramCfg_EraseCompleteCallback_t *EraseCompleteCallback;   /**< Erase complete callback                            */
    ramCfg_IrqPrio_t                IrqPriority;             /**< Priority of the RAMCFG interrupt                   */
    ramCfg_FunctionState_t          NmiHandlerState;         /**< The NMI handler of the module is installed         */
    nvic_IsrCallback_t              PrevNmiHandler;          /**< NMI handler installed before the one of the module */
}   ramCfg_RuntimeData_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static const ramCfg_SramInfo_t *    RamCfg_Get_SramInfo         ( ramCfg_SramId_t sramId );
static ramCfg_RequestState_t        RamCfg_Check_Access         ( ramCfg_SramId_t sramId,
                                                                  ramCfg_FunctionState_t eccRequired );
static ramCfg_RequestState_t        RamCfg_Check_Config         ( const ramCfg_Config_t * const ramCfgConfig );
static ramCfg_RequestState_t        RamCfg_Check_SramConfig     ( ramCfg_SramId_t sramId,
                                                                  const ramCfg_SramConfig_t * const sramConfig );
static ramCfg_RequestState_t        RamCfg_Check_CheckMode      ( ramCfg_SramId_t sramId,
                                                                  ramCfg_CheckMode_t checkMode );
static ramCfg_RequestState_t        RamCfg_Check_ReportMode     ( ramCfg_SramId_t sramId,
                                                                  ramCfg_ReportMode_t reportMode );
static ramCfg_RequestState_t        RamCfg_Check_Protection     ( ramCfg_SramId_t sramId,
                                                                  ramCfg_Page_t firstPage,
                                                                  ramCfg_PageCnt_t pageCnt );
static ramCfg_FlagState_t           RamCfg_Get_StackInRange     ( uint32_t base, uint32_t size );
static ramCfg_FlagState_t           RamCfg_Get_RangeFlag        ( uint32_t address, uint32_t base, uint32_t size );
static ramCfg_FunctionState_t       RamCfg_Get_EccSupport       ( ramCfg_SramId_t sramId );

static void                         RamCfg_Reset_RuntimeData    ( void );
static ramCfg_RequestState_t        RamCfg_Set_Module           ( const ramCfg_Config_t * const ramCfgConfig );
static ramCfg_RequestState_t        RamCfg_Set_SramConfig       ( ramCfg_SramId_t sramId,
                                                                  const ramCfg_SramConfig_t * const sramConfig );
static ramCfg_RequestState_t        RamCfg_Set_SramCheck        ( ramCfg_SramId_t sramId,
                                                                  ramCfg_CheckMode_t checkMode );
static ramCfg_RequestState_t        RamCfg_Set_SramReport       ( ramCfg_SramId_t sramId,
                                                                  ramCfg_ReportMode_t reportMode );
static ramCfg_RequestState_t        RamCfg_Set_NmiHandler       ( ramCfg_FunctionState_t handlerState );
static ramCfg_RequestState_t        RamCfg_Update_Interrupt     ( void );

static void                         RamCfg_Record_Error         ( ramCfg_SramId_t sramId,
                                                                  ramCfg_Address_t address,
                                                                  ramCfg_FunctionState_t doubleError );
static void                         RamCfg_Scrub_Word           ( ramCfg_SramId_t sramId, ramCfg_Address_t address );
static void                         RamCfg_Service_Single       ( ramCfg_SramId_t sramId );
static void                         RamCfg_Service_Double       ( ramCfg_SramId_t sramId );

static void                         RamCfg_IsrHandler           ( void );
static void                         RamCfg_NmiHandler           ( void );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define RAMCFG_MAJOR_VERSION            ( 0u )

/** Value of minor version of SW module */
#define RAMCFG_MINOR_VERSION            ( 1u )

/** Value of patch version of SW module */
#define RAMCFG_PATCH_VERSION            ( 0u )

/** Default priority of the RAMCFG interrupt used by \ref RamCfg_Get_DefaultConfig */
#define RAMCFG_DEFAULT_IRQ_PRIO         ( 0u )

/** Offset between the secure and the non-secure alias of the SRAMs and of the backup SRAM (address bit 28) */
#define RAMCFG_SECURE_ALIAS_OFFSET      ( 0x10000000u )

/** Mask of the address bits below the 32-bit word (the failing word is aligned down) */
#define RAMCFG_WORD_ALIGN_MASK          ( 3u )

/** Count of pages used when the SRAM has no write protection */
#define RAMCFG_NO_PAGES                 ( RAMCFG_PAGE_CNT_UNUSED )

/** First page of the write protection used by the default configuration (no page is protected) */
#define RAMCFG_PAGE_FIRST               ( 0u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Features of the SRAMs of the device line, indexed by \ref ramCfg_SramId_t.
 *
 * The ECC of SRAM1 of STM32H503 is listed in the reference manual (RM0492), but the RAMCFG control and status
 * registers of SRAM1 have no ECC bits (reserved), SRAM1 is therefore listed without the ECC control.
 * The SRAMs without a clock enable bit in the Rcc module (SRAM4, SRAM5) list a placeholder.
 */
static const ramCfg_SramInfo_t          ramCfg_SramInfo[] =
{
    [ RAMCFG_SRAM_1 ] =
    {
        .SramId = RAMCFG_SRAM_1, .Base = SRAM1_BASE_NS, .Size = SRAM1_SIZE,
        .EccState = RAMCFG_FUNCTION_INACTIVE, .WpPageCnt = RAMCFG_NO_PAGES,
        .ClkState = RAMCFG_FUNCTION_ACTIVE, .ClkPeriph = RCC_PERIPH_SRAM1
    },
    [ RAMCFG_SRAM_2 ] =
    {
        .SramId = RAMCFG_SRAM_2, .Base = SRAM2_BASE_NS, .Size = SRAM2_SIZE,
        .EccState = RAMCFG_FUNCTION_ACTIVE, .WpPageCnt = RAMCFG_REG_WP_PAGE_CNT,
        .ClkState = RAMCFG_FUNCTION_ACTIVE, .ClkPeriph = RCC_PERIPH_SRAM2
    },
#if defined(RAMCFG_SRAM3)
    [ RAMCFG_SRAM_3 ] =
    {
        .SramId = RAMCFG_SRAM_3, .Base = SRAM3_BASE_NS, .Size = SRAM3_SIZE,
#if defined(FLASH_OPTSR2_SRAM3_ECC)
        .EccState = RAMCFG_FUNCTION_ACTIVE, .WpPageCnt = RAMCFG_NO_PAGES,
#else
        .EccState = RAMCFG_FUNCTION_INACTIVE, .WpPageCnt = RAMCFG_NO_PAGES,
#endif
        .ClkState = RAMCFG_FUNCTION_ACTIVE, .ClkPeriph = RCC_PERIPH_SRAM3
    },
#endif
#if defined(RAMCFG_SRAM4)
    [ RAMCFG_SRAM_4 ] =
    {
        .SramId = RAMCFG_SRAM_4, .Base = SRAM4_BASE_NS, .Size = SRAM4_SIZE,
        .EccState = RAMCFG_FUNCTION_INACTIVE, .WpPageCnt = RAMCFG_NO_PAGES,
        .ClkState = RAMCFG_FUNCTION_INACTIVE, .ClkPeriph = RCC_PERIPH_RAMCFG
    },
#endif
#if defined(RAMCFG_SRAM5)
    [ RAMCFG_SRAM_5 ] =
    {
        .SramId = RAMCFG_SRAM_5, .Base = SRAM5_BASE_NS, .Size = SRAM5_SIZE,
        .EccState = RAMCFG_FUNCTION_INACTIVE, .WpPageCnt = RAMCFG_NO_PAGES,
        .ClkState = RAMCFG_FUNCTION_INACTIVE, .ClkPeriph = RCC_PERIPH_RAMCFG
    },
#endif
    [ RAMCFG_SRAM_BACKUP ] =
    {
        .SramId = RAMCFG_SRAM_BACKUP, .Base = BKPSRAM_BASE_NS, .Size = BKPSRAM_SIZE,
        .EccState = RAMCFG_FUNCTION_ACTIVE, .WpPageCnt = RAMCFG_NO_PAGES,
        .ClkState = RAMCFG_FUNCTION_ACTIVE, .ClkPeriph = RCC_PERIPH_BKPSRAM
    },
};

_Static_assert( RAMCFG_SRAM_CNT == ( sizeof( ramCfg_SramInfo ) / sizeof( ramCfg_SramInfo_t ) ),
                "RamCfg: ramCfg_SramInfo has incorrect size." );


/** \brief Runtime data of the module */
static volatile ramCfg_RuntimeData_t ramCfg_RuntimeData =
{
    .InitState             = RAMCFG_FUNCTION_INACTIVE,
    .SingleErrorCallback   = RAMCFG_NULL_PTR,
    .DoubleErrorCallback   = RAMCFG_NULL_PTR,
    .EraseCompleteCallback = RAMCFG_NULL_PTR,
    .IrqPriority           = RAMCFG_DEFAULT_IRQ_PRIO,
    .NmiHandlerState       = RAMCFG_FUNCTION_INACTIVE,
    .PrevNmiHandler        = RAMCFG_NULL_PTR,
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
ramCfg_ModuleVersion_t RamCfg_Get_ModuleVersion( void )
{
    ramCfg_ModuleVersion_t retVersion;

    retVersion.Major = RAMCFG_MAJOR_VERSION;
    retVersion.Minor = RAMCFG_MINOR_VERSION;
    retVersion.Patch = RAMCFG_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initializes the module and configures the SRAMs.
 *
 * Sequence: check of the whole configuration (no register is written when it is invalid), clock of the
 * RAMCFG peripheral (Rcc), deactivation of the interrupt (NVIC), then for every SRAM the check mode, the
 * reporting (with the address latch), the scrubbing and the write protection, finally the NMI vector and the
 * interrupt (NVIC) according to the reporting modes. Every register write is verified by a read-back. The
 * statistics and the erase states of all SRAMs start from zero.
 *
 * \note  Write protection and the NMI redirection are locked until the reset of the RAMCFG peripheral: a repeated
 *        initialization adds to them, it does not remove them.
 * \note  The module does not erase an SRAM when the ECC is enabled (the application erases it before).
 *
 * \pre   Configuration shall be valid: check mode, reporting and scrubbing only the ones the SRAM supports (ECC
 *        and its reporting for SRAMs with ECC, parity is never supported), write protection (a count of pages other
 *        than zero) only for SRAM2 within its pages and not over the stack of the caller; otherwise
 *        \ref RAMCFG_REQUEST_ERROR is returned and nothing is changed. A failure after the first change (clock,
 *        read-back mismatch, interrupt priority refused by NVIC, an SRAM with the NMI redirection set asked for
 *        another reporting) deinitializes the module (\ref RamCfg_Deinit) and returns \ref RAMCFG_REQUEST_ERROR.
 *
 * \param ramCfgConfig [in]: Pointer to RAMCFG configuration, see \ref ramCfg_Config_t. Must not be NULL.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Init( const ramCfg_Config_t * const ramCfgConfig )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != ramCfgConfig )
    {
        /*---------------------- Configuration validation ---------------------*/
        retState = RamCfg_Check_Config( ramCfgConfig );

        /*----------------------- Module configuration ------------------------*/
        if( RAMCFG_REQUEST_OK == retState )
        {
            retState = RamCfg_Set_Module( ramCfgConfig );

            if( RAMCFG_REQUEST_OK != retState )
            {
                /* Self-verification failed - interrupts and callbacks are deactivated */
                (void)RamCfg_Deinit();
            }
            else
            {
                /* Module is configured as required */
            }
        }
        else
        {
            /* Invalid configuration, nothing is written */
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes module RamCfg
 *
 * Disables the error interrupts and the address latch of the SRAMs, the RAMCFG interrupt in NVIC, restores the
 * NMI handler which was installed before the one of the module and removes the callbacks. The statistics and
 * the erase states are cleared. The settings the hardware cannot undo stay: the ECC state, the write protection
 * and the NMI redirection (it is cleared only by a reset of the RAMCFG peripheral, a double error is then handled
 * by the NMI handler restored by this function). The clock of the RAMCFG peripheral stays enabled (the controller
 * of the memories is not switched off).
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Deinit( void )
{
    ramCfg_RequestState_t     retState  = RAMCFG_REQUEST_OK;
    const ramCfg_FunctionState_t initState = ramCfg_RuntimeData.InitState;

    if( RAMCFG_FUNCTION_ACTIVE == initState )
    {
        for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
        {
            const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

            if( RAMCFG_FUNCTION_ACTIVE == sramInfo->EccState )
            {
                const ramCfg_RequestState_t regState = RamCfg_Reg_Set_ReportingOff( sramId );

                if( RAMCFG_REQUEST_OK != regState )
                {
                    retState = RAMCFG_REQUEST_ERROR;
                }
                else
                {
                    /* Interrupts of the SRAM are disabled */
                }
            }
            else
            {
                /* SRAM without ECC has no error interrupts */
            }
        }
    }
    else
    {
        /* Registers were not configured by the module */
    }

    const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_RAMCFG );

    if( NVIC_REQUEST_OK != nvicState )
    {
        retState = RAMCFG_REQUEST_ERROR;
    }
    else
    {
        /* Interrupt is inactive */
    }

    const ramCfg_RequestState_t nmiState = RamCfg_Set_NmiHandler( RAMCFG_FUNCTION_INACTIVE );

    if( RAMCFG_REQUEST_OK != nmiState )
    {
        retState = RAMCFG_REQUEST_ERROR;
    }
    else
    {
        /* NMI handler is restored */
    }

    RamCfg_Reset_RuntimeData();

    return ( retState );
}


/**
 * \brief Main task of module RamCfg
 *
 * Reports the end of an erase started by \ref RamCfg_Set_Erase when the erase complete callback is configured:
 * the callback is called once, from the context of the task. Shall be called periodically. Without a started erase
 * (or before \ref RamCfg_Init) the task does nothing.
 */
void RamCfg_Task( void )
{
    const ramCfg_FunctionState_t initState = ramCfg_RuntimeData.InitState;

    if( RAMCFG_FUNCTION_ACTIVE == initState )
    {
        for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
        {
            const ramCfg_FunctionState_t pendingState = ramCfg_RuntimeData.Sram[ sramId ].EraseCallbackPending;

            if( RAMCFG_FUNCTION_ACTIVE == pendingState )
            {
                const ramCfg_FlagState_t busyFlag = RamCfg_Reg_Get_EraseBusy( sramId );

                if( RAMCFG_FLAG_INACTIVE == busyFlag )
                {
                    ramCfg_EraseCompleteCallback_t * const callback = ramCfg_RuntimeData.EraseCompleteCallback;

                    ramCfg_RuntimeData.Sram[ sramId ].EraseCallbackPending = RAMCFG_FUNCTION_INACTIVE;

                    if( RAMCFG_NULL_PTR != callback )
                    {
                        callback( sramId );
                    }
                    else
                    {
                        /* Erase complete callback is not configured */
                    }
                }
                else
                {
                    /* Erase is still running */
                }
            }
            else
            {
                /* No end of an erase is awaited for the SRAM */
            }
        }
    }
    else
    {
        /* Module is not initialized */
    }
}


/**
 * \brief Fills configuration structure with default values.
 *
 * Default configuration: check mode unchanged (the ECC enabled by the option bytes stays), errors reported by the
 * interrupt for the SRAMs with ECC (no reporting for the others), no scrubbing, no write protection, no callbacks,
 * interrupt priority 0. The function does not access any register.
 *
 * \param ramCfgConfig [out]: Pointer to store default configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_DefaultConfig( ramCfg_Config_t * const ramCfgConfig )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != ramCfgConfig )
    {
        for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
        {
            const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

            ramCfgConfig->Sram[ sramId ].CheckMode           = RAMCFG_CHECK_UNCHANGED;
            ramCfgConfig->Sram[ sramId ].Scrubbing           = RAMCFG_FUNCTION_INACTIVE;
            ramCfgConfig->Sram[ sramId ].ProtectionFirstPage = RAMCFG_PAGE_FIRST;
            ramCfgConfig->Sram[ sramId ].ProtectionPageCnt   = RAMCFG_PAGE_CNT_UNUSED;

            if( RAMCFG_FUNCTION_ACTIVE == sramInfo->EccState )
            {
                ramCfgConfig->Sram[ sramId ].ReportMode = RAMCFG_REPORT_IRQ;
            }
            else
            {
                ramCfgConfig->Sram[ sramId ].ReportMode = RAMCFG_REPORT_NONE;
            }
        }

        ramCfgConfig->SingleErrorCallback   = RAMCFG_NULL_PTR;
        ramCfgConfig->DoubleErrorCallback   = RAMCFG_NULL_PTR;
        ramCfgConfig->EraseCompleteCallback = RAMCFG_NULL_PTR;
        ramCfgConfig->IrqPriority           = RAMCFG_DEFAULT_IRQ_PRIO;

        retState = RAMCFG_REQUEST_OK;
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Sets the check mode of an SRAM.
 *
 * \ref RAMCFG_CHECK_ECC sets the ECC enable bit (verified by read-back), \ref RAMCFG_CHECK_NONE disables the ECC with
 * the unlock sequence of the hardware. \ref RAMCFG_CHECK_NONE is accepted for an SRAM without ECC (it has no check),
 * \ref RAMCFG_CHECK_UNCHANGED changes nothing.
 *
 * \note  A memory written while the ECC was disabled has no valid code words: erase the SRAM before the ECC is
 *        enabled (the module does not do it, the erase destroys the data).
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init. The SRAM shall support the mode (ECC: SRAMs with ECC),
 *        parity is not supported; otherwise \ref RAMCFG_REQUEST_ERROR and no register is written.
 *
 * \param sramId    [in]: SRAM
 * \param checkMode [in]: Check mode
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Set_CheckMode( ramCfg_SramId_t sramId, ramCfg_CheckMode_t checkMode )
{
    ramCfg_RequestState_t retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );

    if( RAMCFG_REQUEST_OK == retState )
    {
        retState = RamCfg_Check_CheckMode( sramId, checkMode );
    }
    else
    {
        /* Module is not initialized or invalid SRAM */
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        retState = RamCfg_Set_SramCheck( sramId, checkMode );
    }
    else
    {
        /* Previous step failed, nothing is written */
    }

    return ( retState );
}


/**
 * \brief Returns the check mode of an SRAM.
 *
 * The mode is read from the ECC enable bit; an SRAM without ECC reports \ref RAMCFG_CHECK_NONE. The function does
 * not enable any clock and does not change anything.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init; otherwise \ref RAMCFG_REQUEST_ERROR.
 *
 * \param sramId    [in]: SRAM
 * \param checkMode [out]: Pointer to store the check mode (\ref RAMCFG_CHECK_NONE or \ref RAMCFG_CHECK_ECC).
 *                         Must not be NULL, it is not changed on an error.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_CheckMode( ramCfg_SramId_t sramId, ramCfg_CheckMode_t * const checkMode )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != checkMode )
    {
        retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == sramInfo->EccState )
        {
            const ramCfg_FunctionState_t eccState = RamCfg_Reg_Get_Ecc( sramId );

            if( RAMCFG_FUNCTION_ACTIVE == eccState )
            {
                *checkMode = RAMCFG_CHECK_ECC;
            }
            else
            {
                *checkMode = RAMCFG_CHECK_NONE;
            }
        }
        else
        {
            *checkMode = RAMCFG_CHECK_NONE;
        }
    }
    else
    {
        /* Module is not initialized, invalid SRAM or pointer */
    }

    return ( retState );
}


/**
 * \brief Sets the reporting mode of the errors of an SRAM.
 *
 * Enables the address latch and the interrupts of the errors (\ref RAMCFG_REPORT_IRQ), or the interrupt of the single
 * error and the NMI of the double error (\ref RAMCFG_REPORT_NMI, the NMI vector is taken over by the module before the
 * redirection is enabled), or disables them (\ref RAMCFG_REPORT_NONE). The RAMCFG interrupt in NVIC is active while at
 * least one SRAM is reported. The registers are verified by read-back.
 *
 * \note  The NMI redirection is set only by software and cleared only by a reset of the RAMCFG peripheral: an SRAM
 *        which has it set refuses every mode other than \ref RAMCFG_REPORT_NMI.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init. The reporting (other than \ref RAMCFG_REPORT_NONE) needs an
 *        SRAM with ECC; otherwise \ref RAMCFG_REQUEST_ERROR and no register is written.
 *
 * \param sramId     [in]: SRAM
 * \param reportMode [in]: Reporting mode
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Set_ReportMode( ramCfg_SramId_t sramId, ramCfg_ReportMode_t reportMode )
{
    ramCfg_RequestState_t retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );

    if( RAMCFG_REQUEST_OK == retState )
    {
        retState = RamCfg_Check_ReportMode( sramId, reportMode );
    }
    else
    {
        /* Module is not initialized or invalid SRAM */
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        retState = RamCfg_Set_SramReport( sramId, reportMode );

        if( RAMCFG_REQUEST_OK == retState )
        {
            retState = RamCfg_Update_Interrupt();
        }
        else
        {
            /* Register write failed, the interrupt keeps its state */
        }
    }
    else
    {
        /* Previous step failed, nothing is written */
    }

    return ( retState );
}


/**
 * \brief Returns the reporting mode of the errors of an SRAM.
 *
 * The mode is read from the interrupt enable bits; an SRAM without ECC reports \ref RAMCFG_REPORT_NONE. The function
 * does not enable any clock and does not change anything.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init; otherwise \ref RAMCFG_REQUEST_ERROR. The enable bits shall
 *        be a mode of the module (not another combination of the bits).
 *
 * \param sramId     [in]: SRAM
 * \param reportMode [out]: Pointer to store the reporting mode. Must not be NULL, it is not changed on an error.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_ReportMode( ramCfg_SramId_t sramId, ramCfg_ReportMode_t * const reportMode )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != reportMode )
    {
        retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == sramInfo->EccState )
        {
            retState = RamCfg_Reg_Get_Reporting( sramId, reportMode );
        }
        else
        {
            *reportMode = RAMCFG_REPORT_NONE;
        }
    }
    else
    {
        /* Module is not initialized, invalid SRAM or pointer */
    }

    return ( retState );
}


/**
 * \brief Activates or deactivates the scrubbing of an SRAM.
 *
 * With the scrubbing active, the interrupt handler reads the failing word of a single error (the hardware returns the
 * corrected value) and writes it back, so the error does not persist in the cell. The scrubbing works while the
 * errors of the SRAM are reported (\ref RAMCFG_REPORT_IRQ or \ref RAMCFG_REPORT_NMI, the address of the error is
 * needed). It is a setting of the module, no register is written.
 *
 * \note  The write-back is not atomic with respect to other masters (DMA): a write of another master to the same
 *        word between the read and the write-back is lost.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init. The activation needs an SRAM with ECC; otherwise
 *        \ref RAMCFG_REQUEST_ERROR.
 *
 * \param sramId    [in]: SRAM
 * \param scrubbing [in]: \ref RAMCFG_FUNCTION_ACTIVE writes the corrected word back
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Set_Scrubbing( ramCfg_SramId_t sramId, ramCfg_FunctionState_t scrubbing )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_FUNCTION_ACTIVE == scrubbing )
    {
        retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_ACTIVE );
    }
    else if( RAMCFG_FUNCTION_INACTIVE == scrubbing )
    {
        retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );
    }
    else
    {
        /* Invalid state */
        retState = RAMCFG_REQUEST_ERROR;
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        ramCfg_RuntimeData.Sram[ sramId ].Scrubbing = scrubbing;
    }
    else
    {
        /* Module is not initialized, invalid SRAM, SRAM without ECC or invalid state */
    }

    return ( retState );
}


/**
 * \brief Returns the statistics of the ECC errors of an SRAM.
 *
 * The counters and the address are the ones recorded by the interrupt handler (the errors of an SRAM which is not
 * reported are not recorded). The function does not read any register.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init and the SRAM shall have ECC; otherwise
 *        \ref RAMCFG_REQUEST_ERROR.
 *
 * \param sramId     [in]: SRAM
 * \param statistics [out]: Pointer to store the statistics. Must not be NULL, it is not changed on an error.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_ErrorStatistics( ramCfg_SramId_t sramId, ramCfg_ErrorStatistics_t * const statistics )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != statistics )
    {
        retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_ACTIVE );
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        const ramCfg_Address_t lastAddress = ramCfg_RuntimeData.Sram[ sramId ].LastErrorAddress;

        statistics->SingleErrorCnt   = ramCfg_RuntimeData.Sram[ sramId ].SingleErrorCnt;
        statistics->DoubleErrorCnt   = ramCfg_RuntimeData.Sram[ sramId ].DoubleErrorCnt;
        statistics->LastErrorAddress = lastAddress;

        if( RAMCFG_ADDRESS_UNUSED == lastAddress )
        {
            statistics->LastErrorSram = RAMCFG_SRAM_UNUSED;
        }
        else
        {
            statistics->LastErrorSram = sramId;
        }
    }
    else
    {
        /* Module is not initialized, invalid SRAM, SRAM without ECC or pointer */
    }

    return ( retState );
}


/**
 * \brief Clears the statistics of the ECC errors of an SRAM (counters and the failing address).
 *
 * The flags and the registers of the hardware are not touched.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init and the SRAM shall have ECC; otherwise
 *        \ref RAMCFG_REQUEST_ERROR.
 *
 * \param sramId [in]: SRAM
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Set_ErrorStatisticsClear( ramCfg_SramId_t sramId )
{
    ramCfg_RequestState_t retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_ACTIVE );

    if( RAMCFG_REQUEST_OK == retState )
    {
        ramCfg_RuntimeData.Sram[ sramId ].SingleErrorCnt   = 0u;
        ramCfg_RuntimeData.Sram[ sramId ].DoubleErrorCnt   = 0u;
        ramCfg_RuntimeData.Sram[ sramId ].LastErrorAddress = RAMCFG_ADDRESS_UNUSED;
    }
    else
    {
        /* Module is not initialized, invalid SRAM or SRAM without ECC */
    }

    return ( retState );
}


/**
 * \brief Returns the failing address of the last ECC error (single or double) of an SRAM.
 *
 * The address is the one recorded by the interrupt handler, as the hardware reports it (an address of the memory map;
 * the module also accepts the offset in the SRAM when it scrubs). \ref RAMCFG_ADDRESS_UNUSED is returned when no error
 * was recorded since the initialization or the last clear. The function does not read any register.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init and the SRAM shall have ECC; otherwise
 *        \ref RAMCFG_REQUEST_ERROR.
 *
 * \param sramId         [in]: SRAM
 * \param failingAddress [out]: Pointer to store the address. Must not be NULL, it is not changed on an error.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_FailingAddress( ramCfg_SramId_t sramId, ramCfg_Address_t * const failingAddress )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != failingAddress )
    {
        retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_ACTIVE );
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        *failingAddress = ramCfg_RuntimeData.Sram[ sramId ].LastErrorAddress;
    }
    else
    {
        /* Module is not initialized, invalid SRAM, SRAM without ECC or pointer */
    }

    return ( retState );
}


/**
 * \brief Write-protects a range of pages of an SRAM (SRAM2).
 *
 * The pages reject the writes until the reset of the RAMCFG peripheral, the protection cannot be removed (a request
 * without a page, which would remove it, is refused). Pages which are protected already stay protected. The protection
 * bits are verified by read-back.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init. The SRAM shall have the write protection (SRAM2), the range
 *        shall have at least one page and lie within the pages of the SRAM (1 Kbyte pages: 16 on STM32H503, 80 on
 *        STM32H523 ... STM32H553, 64 on STM32H562, STM32H563 and STM32H573, 128 on STM32H5E4 ... STM32H5F5) and shall
 *        not contain the stack of the caller (main or process stack pointer); otherwise \ref RAMCFG_REQUEST_ERROR and
 *        no register is written.
 *
 * \param sramId    [in]: SRAM
 * \param firstPage [in]: First page of the range
 * \param pageCnt   [in]: Count of pages of the range
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Set_WriteProtection( ramCfg_SramId_t sramId,
                                                  ramCfg_Page_t firstPage,
                                                  ramCfg_PageCnt_t pageCnt )
{
    ramCfg_RequestState_t retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );

    if( RAMCFG_REQUEST_OK == retState )
    {
        retState = RamCfg_Check_Protection( sramId, firstPage, pageCnt );
    }
    else
    {
        /* Module is not initialized or invalid SRAM */
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        retState = RamCfg_Reg_Set_WriteProtection( sramId, firstPage, pageCnt );
    }
    else
    {
        /* Previous step failed, nothing is written */
    }

    return ( retState );
}


/**
 * \brief Returns whether a range of pages of an SRAM is write-protected.
 *
 * The function does not enable any clock and does not change anything.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init. The SRAM shall have the write protection (SRAM2) and the
 *        range shall have at least one page and lie within the pages of the SRAM; otherwise \ref RAMCFG_REQUEST_ERROR.
 *
 * \param sramId     [in]: SRAM
 * \param firstPage  [in]: First page of the range
 * \param pageCnt    [in]: Count of pages of the range
 * \param protection [out]: Pointer to store the state: \ref RAMCFG_FUNCTION_ACTIVE if every page of the range is
 *                          protected, otherwise \ref RAMCFG_FUNCTION_INACTIVE. Must not be NULL, it is not changed
 *                          on an error.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_WriteProtection( ramCfg_SramId_t sramId,
                                                  ramCfg_Page_t firstPage,
                                                  ramCfg_PageCnt_t pageCnt,
                                                  ramCfg_FunctionState_t * const protection )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != protection )
    {
        retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

        if( RAMCFG_NO_PAGES < sramInfo->WpPageCnt )
        {
            retState = RamCfg_Reg_Get_WriteProtection( sramId, firstPage, pageCnt, protection );
        }
        else
        {
            /* SRAM has no write protection */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else
    {
        /* Module is not initialized, invalid SRAM or pointer */
    }

    return ( retState );
}


/**
 * \brief Starts the software erase of an SRAM.
 *
 * Clock of the memory (Rcc, where the memory has a clock enable bit), unlock keys and start of the erase. The content
 * of the SRAM is zero after the erase, the SRAM is not usable while it runs (the accesses wait). With the erase
 * complete callback configured the function returns at once and \ref RamCfg_Task calls the callback at the end,
 * otherwise the function waits for the end (bounded loop on the hardware flag, error on a timeout - the erase keeps
 * running and \ref RamCfg_Get_EraseState reports it).
 *
 * \note  The erase destroys the data of the SRAM, also the runtime data and the stack of the module and of the
 *        application if they are in it. Only the stack of the caller is checked.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init. The SRAM shall not hold the stack of the caller (main or
 *        process stack pointer) and no erase of the SRAM shall be running; otherwise \ref RAMCFG_REQUEST_ERROR and no
 *        register is written.
 *
 * \param sramId [in]: SRAM to erase
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Set_Erase( ramCfg_SramId_t sramId )
{
    ramCfg_RequestState_t retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );

    if( RAMCFG_REQUEST_OK == retState )
    {
        const ramCfg_SramInfo_t * const sramInfo  = RamCfg_Get_SramInfo( sramId );
        const ramCfg_FlagState_t        stackFlag = RamCfg_Get_StackInRange( sramInfo->Base, sramInfo->Size );
        const ramCfg_FlagState_t        busyFlag  = RamCfg_Reg_Get_EraseBusy( sramId );

        if( ( RAMCFG_FLAG_INACTIVE == stackFlag ) &&
            ( RAMCFG_FLAG_INACTIVE == busyFlag  )    )
        {
            if( RAMCFG_FUNCTION_ACTIVE == sramInfo->ClkState )
            {
                const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( sramInfo->ClkPeriph );

                if( RCC_REQUEST_OK != rccState )
                {
                    retState = RAMCFG_REQUEST_ERROR;
                }
                else
                {
                    /* Clock of the memory is active */
                }
            }
            else
            {
                /* The clock of the memory has no enable bit in the Rcc module */
            }
        }
        else
        {
            /* The stack of the caller is in the SRAM or an erase is running */
            retState = RAMCFG_REQUEST_ERROR;
        }

        if( RAMCFG_REQUEST_OK == retState )
        {
            retState = RamCfg_Reg_Set_EraseStart( sramId );
        }
        else
        {
            /* Previous step failed, nothing is written */
        }

        if( RAMCFG_REQUEST_OK == retState )
        {
            ramCfg_EraseCompleteCallback_t * const callback = ramCfg_RuntimeData.EraseCompleteCallback;

            ramCfg_RuntimeData.Sram[ sramId ].EraseStarted = RAMCFG_FUNCTION_ACTIVE;

            if( RAMCFG_NULL_PTR != callback )
            {
                ramCfg_RuntimeData.Sram[ sramId ].EraseCallbackPending = RAMCFG_FUNCTION_ACTIVE;
            }
            else
            {
                retState = RamCfg_Reg_Wait_EraseEnd( sramId );
            }
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        /* Module is not initialized or invalid SRAM */
    }

    return ( retState );
}


/**
 * \brief Returns the state of the erase of an SRAM.
 *
 * \ref RAMCFG_ERASE_BUSY while the hardware reports a running erase (also an erase started by a reset, a tamper event
 * or a regression of the product state), \ref RAMCFG_ERASE_DONE when an erase requested by \ref RamCfg_Set_Erase is
 * finished, otherwise \ref RAMCFG_ERASE_IDLE. The function does not enable any clock and does not change anything.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init; otherwise \ref RAMCFG_REQUEST_ERROR.
 *
 * \param sramId     [in]: SRAM
 * \param eraseState [out]: Pointer to store the erase state. Must not be NULL, it is not changed on an error.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_EraseState( ramCfg_SramId_t sramId, ramCfg_EraseState_t * const eraseState )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != eraseState )
    {
        retState = RamCfg_Check_Access( sramId, RAMCFG_FUNCTION_INACTIVE );
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    if( RAMCFG_REQUEST_OK == retState )
    {
        const ramCfg_FlagState_t     busyFlag = RamCfg_Reg_Get_EraseBusy( sramId );
        const ramCfg_FunctionState_t started  = ramCfg_RuntimeData.Sram[ sramId ].EraseStarted;

        if( RAMCFG_FLAG_ACTIVE == busyFlag )
        {
            *eraseState = RAMCFG_ERASE_BUSY;
        }
        else if( RAMCFG_FUNCTION_ACTIVE == started )
        {
            *eraseState = RAMCFG_ERASE_DONE;
        }
        else
        {
            *eraseState = RAMCFG_ERASE_IDLE;
        }
    }
    else
    {
        /* Module is not initialized, invalid SRAM or pointer */
    }

    return ( retState );
}


/**
 * \brief Sets the wait states of an SRAM.
 *
 * The SRAMs of STM32H5 have no setting of the wait states (the controller has no such register), the function keeps
 * its place in the common interface and always returns an error.
 *
 * \param sramId     [in]: SRAM (not used)
 * \param waitStates [in]: Count of the wait states (not used)
 *
 * \return Always \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Set_WaitStates( ramCfg_SramId_t sramId, ramCfg_WaitStates_t waitStates )
{
    (void)sramId;
    (void)waitStates;

    return ( RAMCFG_REQUEST_ERROR );
}


/**
 * \brief Returns the wait states of an SRAM.
 *
 * The SRAMs of STM32H5 have no setting of the wait states, the function keeps its place in the common interface and
 * always returns an error, the output is not changed.
 *
 * \param sramId     [in]: SRAM (not used)
 * \param waitStates [out]: Pointer to the count of the wait states (not used)
 *
 * \return Always \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_WaitStates( ramCfg_SramId_t sramId, ramCfg_WaitStates_t * const waitStates )
{
    (void)sramId;
    (void)waitStates;

    return ( RAMCFG_REQUEST_ERROR );
}


/**
 * \brief Sets the priority of the RAMCFG interrupt.
 *
 * The priority is passed to the NVIC module (which checks its range) and used when the interrupt is activated. The
 * NMI has a fixed priority.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init; otherwise \ref RAMCFG_REQUEST_ERROR.
 *
 * \param irqPrio [in]: Interrupt priority (0 - highest priority)
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Set_IrqPriority( ramCfg_IrqPrio_t irqPrio )
{
    ramCfg_RequestState_t        retState  = RAMCFG_REQUEST_ERROR;
    const ramCfg_FunctionState_t initState = ramCfg_RuntimeData.InitState;

    if( RAMCFG_FUNCTION_ACTIVE == initState )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_RAMCFG, irqPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            ramCfg_RuntimeData.IrqPriority = irqPrio;
            retState = RAMCFG_REQUEST_OK;
        }
        else
        {
            /* Priority refused by NVIC */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else
    {
        /* Module is not initialized */
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the priority of the RAMCFG interrupt.
 *
 * The priority is read from the NVIC module. The function does not change anything.
 *
 * \pre   Module shall be initialized by \ref RamCfg_Init; otherwise \ref RAMCFG_REQUEST_ERROR.
 *
 * \param irqPrio [out]: Pointer to store the priority. Must not be NULL.
 *
 * \return Function processing state. Returns \ref RAMCFG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
ramCfg_RequestState_t RamCfg_Get_IrqPriority( ramCfg_IrqPrio_t * const irqPrio )
{
    ramCfg_RequestState_t        retState  = RAMCFG_REQUEST_ERROR;
    const ramCfg_FunctionState_t initState = ramCfg_RuntimeData.InitState;

    if( ( RAMCFG_NULL_PTR          != irqPrio  ) &&
        ( RAMCFG_FUNCTION_ACTIVE   == initState )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( NVIC_PERIPH_IRQ_RAMCFG, irqPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = RAMCFG_REQUEST_OK;
        }
        else
        {
            /* NVIC does not know the priority */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else
    {
        /* Module is not initialized or invalid pointer */
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Returns the features of an SRAM.
 *
 * \param sramId [in]: SRAM
 *
 * \return Pointer to the item of the table of the SRAMs, \ref RAMCFG_NULL_PTR for an invalid SRAM.
 */
static const ramCfg_SramInfo_t * RamCfg_Get_SramInfo( ramCfg_SramId_t sramId )
{
    const ramCfg_SramInfo_t * retInfo = RAMCFG_NULL_PTR;

    if( RAMCFG_SRAM_CNT > sramId )
    {
        retInfo = &ramCfg_SramInfo[ sramId ];
    }
    else
    {
        retInfo = RAMCFG_NULL_PTR;
    }

    return ( retInfo );
}


/**
 * \brief Checks that a function may access an SRAM: the module is initialized, the SRAM exists and has the ECC if
 *        it is required.
 *
 * \param sramId      [in]: SRAM
 * \param eccRequired [in]: \ref RAMCFG_FUNCTION_ACTIVE requires an SRAM with ECC
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the access is allowed. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Check_Access( ramCfg_SramId_t sramId, ramCfg_FunctionState_t eccRequired )
{
    ramCfg_RequestState_t         retState  = RAMCFG_REQUEST_ERROR;
    const ramCfg_FunctionState_t  initState = ramCfg_RuntimeData.InitState;
    const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

    if( ( RAMCFG_FUNCTION_ACTIVE == initState ) &&
        ( RAMCFG_NULL_PTR        != sramInfo  )    )
    {
        if( ( RAMCFG_FUNCTION_ACTIVE   == eccRequired        ) &&
            ( RAMCFG_FUNCTION_INACTIVE == sramInfo->EccState )    )
        {
            /* The SRAM has no ECC */
            retState = RAMCFG_REQUEST_ERROR;
        }
        else
        {
            retState = RAMCFG_REQUEST_OK;
        }
    }
    else
    {
        /* Module is not initialized or invalid SRAM */
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the whole configuration, no register is touched.
 *
 * \param ramCfgConfig [in]: Pointer to RAMCFG configuration. Must not be NULL.
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the configuration of every SRAM is valid. Otherwise returns
 *         \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Check_Config( const ramCfg_Config_t * const ramCfgConfig )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != ramCfgConfig )
    {
        retState = RAMCFG_REQUEST_OK;

        for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1;
             ( RAMCFG_SRAM_CNT     >  sramId   ) &&
             ( RAMCFG_REQUEST_OK   == retState );
             sramId ++ )
        {
            retState = RamCfg_Check_SramConfig( sramId, &ramCfgConfig->Sram[ sramId ] );
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the configuration of an SRAM against the features of the SRAM, no register is touched.
 *
 * \param sramId     [in]: SRAM
 * \param sramConfig [in]: Pointer to the configuration of the SRAM. Must not be NULL.
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the configuration is valid. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Check_SramConfig( ramCfg_SramId_t sramId,
                                                      const ramCfg_SramConfig_t * const sramConfig )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != sramConfig )
    {
        retState = RamCfg_Check_CheckMode( sramId, sramConfig->CheckMode );

        if( RAMCFG_REQUEST_OK == retState )
        {
            retState = RamCfg_Check_ReportMode( sramId, sramConfig->ReportMode );
        }
        else
        {
            /* Previous step failed */
        }

        if( RAMCFG_REQUEST_OK == retState )
        {
            const ramCfg_FunctionState_t eccSupport = RamCfg_Get_EccSupport( sramId );

            if( ( RAMCFG_FUNCTION_ACTIVE == sramConfig->Scrubbing ) &&
                ( RAMCFG_FUNCTION_ACTIVE == eccSupport            )    )
            {
                /* Scrubbing of an SRAM with ECC */
                retState = RAMCFG_REQUEST_OK;
            }
            else if( RAMCFG_FUNCTION_INACTIVE == sramConfig->Scrubbing )
            {
                /* No scrubbing, no feature of the SRAM is needed */
                retState = RAMCFG_REQUEST_OK;
            }
            else
            {
                /* Invalid state or scrubbing of an SRAM without ECC */
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Previous step failed */
        }

        if( ( RAMCFG_REQUEST_OK        == retState                       ) &&
            ( RAMCFG_PAGE_CNT_UNUSED   != sramConfig->ProtectionPageCnt  )    )
        {
            retState = RamCfg_Check_Protection( sramId,
                                                sramConfig->ProtectionFirstPage,
                                                sramConfig->ProtectionPageCnt );
        }
        else
        {
            /* Write protection is not requested or previous step failed */
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks a check mode against the features of an SRAM.
 *
 * \param sramId    [in]: SRAM
 * \param checkMode [in]: Check mode
 *
 * \return Returns \ref RAMCFG_REQUEST_OK for unchanged, none (every SRAM) and ECC (SRAM with ECC). Returns
 *         \ref RAMCFG_REQUEST_ERROR for parity, for ECC of an SRAM without it and for an invalid value or SRAM.
 */
static ramCfg_RequestState_t RamCfg_Check_CheckMode( ramCfg_SramId_t sramId, ramCfg_CheckMode_t checkMode )
{
    ramCfg_RequestState_t           retState = RAMCFG_REQUEST_ERROR;
    const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

    if( RAMCFG_NULL_PTR != sramInfo )
    {
        if( ( RAMCFG_CHECK_UNCHANGED == checkMode ) ||
            ( RAMCFG_CHECK_NONE      == checkMode )    )
        {
            retState = RAMCFG_REQUEST_OK;
        }
        else if( RAMCFG_CHECK_ECC == checkMode )
        {
            if( RAMCFG_FUNCTION_ACTIVE == sramInfo->EccState )
            {
                retState = RAMCFG_REQUEST_OK;
            }
            else
            {
                /* The SRAM has no ECC */
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Parity check does not exist on STM32H5, other values are invalid */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks a reporting mode against the features of an SRAM.
 *
 * \param sramId     [in]: SRAM
 * \param reportMode [in]: Reporting mode
 *
 * \return Returns \ref RAMCFG_REQUEST_OK for none (every SRAM), interrupt and NMI (SRAM with ECC). Returns
 *         \ref RAMCFG_REQUEST_ERROR for the reporting of an SRAM without ECC and for an invalid value or SRAM.
 */
static ramCfg_RequestState_t RamCfg_Check_ReportMode( ramCfg_SramId_t sramId, ramCfg_ReportMode_t reportMode )
{
    ramCfg_RequestState_t           retState = RAMCFG_REQUEST_ERROR;
    const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

    if( RAMCFG_NULL_PTR != sramInfo )
    {
        if( RAMCFG_REPORT_NONE == reportMode )
        {
            retState = RAMCFG_REQUEST_OK;
        }
        else if( ( RAMCFG_REPORT_IRQ == reportMode ) ||
                 ( RAMCFG_REPORT_NMI == reportMode )    )
        {
            if( RAMCFG_FUNCTION_ACTIVE == sramInfo->EccState )
            {
                retState = RAMCFG_REQUEST_OK;
            }
            else
            {
                /* The SRAM has no ECC, there is no error to report */
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Invalid value */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks a range of write protection pages against the features of an SRAM and the stack of the caller.
 *
 * \param sramId    [in]: SRAM
 * \param firstPage [in]: First page of the range
 * \param pageCnt   [in]: Count of pages of the range
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the SRAM has the write protection, the range has at least one page and lies
 *         within the pages of the SRAM and does not hold the stack of the caller. Otherwise returns
 *         \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Check_Protection( ramCfg_SramId_t sramId,
                                                      ramCfg_Page_t firstPage,
                                                      ramCfg_PageCnt_t pageCnt )
{
    ramCfg_RequestState_t           retState = RAMCFG_REQUEST_ERROR;
    const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

    if( RAMCFG_NULL_PTR != sramInfo )
    {
        if( ( RAMCFG_NO_PAGES               <  sramInfo->WpPageCnt                  ) &&
            ( RAMCFG_PAGE_CNT_UNUSED        <  pageCnt                              ) &&
            ( sramInfo->WpPageCnt           >= ( (uint32_t)firstPage + pageCnt )    )    )
        {
            const uint32_t rangeBase = ( sramInfo->Base + ( (uint32_t)firstPage * RAMCFG_PAGE_SIZE ) );
            const uint32_t rangeSize = ( (uint32_t)pageCnt * RAMCFG_PAGE_SIZE );
            const ramCfg_FlagState_t stackFlag = RamCfg_Get_StackInRange( rangeBase, rangeSize );

            if( RAMCFG_FLAG_INACTIVE == stackFlag )
            {
                retState = RAMCFG_REQUEST_OK;
            }
            else
            {
                /* The pages hold the stack of the caller, a write to a protected page would fault */
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
        else
        {
            /* SRAM without write protection, empty range or range beyond the pages */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns whether the stack of the caller (main or process stack pointer) lies in an address range.
 *
 * The secure alias of the address is folded to the non-secure one, so the stack pointer is found in either alias.
 *
 * \param base [in]: First address of the range (non-secure alias)
 * \param size [in]: Size of the range in bytes
 *
 * \return \ref RAMCFG_FLAG_ACTIVE if one of the stack pointers is inside the range, otherwise
 *         \ref RAMCFG_FLAG_INACTIVE.
 */
static ramCfg_FlagState_t RamCfg_Get_StackInRange( uint32_t base, uint32_t size )
{
    ramCfg_FlagState_t       retState = RAMCFG_FLAG_INACTIVE;
    const uint32_t           mspValue = ( __get_MSP() & ~RAMCFG_SECURE_ALIAS_OFFSET );
    const uint32_t           pspValue = ( __get_PSP() & ~RAMCFG_SECURE_ALIAS_OFFSET );
    const ramCfg_FlagState_t mspFlag  = RamCfg_Get_RangeFlag( mspValue, base, size );
    const ramCfg_FlagState_t pspFlag  = RamCfg_Get_RangeFlag( pspValue, base, size );

    if( ( RAMCFG_FLAG_ACTIVE == mspFlag ) ||
        ( RAMCFG_FLAG_ACTIVE == pspFlag )    )
    {
        retState = RAMCFG_FLAG_ACTIVE;
    }
    else
    {
        retState = RAMCFG_FLAG_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Returns whether an address lies in an address range.
 *
 * \param address [in]: Address
 * \param base    [in]: First address of the range
 * \param size    [in]: Size of the range in bytes
 *
 * \return \ref RAMCFG_FLAG_ACTIVE if the address is in the range (the first address belongs to it, the first address
 *         behind the range does not), otherwise \ref RAMCFG_FLAG_INACTIVE.
 */
static ramCfg_FlagState_t RamCfg_Get_RangeFlag( uint32_t address, uint32_t base, uint32_t size )
{
    ramCfg_FlagState_t retState = RAMCFG_FLAG_INACTIVE;

    if( ( base            <= address ) &&
        ( ( base + size ) >  address )    )
    {
        retState = RAMCFG_FLAG_ACTIVE;
    }
    else
    {
        retState = RAMCFG_FLAG_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Returns whether the RAMCFG controls the ECC of an SRAM.
 *
 * \param sramId [in]: SRAM
 *
 * \return \ref RAMCFG_FUNCTION_ACTIVE for an SRAM with ECC, otherwise \ref RAMCFG_FUNCTION_INACTIVE (also for an
 *         invalid SRAM).
 */
static ramCfg_FunctionState_t RamCfg_Get_EccSupport( ramCfg_SramId_t sramId )
{
    ramCfg_FunctionState_t          retState = RAMCFG_FUNCTION_INACTIVE;
    const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

    if( RAMCFG_NULL_PTR != sramInfo )
    {
        retState = sramInfo->EccState;
    }
    else
    {
        retState = RAMCFG_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Resets the runtime data to the state before the initialization (the bookkeeping of the NMI handler stays).
 */
static void RamCfg_Reset_RuntimeData( void )
{
    ramCfg_RuntimeData.InitState             = RAMCFG_FUNCTION_INACTIVE;
    ramCfg_RuntimeData.SingleErrorCallback   = RAMCFG_NULL_PTR;
    ramCfg_RuntimeData.DoubleErrorCallback   = RAMCFG_NULL_PTR;
    ramCfg_RuntimeData.EraseCompleteCallback = RAMCFG_NULL_PTR;
    ramCfg_RuntimeData.IrqPriority           = RAMCFG_DEFAULT_IRQ_PRIO;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        ramCfg_RuntimeData.Sram[ sramId ].ReportMode           = RAMCFG_REPORT_NONE;
        ramCfg_RuntimeData.Sram[ sramId ].Scrubbing            = RAMCFG_FUNCTION_INACTIVE;
        ramCfg_RuntimeData.Sram[ sramId ].SingleErrorCnt       = 0u;
        ramCfg_RuntimeData.Sram[ sramId ].DoubleErrorCnt       = 0u;
        ramCfg_RuntimeData.Sram[ sramId ].LastErrorAddress     = RAMCFG_ADDRESS_UNUSED;
        ramCfg_RuntimeData.Sram[ sramId ].EraseStarted         = RAMCFG_FUNCTION_INACTIVE;
        ramCfg_RuntimeData.Sram[ sramId ].EraseCallbackPending = RAMCFG_FUNCTION_INACTIVE;
    }
}


/**
 * \brief Applies the validated configuration: runtime data, clock, interrupt, SRAMs.
 *
 * \param ramCfgConfig [in]: Pointer to RAMCFG configuration. Must not be NULL.
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the SRAMs are configured as required. Otherwise returns
 *         \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Set_Module( const ramCfg_Config_t * const ramCfgConfig )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_NULL_PTR != ramCfgConfig )
    {
        /*---------------- Interrupt deactivation (reconfiguration) -------*/
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_RAMCFG );

        /*--------------- Runtime data (callbacks, priority, statistics) -------*/
        RamCfg_Reset_RuntimeData();

        ramCfg_RuntimeData.SingleErrorCallback   = ramCfgConfig->SingleErrorCallback;
        ramCfg_RuntimeData.DoubleErrorCallback   = ramCfgConfig->DoubleErrorCallback;
        ramCfg_RuntimeData.EraseCompleteCallback = ramCfgConfig->EraseCompleteCallback;
        ramCfg_RuntimeData.IrqPriority           = ramCfgConfig->IrqPriority;

        /*------------------------- Peripheral clock -------------------------*/
        if( NVIC_REQUEST_OK == nvicState )
        {
            const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( RCC_PERIPH_RAMCFG );

            if( RCC_REQUEST_OK == rccState )
            {
                retState = RAMCFG_REQUEST_OK;
            }
            else
            {
                /* Peripheral clock can not be activated */
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Interrupt can not be deactivated */
            retState = RAMCFG_REQUEST_ERROR;
        }

        /*------------------------------ SRAMs --------------------------------*/
        if( RAMCFG_REQUEST_OK == retState )
        {
            ramCfg_RuntimeData.InitState = RAMCFG_FUNCTION_ACTIVE;

            for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1;
                 ( RAMCFG_SRAM_CNT     >  sramId   ) &&
                 ( RAMCFG_REQUEST_OK   == retState );
                 sramId ++ )
            {
                retState = RamCfg_Set_SramConfig( sramId, &ramCfgConfig->Sram[ sramId ] );
            }
        }
        else
        {
            /* Previous step failed */
        }

        /*--------------------- Interrupt and NMI vector ----------------------*/
        if( RAMCFG_REQUEST_OK == retState )
        {
            retState = RamCfg_Update_Interrupt();
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Applies the configuration of an SRAM: check mode, reporting, scrubbing, write protection.
 *
 * \param sramId     [in]: SRAM
 * \param sramConfig [in]: Pointer to the validated configuration of the SRAM. Must not be NULL.
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the SRAM is configured as required. Otherwise returns
 *         \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Set_SramConfig( ramCfg_SramId_t sramId,
                                                    const ramCfg_SramConfig_t * const sramConfig )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( ( RAMCFG_NULL_PTR != sramConfig ) &&
        ( RAMCFG_SRAM_CNT  > sramId     )    )
    {
        retState = RamCfg_Set_SramCheck( sramId, sramConfig->CheckMode );

        if( RAMCFG_REQUEST_OK == retState )
        {
            retState = RamCfg_Set_SramReport( sramId, sramConfig->ReportMode );
        }
        else
        {
            /* Previous step failed */
        }

        if( RAMCFG_REQUEST_OK == retState )
        {
            ramCfg_RuntimeData.Sram[ sramId ].Scrubbing = sramConfig->Scrubbing;

            if( RAMCFG_PAGE_CNT_UNUSED != sramConfig->ProtectionPageCnt )
            {
                retState = RamCfg_Reg_Set_WriteProtection( sramId,
                                                           sramConfig->ProtectionFirstPage,
                                                           sramConfig->ProtectionPageCnt );
            }
            else
            {
                /* Write protection is not requested */
            }
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the check mode of an SRAM (the mode is validated by the caller).
 *
 * \param sramId    [in]: SRAM
 * \param checkMode [in]: Check mode
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the mode is set (nothing is written for unchanged and for none of an
 *         SRAM without ECC). Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Set_SramCheck( ramCfg_SramId_t sramId, ramCfg_CheckMode_t checkMode )
{
    ramCfg_RequestState_t           retState = RAMCFG_REQUEST_ERROR;
    const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

    if( RAMCFG_NULL_PTR != sramInfo )
    {
        if( RAMCFG_CHECK_UNCHANGED == checkMode )
        {
            /* The state of the hardware is kept */
            retState = RAMCFG_REQUEST_OK;
        }
        else if( RAMCFG_CHECK_NONE == checkMode )
        {
            if( RAMCFG_FUNCTION_ACTIVE == sramInfo->EccState )
            {
                retState = RamCfg_Reg_Set_Ecc( sramId, RAMCFG_FUNCTION_INACTIVE );
            }
            else
            {
                /* The SRAM has no check */
                retState = RAMCFG_REQUEST_OK;
            }
        }
        else if( RAMCFG_CHECK_ECC == checkMode )
        {
            retState = RamCfg_Reg_Set_Ecc( sramId, RAMCFG_FUNCTION_ACTIVE );
        }
        else
        {
            /* Not a mode of STM32H5 */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the reporting mode of an SRAM and stores it in the runtime data (the mode is validated by the caller).
 *
 * For the NMI mode the NMI vector is taken over before the NMI redirection is enabled. The interrupt in NVIC is
 * not changed here (\ref RamCfg_Update_Interrupt).
 *
 * \param sramId     [in]: SRAM
 * \param reportMode [in]: Reporting mode
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the mode is set (nothing is written for none of an SRAM without ECC).
 *         Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Set_SramReport( ramCfg_SramId_t sramId, ramCfg_ReportMode_t reportMode )
{
    ramCfg_RequestState_t           retState = RAMCFG_REQUEST_ERROR;
    const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

    if( RAMCFG_NULL_PTR != sramInfo )
    {
        if( RAMCFG_FUNCTION_ACTIVE == sramInfo->EccState )
        {
            retState = RAMCFG_REQUEST_OK;

            if( RAMCFG_REPORT_NMI == reportMode )
            {
                retState = RamCfg_Set_NmiHandler( RAMCFG_FUNCTION_ACTIVE );
            }
            else
            {
                /* The NMI vector is not needed */
            }

            if( RAMCFG_REQUEST_OK == retState )
            {
                retState = RamCfg_Reg_Set_Reporting( sramId, reportMode );
            }
            else
            {
                /* Previous step failed */
            }
        }
        else
        {
            /* The SRAM has no ECC, there is nothing to report (the reporting was validated by the caller) */
            retState = RAMCFG_REQUEST_OK;
        }

        if( RAMCFG_REQUEST_OK == retState )
        {
            ramCfg_RuntimeData.Sram[ sramId ].ReportMode = reportMode;
        }
        else
        {
            /* The mode was not set */
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Installs the NMI handler of the module (the previous one is saved) or restores the previous handler.
 *
 * \param handlerState [in]: \ref RAMCFG_FUNCTION_ACTIVE installs the handler of the module,
 *                           \ref RAMCFG_FUNCTION_INACTIVE restores the previous one. Nothing is done when the handler
 *                           is in the requested state.
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the NMI vector is in the requested state. Otherwise returns
 *         \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Set_NmiHandler( ramCfg_FunctionState_t handlerState )
{
    ramCfg_RequestState_t        retState     = RAMCFG_REQUEST_OK;
    const ramCfg_FunctionState_t installState = ramCfg_RuntimeData.NmiHandlerState;

    if( ( RAMCFG_FUNCTION_ACTIVE   == handlerState ) &&
        ( RAMCFG_FUNCTION_INACTIVE == installState )    )
    {
        nvic_IsrCallback_t        prevHandler = RAMCFG_NULL_PTR;
        const nvic_RequestState_t getState    = Nvic_Get_CoreIrq_Handler( NVIC_CORE_IRQ_NMI, &prevHandler );

        if( NVIC_REQUEST_OK == getState )
        {
            const nvic_RequestState_t setState = Nvic_Set_CoreIrq_Handler( NVIC_CORE_IRQ_NMI, RamCfg_NmiHandler );

            if( NVIC_REQUEST_OK == setState )
            {
                ramCfg_RuntimeData.PrevNmiHandler  = prevHandler;
                ramCfg_RuntimeData.NmiHandlerState = RAMCFG_FUNCTION_ACTIVE;
            }
            else
            {
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
        else
        {
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else if( ( RAMCFG_FUNCTION_INACTIVE == handlerState ) &&
             ( RAMCFG_FUNCTION_ACTIVE   == installState )    )
    {
        const nvic_IsrCallback_t prevHandler = ramCfg_RuntimeData.PrevNmiHandler;

        if( RAMCFG_NULL_PTR != prevHandler )
        {
            const nvic_RequestState_t setState = Nvic_Set_CoreIrq_Handler( NVIC_CORE_IRQ_NMI, prevHandler );

            if( NVIC_REQUEST_OK == setState )
            {
                ramCfg_RuntimeData.NmiHandlerState = RAMCFG_FUNCTION_INACTIVE;
            }
            else
            {
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
        else
        {
            /* There was no handler to restore */
            ramCfg_RuntimeData.NmiHandlerState = RAMCFG_FUNCTION_INACTIVE;
        }
    }
    else
    {
        /* The NMI vector is in the requested state */
    }

    return ( retState );
}


/**
 * \brief Sets the RAMCFG interrupt in NVIC and the NMI vector according to the reporting modes of the SRAMs.
 *
 * The interrupt (handler, priority, activation) is used while at least one SRAM is reported (interrupt or NMI mode,
 * the single error is always reported by the interrupt), otherwise it is deactivated. The NMI handler of the module is
 * restored to the previous one when no SRAM uses the NMI mode.
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the NVIC accepted every request. Otherwise returns
 *         \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Update_Interrupt( void )
{
    ramCfg_RequestState_t  retState = RAMCFG_REQUEST_OK;
    ramCfg_FunctionState_t irqUsed  = RAMCFG_FUNCTION_INACTIVE;
    ramCfg_FunctionState_t nmiUsed  = RAMCFG_FUNCTION_INACTIVE;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const ramCfg_ReportMode_t reportMode = ramCfg_RuntimeData.Sram[ sramId ].ReportMode;

        if( ( RAMCFG_REPORT_IRQ == reportMode ) ||
            ( RAMCFG_REPORT_NMI == reportMode )    )
        {
            irqUsed = RAMCFG_FUNCTION_ACTIVE;
        }
        else
        {
            /* The SRAM is not reported */
        }

        if( RAMCFG_REPORT_NMI == reportMode )
        {
            nmiUsed = RAMCFG_FUNCTION_ACTIVE;
        }
        else
        {
            /* The SRAM does not use the NMI */
        }
    }

    if( RAMCFG_FUNCTION_INACTIVE == nmiUsed )
    {
        retState = RamCfg_Set_NmiHandler( RAMCFG_FUNCTION_INACTIVE );
    }
    else
    {
        /* The NMI handler of the module is needed (installed before the NMI redirection was enabled) */
    }

    if( RAMCFG_FUNCTION_ACTIVE == irqUsed )
    {
        nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Handler( NVIC_PERIPH_IRQ_RAMCFG, RamCfg_IsrHandler );

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_RAMCFG, ramCfg_RuntimeData.IrqPriority );
        }
        else
        {
            /* Handler configuration failed */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Active( NVIC_PERIPH_IRQ_RAMCFG );
        }
        else
        {
            /* Previous step failed */
        }

        if( NVIC_REQUEST_OK != nvicState )
        {
            retState = RAMCFG_REQUEST_ERROR;
        }
        else
        {
            /* Interrupt is active */
        }
    }
    else
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_RAMCFG );

        if( NVIC_REQUEST_OK != nvicState )
        {
            retState = RAMCFG_REQUEST_ERROR;
        }
        else
        {
            /* Interrupt is inactive */
        }
    }

    return ( retState );
}


/**
 * \brief Records an error of an SRAM in the statistics (the counters wrap around after 2^32 errors).
 *
 * \param sramId      [in]: SRAM
 * \param address     [in]: Address of the failing word
 * \param doubleError [in]: \ref RAMCFG_FUNCTION_ACTIVE for a double error, \ref RAMCFG_FUNCTION_INACTIVE for a single
 *                          one
 */
static void RamCfg_Record_Error( ramCfg_SramId_t sramId, ramCfg_Address_t address, ramCfg_FunctionState_t doubleError )
{
    if( RAMCFG_FUNCTION_ACTIVE == doubleError )
    {
        const ramCfg_ErrorCnt_t errorCnt = ramCfg_RuntimeData.Sram[ sramId ].DoubleErrorCnt;

        ramCfg_RuntimeData.Sram[ sramId ].DoubleErrorCnt = ( errorCnt + 1u );
    }
    else
    {
        const ramCfg_ErrorCnt_t errorCnt = ramCfg_RuntimeData.Sram[ sramId ].SingleErrorCnt;

        ramCfg_RuntimeData.Sram[ sramId ].SingleErrorCnt = ( errorCnt + 1u );
    }

    ramCfg_RuntimeData.Sram[ sramId ].LastErrorAddress = address;
}


/**
 * \brief Scrubs the failing word of a single error: reads it (the hardware returns the corrected value) and
 *        writes it back.
 *
 * The address of the hardware is accepted as the address of the memory map (either alias) or as the offset in the SRAM;
 * an address outside of the SRAM is not scrubbed. The word is aligned down.
 *
 * \param sramId  [in]: SRAM
 * \param address [in]: Address of the failing word as reported by the hardware
 */
static void RamCfg_Scrub_Word( ramCfg_SramId_t sramId, ramCfg_Address_t address )
{
    const ramCfg_SramInfo_t * const sramInfo = RamCfg_Get_SramInfo( sramId );

    if( RAMCFG_NULL_PTR != sramInfo )
    {
        const uint32_t foldedAddr = ( address & ~RAMCFG_SECURE_ALIAS_OFFSET );
        uint32_t       wordAddr   = RAMCFG_ADDRESS_UNUSED;

        const ramCfg_FlagState_t mapFlag = RamCfg_Get_RangeFlag( foldedAddr, sramInfo->Base, sramInfo->Size );

        if( RAMCFG_FLAG_ACTIVE == mapFlag )
        {
            /* Address of the memory map */
            wordAddr = address;
        }
        else if( sramInfo->Size > address )
        {
            /* Offset in the SRAM */
            wordAddr = ( sramInfo->Base + address );
        }
        else
        {
            /* The address is not in the SRAM, the word is not scrubbed */
            wordAddr = RAMCFG_ADDRESS_UNUSED;
        }

        if( RAMCFG_ADDRESS_UNUSED != wordAddr )
        {
            volatile uint32_t * const word  = (volatile uint32_t *)(uintptr_t)( wordAddr & ~RAMCFG_WORD_ALIGN_MASK );
            const uint32_t            value = *word;

            *word = value;
        }
        else
        {
            /* Nothing to scrub */
        }
    }
    else
    {
        /* Invalid SRAM */
    }
}


/**
 * \brief Services the single error of an SRAM: records it, scrubs the word, acknowledges the flag, calls the callback.
 *
 * The flag is acknowledged after the scrub (the read of the failing cell reports the error again, the repeated
 * report is acknowledged with it) and before the callback.
 *
 * \param sramId [in]: SRAM with ECC
 */
static void RamCfg_Service_Single( ramCfg_SramId_t sramId )
{
    const ramCfg_FlagState_t singleFlag = RamCfg_Reg_Get_SingleFlag( sramId );

    if( RAMCFG_FLAG_ACTIVE == singleFlag )
    {
        const ramCfg_Address_t       address   = RamCfg_Reg_Get_SingleAddress( sramId );
        const ramCfg_FunctionState_t scrubbing = ramCfg_RuntimeData.Sram[ sramId ].Scrubbing;
        ramCfg_SingleErrorCallback_t * const callback = ramCfg_RuntimeData.SingleErrorCallback;

        RamCfg_Record_Error( sramId, address, RAMCFG_FUNCTION_INACTIVE );

        if( RAMCFG_FUNCTION_ACTIVE == scrubbing )
        {
            RamCfg_Scrub_Word( sramId, address );
        }
        else
        {
            /* Scrubbing is not active */
        }

        RamCfg_Reg_Clear_SingleFlag( sramId );

        if( RAMCFG_NULL_PTR != callback )
        {
            callback( sramId, address );
        }
        else
        {
            /* Single error callback is not configured */
        }
    }
    else
    {
        /* No single error is pending */
    }
}


/**
 * \brief Services the double error of an SRAM: records it, acknowledges the flag, calls the callback.
 *
 * \param sramId [in]: SRAM with ECC
 */
static void RamCfg_Service_Double( ramCfg_SramId_t sramId )
{
    const ramCfg_FlagState_t doubleFlag = RamCfg_Reg_Get_DoubleFlag( sramId );

    if( RAMCFG_FLAG_ACTIVE == doubleFlag )
    {
        const ramCfg_Address_t address = RamCfg_Reg_Get_DoubleAddress( sramId );
        ramCfg_DoubleErrorCallback_t * const callback = ramCfg_RuntimeData.DoubleErrorCallback;

        RamCfg_Record_Error( sramId, address, RAMCFG_FUNCTION_ACTIVE );

        RamCfg_Reg_Clear_DoubleFlag( sramId );

        if( RAMCFG_NULL_PTR != callback )
        {
            callback( sramId, address );
        }
        else
        {
            /* Double error callback is not configured */
        }
    }
    else
    {
        /* No double error is pending */
    }
}

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief RAMCFG interrupt handler.
 *
 * Services the errors of the SRAMs which are reported: the single error of the interrupt and NMI modes, the double
 * error of the interrupt mode (the double error of the NMI mode is serviced by the NMI handler).
 */
static void RamCfg_IsrHandler( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const ramCfg_ReportMode_t reportMode = ramCfg_RuntimeData.Sram[ sramId ].ReportMode;

        if( RAMCFG_REPORT_IRQ == reportMode )
        {
            RamCfg_Service_Single( sramId );
            RamCfg_Service_Double( sramId );
        }
        else if( RAMCFG_REPORT_NMI == reportMode )
        {
            RamCfg_Service_Single( sramId );
        }
        else
        {
            /* The errors of the SRAM are not reported */
        }
    }
}


/**
 * \brief NMI handler of the module (the double error of the SRAMs in the NMI mode).
 *
 * The NMI preempts everything, the double error callback shall be minimal. The NMI which does not come from the SRAMs
 * (no double error flag set) is not handled by the module.
 */
static void RamCfg_NmiHandler( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const ramCfg_ReportMode_t reportMode = ramCfg_RuntimeData.Sram[ sramId ].ReportMode;

        if( RAMCFG_REPORT_NMI == reportMode )
        {
            RamCfg_Service_Double( sramId );
        }
        else
        {
            /* The double error of the SRAM is not reported by the NMI */
        }
    }
}

/* ================================ TASKS =================================== */
