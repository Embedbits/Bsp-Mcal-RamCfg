/**
 * \author Mr.Nobody
 * \file ItTest_RamCfg.c
 * \ingroup RamCfg
 * \brief Integration tests of the SRAMs configuration controller (RAMCFG) module on target.
 *
 * RamCfg module runs on the MCU together with the real RCC and NVIC modules and the hardware. Tests verify
 * behavior which cannot be verified by unit tests (emulated registers): the unlock sequences of the ECC
 * enable bit and of the erase, the erase of an SRAM (the content is zero afterwards), the write
 * protection of the pages of SRAM2, the enable bits of the error reporting. The test cases use only the
 * public interface of the module, the backup SRAM is accessed through its address.
 *
 * Safety of the memory: the test firmware keeps its data (.data, .bss, mailbox, vector table) at the start of the
 * RAM (SRAM1) and its stack at the end of the RAM. The test cases never erase and never write-protect a memory which
 * the firmware uses: an SRAM is touched by the erase and the write protection only if it overlaps neither the
 * range of the firmware data (_ram_start ... _irqVectorTable_RAM_End) nor the top of the stack (the last
 * IT_RAMCFG_STACK_RESERVE bytes below _estack), otherwise the test case is ignored. The memories which pass the
 * check: the backup SRAM (the linker script does not place anything there) on every line, SRAM2 on the lines
 * STM32H523 ... STM32H5F5 (STM32H503: SRAM2 holds the stack), SRAM3 on the lines whose RAM ends in a later SRAM. SRAM1
 * is never erased (it holds the firmware data), SRAM4 and SRAM5 are never touched (the Rcc module has no clock
 * identification of them). The stack check of the module (it refuses the erase and the write protection of the SRAM
 * with the stack pointer) is verified by the unit tests, the integration tests do not call the erase for the SRAM with
 * the stack. Cases which are safe on every board: the initialization, the getters, the refusals of the module (SRAM
 * without ECC, parity, wait states), the switching of the reporting between none and the interrupt, the priority of the
 * interrupt, the erase and the ECC of the backup SRAM, the erase and the write protection of the SRAMs which pass the
 * check.
 *
 * The NMI reporting mode is never used by the tests (the integration test framework uses the NMI as a fault handler).
 * The ECC errors are not provoked (no error injection in the first version of the module).
 *
 * \note The backup SRAM is written by the CPU only with the backup domain write access enabled (PWR_DBPCR.DBP), which
 *       belongs to the Pwr module. The Pwr module is not available yet and the module under test does not touch the
 *       registers of other peripherals, so the test cases which need the CPU write access to the backup SRAM end with
 *       TEST_IGNORE_MESSAGE.
 * \note The tests were compiled only, they have not been run on a board.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "RamCfg_Port.h"                    /* Module under test              */
#include "Rcc_Port.h"                       /* Clock of the backup SRAM       */
#include "Stm32.h"                          /* SRAM sizes                     */
/* ============================= TYPEDEFS =================================== */

/** \brief Features of an SRAM of the line used by the tests (device header of the line) */
typedef struct
{
    uint32_t               Base; /**< First address of the memory (non-secure alias)        */
    uint32_t               Size; /**< Size of the memory in bytes                           */
    ramCfg_FunctionState_t Ecc;  /**< The RAMCFG controls the ECC of the memory             */
}   itRamCfg_Sram_t;


/** \brief SRAM which the erase test may touch and the clock of the memory in the Rcc module */
typedef struct
{
    ramCfg_SramId_t SramId;    /**< SRAM                                   */
    rcc_PeriphId_t  ClkPeriph; /**< Clock identification in the Rcc module */
}   itRamCfg_Erase_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 ItRamCfg_EraseCallback      ( ramCfg_SramId_t sramId );
static ramCfg_FlagState_t   ItRamCfg_Get_SramFree       ( ramCfg_SramId_t sramId );
static void                 ItRamCfg_Init_Default       ( void );
static void                 ItRamCfg_Init_Callback      ( void );
static void                 ItRamCfg_Erase_Sram         ( ramCfg_SramId_t sramId );
static void                 ItRamCfg_Check_Erased       ( ramCfg_SramId_t sramId );
static volatile uint32_t *  ItRamCfg_Get_Word           ( ramCfg_SramId_t sramId, uint32_t wordIdx );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Size of the part of the memory below the top of the RAM which is considered as the stack of the firmware */
#define IT_RAMCFG_STACK_RESERVE             ( 0x4000u )

/** Value written to the memory before the erase */
#define IT_RAMCFG_PATTERN                   ( 0xA5C3E71Bu )

/** Size of a word in bytes */
#define IT_RAMCFG_WORD_SIZE                 ( 4u )

/** Priority of the RAMCFG interrupt set by the tests */
#define IT_RAMCFG_IRQ_PRIO                  ( 2u )

/** Count of the calls of the task while the erase runs (the erase of the largest SRAM lasts below a millisecond) */
#define IT_RAMCFG_TASK_TIMEOUT_CNT          ( 10000000u )

/** Value of an output which the function must not change */
#define IT_RAMCFG_SENTINEL                  ( 0x77u )

/** Page of the write protection tested at the border of two write protection registers (page 31 and 32) */
#define IT_RAMCFG_BORDER_PAGE               ( 31u )

/** Count of the pages of SRAM2 (1 Kbyte pages) */
#define IT_RAMCFG_SRAM2_PAGES               ( SRAM2_SIZE / RAMCFG_PAGE_SIZE )

/** Count of the pages in one write protection register */
#define IT_RAMCFG_PAGES_PER_WPR             ( 32u )

/** Count of the pages of a write protection range of two pages (also the range across the border of two registers) */
#define IT_RAMCFG_RANGE_TWO_PAGES           ( 2u )

/** Divider of the count of the words of an SRAM which gives the index of a middle word */
#define IT_RAMCFG_MIDDLE_DIVIDER            ( 2u )

/* ========================== LOCAL VARIABLES =============================== */

/** Linker symbols: start of the RAM, end of the firmware data (the vector table is the last one), top of the stack */
extern uint32_t _ram_start;
extern uint32_t _irqVectorTable_RAM_End;
extern uint32_t _estack;

/** Features of the SRAMs of the line */
static const itRamCfg_Sram_t itRamCfg_SramList[ RAMCFG_SRAM_CNT ] =
{
    [ RAMCFG_SRAM_1 ]      = { .Base = SRAM1_BASE_NS,   .Size = SRAM1_SIZE,   .Ecc = RAMCFG_FUNCTION_INACTIVE },
    [ RAMCFG_SRAM_2 ]      = { .Base = SRAM2_BASE_NS,   .Size = SRAM2_SIZE,   .Ecc = RAMCFG_FUNCTION_ACTIVE   },
#if defined(RAMCFG_SRAM3)
#if defined(STM32H523xx) || \
    defined(STM32H533xx)
    [ RAMCFG_SRAM_3 ]      = { .Base = SRAM3_BASE_NS,   .Size = SRAM3_SIZE,   .Ecc = RAMCFG_FUNCTION_INACTIVE },
#else
    [ RAMCFG_SRAM_3 ]      = { .Base = SRAM3_BASE_NS,   .Size = SRAM3_SIZE,   .Ecc = RAMCFG_FUNCTION_ACTIVE   },
#endif
#endif
#if defined(RAMCFG_SRAM4)
    [ RAMCFG_SRAM_4 ]      = { .Base = SRAM4_BASE_NS,   .Size = SRAM4_SIZE,   .Ecc = RAMCFG_FUNCTION_INACTIVE },
#endif
#if defined(RAMCFG_SRAM5)
    [ RAMCFG_SRAM_5 ]      = { .Base = SRAM5_BASE_NS,   .Size = SRAM5_SIZE,   .Ecc = RAMCFG_FUNCTION_INACTIVE },
#endif
    [ RAMCFG_SRAM_BACKUP ] = { .Base = BKPSRAM_BASE_NS, .Size = BKPSRAM_SIZE, .Ecc = RAMCFG_FUNCTION_ACTIVE   },
};

/** SRAMs which the erase test touches when the firmware does not use them (SRAM1 holds the firmware data, SRAM4 and
 *  SRAM5 have no clock identification in the Rcc module and are not touched) */
static const itRamCfg_Erase_t itRamCfg_EraseList[] =
{
    { RAMCFG_SRAM_2, RCC_PERIPH_SRAM2 },
#if defined(RAMCFG_SRAM3)
    { RAMCFG_SRAM_3, RCC_PERIPH_SRAM3 },
#endif
};

/** Count and the last SRAM of the erase complete callback */
static volatile uint32_t    itRamCfg_EraseCnt;
static volatile ramCfg_SramId_t itRamCfg_EraseSram;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* Every test case starts after the system reset, the peripheral is in the reset state */
    itRamCfg_EraseCnt  = 0u;
    itRamCfg_EraseSram = RAMCFG_SRAM_UNUSED;
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* =============================== TESTS ==================================== */

/**
 * \brief   RamCfg_Get_ModuleVersion() returns the version of the module.
 *
 * \details Reads the version of the module.
 *
 * \par Expected results
 * - Version 0.1.0.
 */
void It_RamCfg_Get_ModuleVersion_ReturnsVersion( void )
{
    const ramCfg_ModuleVersion_t version = RamCfg_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 0u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 1u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}


/**
 * \brief   RamCfg_Init() with the default configuration reports the SRAMs with ECC by the interrupt.
 *
 * \details Initializes the module with the default configuration (check mode unchanged, no scrubbing, no protection)
 *          and reads the check mode and the reporting mode of every SRAM.
 *
 * \par Expected results
 * - RamCfg_Init() returns RAMCFG_REQUEST_OK, the check mode of every SRAM reads (the ECC is as the option bytes set
 *   it).
 * - The reporting mode is the interrupt for the SRAMs with ECC and none for the others.
 */
void It_RamCfg_Init_DefaultConfig_ReportsEccSrams( void )
{
    ItRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        ramCfg_CheckMode_t  checkMode  = RAMCFG_CHECK_CNT;
        ramCfg_ReportMode_t reportMode = RAMCFG_REPORT_CNT;

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_CheckMode( sramId, &checkMode ) );
        if( ( RAMCFG_CHECK_NONE == checkMode ) ||
            ( RAMCFG_CHECK_ECC  == checkMode )    )
        {
            /* The check mode is one the hardware can have */
        }
        else
        {
            TEST_FAIL_MESSAGE( "The check mode is neither none nor ECC" );
        }

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode( sramId, &reportMode ) );

        if( RAMCFG_FUNCTION_ACTIVE == itRamCfg_SramList[ sramId ].Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REPORT_IRQ, reportMode );
        }
        else
        {
            TEST_ASSERT_EQUAL( RAMCFG_REPORT_NONE, reportMode );
        }
    }
}


/**
 * \brief   RamCfg_Init() without any reporting leaves the errors unreported.
 *
 * \details Initializes the module with the reporting none for every SRAM and reads the reporting mode.
 *
 * \par Expected results
 * - RamCfg_Init() returns RAMCFG_REQUEST_OK, the reporting mode of every SRAM is none.
 */
void It_RamCfg_Init_NoReporting_ReportModeNone( void )
{
    ramCfg_Config_t config;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_DefaultConfig( &config ) );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_NONE;
    }

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Init( &config ) );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        ramCfg_ReportMode_t reportMode = RAMCFG_REPORT_CNT;

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode( sramId, &reportMode ) );
        TEST_ASSERT_EQUAL( RAMCFG_REPORT_NONE, reportMode );
    }
}


/**
 * \brief   RamCfg_Init() refuses a configuration which the hardware cannot do.
 *
 * \details Requests the parity check (the SRAMs of STM32H5 have the ECC only), the ECC of SRAM1 (no ECC control), the
 *          reporting by the interrupt of SRAM1 and the write protection of SRAM1 (only SRAM2 has it).
 *
 * \par Expected results
 * - RamCfg_Init() returns RAMCFG_REQUEST_ERROR in every case before any register is written, the module is not
 *   initialized afterwards (a getter returns RAMCFG_REQUEST_ERROR).
 */
void It_RamCfg_Init_UnsupportedConfig_ReturnsError( void )
{
    ramCfg_Config_t    config;
    ramCfg_CheckMode_t checkMode = RAMCFG_CHECK_CNT;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_DefaultConfig( &config ) );

    config.Sram[ RAMCFG_SRAM_2 ].CheckMode = RAMCFG_CHECK_PARITY;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Init( &config ) );
    config.Sram[ RAMCFG_SRAM_2 ].CheckMode = RAMCFG_CHECK_UNCHANGED;

    config.Sram[ RAMCFG_SRAM_1 ].CheckMode = RAMCFG_CHECK_ECC;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Init( &config ) );
    config.Sram[ RAMCFG_SRAM_1 ].CheckMode = RAMCFG_CHECK_UNCHANGED;

    config.Sram[ RAMCFG_SRAM_1 ].ReportMode = RAMCFG_REPORT_IRQ;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Init( &config ) );
    config.Sram[ RAMCFG_SRAM_1 ].ReportMode = RAMCFG_REPORT_NONE;

    config.Sram[ RAMCFG_SRAM_1 ].ProtectionFirstPage = 0u;
    config.Sram[ RAMCFG_SRAM_1 ].ProtectionPageCnt   = 1u;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Init( &config ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_CheckMode( RAMCFG_SRAM_2, &checkMode ) );
}


/**
 * \brief   The error statistics of the SRAMs with ECC are clear after the initialization.
 *
 * \details Initializes the module with the default configuration, reads the statistics and the failing address of every
 *          SRAM, clears the statistics.
 *
 * \par Expected results
 * - SRAMs with ECC: both counters zero, the last SRAM and address "unused", the clear function returns
 *   RAMCFG_REQUEST_OK.
 * - SRAMs without ECC: the functions return RAMCFG_REQUEST_ERROR.
 */
void It_RamCfg_Get_ErrorStatistics_AfterInit_NoErrorRecorded( void )
{
    ItRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        ramCfg_ErrorStatistics_t statistics;
        ramCfg_Address_t         address = RAMCFG_ADDRESS_UNUSED;

        if( RAMCFG_FUNCTION_ACTIVE == itRamCfg_SramList[ sramId ].Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( sramId, &statistics ) );
            TEST_ASSERT_EQUAL_UINT32( 0u, statistics.SingleErrorCnt );
            TEST_ASSERT_EQUAL_UINT32( 0u, statistics.DoubleErrorCnt );
            TEST_ASSERT_EQUAL( RAMCFG_SRAM_UNUSED, statistics.LastErrorSram );
            TEST_ASSERT_EQUAL_HEX32( RAMCFG_ADDRESS_UNUSED, statistics.LastErrorAddress );

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_FailingAddress( sramId, &address ) );
            TEST_ASSERT_EQUAL_HEX32( RAMCFG_ADDRESS_UNUSED, address );

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ErrorStatisticsClear( sramId ) );
        }
        else
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ErrorStatistics( sramId, &statistics ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_FailingAddress( sramId, &address ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ErrorStatisticsClear( sramId ) );
        }
    }
}


/**
 * \brief   The reporting of SRAM2 switches between none and the interrupt.
 *
 * \details Initializes the module with the default configuration, then sets the reporting none, the interrupt and none
 *          again for SRAM2 and reads it back (the NMI mode is never used, its redirection cannot be undone).
 *
 * \par Expected results
 * - RamCfg_Set_ReportMode() returns RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode() returns the mode which was set.
 */
void It_RamCfg_Set_ReportMode_Sram2_SwitchesBetweenNoneAndInterrupt( void )
{
    const ramCfg_ReportMode_t modeList[] = { RAMCFG_REPORT_NONE, RAMCFG_REPORT_IRQ, RAMCFG_REPORT_NONE };

    ItRamCfg_Init_Default();

    for( uint32_t modeIdx = 0u; ( sizeof( modeList ) / sizeof( modeList[ 0u ] ) ) > modeIdx; modeIdx ++ )
    {
        ramCfg_ReportMode_t reportMode = RAMCFG_REPORT_CNT;

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, modeList[ modeIdx ] ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode( RAMCFG_SRAM_2, &reportMode ) );
        TEST_ASSERT_EQUAL( modeList[ modeIdx ], reportMode );
    }
}


/**
 * \brief   SRAM1 has no ECC control, the module refuses the requests which need it.
 *
 * \details Initializes the module with the default configuration and requests the ECC, the reporting by the interrupt,
 *          the scrubbing and the write protection of SRAM1, reads the check mode.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR for the requests, no register is written, the check mode of SRAM1 reads none.
 */
void It_RamCfg_Set_CheckMode_Sram1_RefusesEccRequests( void )
{
    ramCfg_CheckMode_t checkMode = RAMCFG_CHECK_CNT;

    ItRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( RAMCFG_SRAM_1, RAMCFG_CHECK_ECC ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_1, RAMCFG_REPORT_IRQ ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Scrubbing( RAMCFG_SRAM_1, RAMCFG_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_1, 0u, 1u ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_CheckMode( RAMCFG_SRAM_1, &checkMode ) );
    TEST_ASSERT_EQUAL( RAMCFG_CHECK_NONE, checkMode );
}


/**
 * \brief   The scrubbing of the SRAMs with ECC can be activated and deactivated.
 *
 * \details Initializes the module with the default configuration, activates and deactivates the scrubbing of every SRAM
 *          (a setting of the module, no register is written).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK for the SRAMs with ECC, RAMCFG_REQUEST_ERROR for the activation of an SRAM without ECC.
 */
void It_RamCfg_Set_Scrubbing_Sram_AcceptsEccSramsOnly( void )
{
    ItRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( sramId, RAMCFG_FUNCTION_INACTIVE ) );

        if( RAMCFG_FUNCTION_ACTIVE == itRamCfg_SramList[ sramId ].Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( sramId, RAMCFG_FUNCTION_ACTIVE ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( sramId, RAMCFG_FUNCTION_INACTIVE ) );
        }
        else
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Scrubbing( sramId, RAMCFG_FUNCTION_ACTIVE ) );
        }
    }
}


/**
 * \brief   The ECC of the backup SRAM is switched with the unlock sequence of the hardware.
 *
 * \details The backup SRAM is not used by the firmware. Erases it (the ECC code words are valid afterwards), enables
 *          the ECC, disables it (the keys are written to the key register) and enables it again, reading the check mode
 *          after every step.
 *
 * \par Expected results
 * - The functions return RAMCFG_REQUEST_OK, the check mode reads ECC, none and ECC (the read-back of the enable bit
 *   confirms the unlock sequence of the hardware).
 */
void It_RamCfg_Set_CheckMode_BackupSram_SwitchesEcc( void )
{
    const ramCfg_CheckMode_t modeList[] = { RAMCFG_CHECK_ECC, RAMCFG_CHECK_NONE, RAMCFG_CHECK_ECC };

    ItRamCfg_Init_Default();
    ItRamCfg_Erase_Sram( RAMCFG_SRAM_BACKUP );

    for( uint32_t modeIdx = 0u; ( sizeof( modeList ) / sizeof( modeList[ 0u ] ) ) > modeIdx; modeIdx ++ )
    {
        ramCfg_CheckMode_t checkMode = RAMCFG_CHECK_CNT;

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_CheckMode( RAMCFG_SRAM_BACKUP, modeList[ modeIdx ] ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_CheckMode( RAMCFG_SRAM_BACKUP, &checkMode ) );
        TEST_ASSERT_EQUAL( modeList[ modeIdx ], checkMode );
    }
}


/**
 * \brief   RamCfg_Set_Erase() zeroes the backup SRAM (the check with a pattern is not available yet).
 *
 * \details The backup SRAM is not used by the firmware. Reads the state of the erase, erases the backup SRAM without an
 *          erase complete callback (the function waits for the end of the erase), reads the content and the state of
 *          the erase, erases it again. The pattern which the erase has to remove is not written: the CPU write access
 *          to the backup SRAM needs the backup domain write access of the Pwr module, which is not available yet, so
 *          the test case ends with TEST_IGNORE_MESSAGE.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the state is "idle" before and "done" after the erase, the first, a middle and the last word
 *   are zero, the second erase is accepted.
 * - The test case is reported as ignored until the Pwr module provides the backup domain write access.
 */
void It_RamCfg_Set_Erase_BackupSram_ZeroesContent( void )
{
    ramCfg_EraseState_t eraseState = RAMCFG_ERASE_CNT;

    ItRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_BACKUP, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_IDLE, eraseState );

    ItRamCfg_Erase_Sram( RAMCFG_SRAM_BACKUP );
    ItRamCfg_Check_Erased( RAMCFG_SRAM_BACKUP );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_BACKUP, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_DONE, eraseState );

    ItRamCfg_Erase_Sram( RAMCFG_SRAM_BACKUP );

    TEST_IGNORE_MESSAGE( "Backup SRAM write access needs the Pwr module, which is not available yet" );
}


/**
 * \brief   RamCfg_Set_Erase() with an erase complete callback returns at once and RamCfg_Task() reports the end.
 *
 * \details Initializes the module with the callback, erases the backup SRAM (the firmware does not use it), calls the
 *          task until the callback was called, calls the task again. The pattern which the erase has to remove is not
 *          written: the CPU write access to the backup SRAM needs the backup domain write access of the Pwr module,
 *          which is not available yet, so the test case ends with TEST_IGNORE_MESSAGE.
 *
 * \par Expected results
 * - RamCfg_Set_Erase() returns RAMCFG_REQUEST_OK, the callback is called once with the backup SRAM, the content is
 *   zero, the state is "done", the task does not report the end a second time.
 * - The test case is reported as ignored until the Pwr module provides the backup domain write access.
 */
void It_RamCfg_Set_Erase_WithCallback_TaskReportsEnd( void )
{
    ramCfg_EraseState_t eraseState = RAMCFG_ERASE_CNT;

    ItRamCfg_Init_Callback();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Erase( RAMCFG_SRAM_BACKUP ) );

    for( uint32_t taskIdx = 0u;
         ( IT_RAMCFG_TASK_TIMEOUT_CNT > taskIdx ) &&
         ( 0u == itRamCfg_EraseCnt );
         taskIdx ++ )
    {
        RamCfg_Task();
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, itRamCfg_EraseCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_BACKUP, itRamCfg_EraseSram );

    RamCfg_Task();
    RamCfg_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, itRamCfg_EraseCnt );
    ItRamCfg_Check_Erased( RAMCFG_SRAM_BACKUP );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_BACKUP, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_DONE, eraseState );

    TEST_IGNORE_MESSAGE( "Backup SRAM write access needs the Pwr module, which is not available yet" );
}


/**
 * \brief   RamCfg_Set_Erase() zeroes every SRAM which the firmware does not use.
 *
 * \details For SRAM2 and SRAM3 (where the line has it) which overlap neither the firmware data nor the top of the stack
 *          (SRAM2 on STM32H523 ... STM32H5F5, SRAM3 where the RAM ends behind it) activates the clock of the memory,
 *          writes a pattern to the first and the last word, erases the SRAM, reads the content. The test case is
 *          ignored on a line where both SRAMs are used by the firmware (STM32H503 has no SRAM3 and uses SRAM2 for the
 *          stack).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK for every erase, the first, a middle and the last word are zero.
 */
void It_RamCfg_Set_Erase_FreeSrams_ZeroesContent( void )
{
    uint32_t erasedCnt = 0u;

    ItRamCfg_Init_Default();

    for( uint32_t listIdx = 0u;
         ( sizeof( itRamCfg_EraseList ) / sizeof( itRamCfg_EraseList[ 0u ] ) ) > listIdx;
         listIdx ++ )
    {
        const ramCfg_SramId_t    sramId   = itRamCfg_EraseList[ listIdx ].SramId;
        const ramCfg_FlagState_t freeFlag = ItRamCfg_Get_SramFree( sramId );

        if( RAMCFG_FLAG_ACTIVE == freeFlag )
        {
            TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_PeriphActive( itRamCfg_EraseList[ listIdx ].ClkPeriph ) );

            *ItRamCfg_Get_Word( sramId, 0u ) = IT_RAMCFG_PATTERN;
            *ItRamCfg_Get_Word( sramId,
                                ( itRamCfg_SramList[ sramId ].Size / IT_RAMCFG_WORD_SIZE ) - 1u ) = IT_RAMCFG_PATTERN;

            ItRamCfg_Erase_Sram( sramId );
            ItRamCfg_Check_Erased( sramId );

            erasedCnt ++;
        }
        else
        {
            /* The firmware uses the memory, it is not erased */
        }
    }

    if( 0u == erasedCnt )
    {
        TEST_IGNORE_MESSAGE( "Every SRAM of the line is used by the test firmware (data or stack)" );
    }
    else
    {
        /* The SRAMs which the firmware does not use were erased */
    }
}


/**
 * \brief   RamCfg_Set_WriteProtection() protects pages of SRAM2 when the firmware does not use SRAM2.
 *
 * \details Protects the first page, the last page, two pages across the border of two write protection registers (lines
 *          with more than 32 pages) and all pages of SRAM2 one after another and reads the state of the pages (no page
 *          is written). The protection is cleared by the system reset between the test cases. The test case is ignored
 *          where the firmware uses SRAM2 (STM32H503: the stack).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, RamCfg_Get_WriteProtection() returns "active" for the protected pages and "inactive" for a range
 *   which contains an unprotected page.
 */
void It_RamCfg_Set_WriteProtection_FreeSram2_ProtectsPages( void )
{
    const ramCfg_FlagState_t freeFlag = ItRamCfg_Get_SramFree( RAMCFG_SRAM_2 );

    if( RAMCFG_FLAG_ACTIVE == freeFlag )
    {
        ramCfg_FunctionState_t protection = (ramCfg_FunctionState_t)IT_RAMCFG_SENTINEL;

        ItRamCfg_Init_Default();

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, 1u, &protection ) );
        TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, protection );

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, 1u ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, 1u, &protection ) );
        TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_ACTIVE, protection );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, IT_RAMCFG_RANGE_TWO_PAGES, &protection ) );
        TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, protection );

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, IT_RAMCFG_SRAM2_PAGES - 1u, 1u ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, IT_RAMCFG_SRAM2_PAGES - 1u, 1u, &protection ) );
        TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_ACTIVE, protection );

#if ( IT_RAMCFG_SRAM2_PAGES > IT_RAMCFG_PAGES_PER_WPR )
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                       IT_RAMCFG_BORDER_PAGE,
                                                       IT_RAMCFG_RANGE_TWO_PAGES ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Get_WriteProtection( RAMCFG_SRAM_2,
                                                       IT_RAMCFG_BORDER_PAGE,
                                                       IT_RAMCFG_RANGE_TWO_PAGES,
                                                       &protection ) );
        TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_ACTIVE, protection );
#endif

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, IT_RAMCFG_SRAM2_PAGES ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, IT_RAMCFG_SRAM2_PAGES, &protection ) );
        TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_ACTIVE, protection );
    }
    else
    {
        TEST_IGNORE_MESSAGE( "SRAM2 is used by the test firmware (data or stack)" );
    }
}


/**
 * \brief   RamCfg_Set_WriteProtection() refuses a range without a page and a range beyond the pages.
 *
 * \details Requests an empty range, a range which starts beyond the pages and one which ends beyond the pages for SRAM2
 *          (the requests are refused before any register is written, so the test case is safe on every line).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the pages are not protected afterwards.
 */
void It_RamCfg_Set_WriteProtection_InvalidRange_ReturnsError( void )
{
    ramCfg_FunctionState_t protection = RAMCFG_FUNCTION_ACTIVE;

    ItRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, RAMCFG_PAGE_CNT_UNUSED ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, IT_RAMCFG_SRAM2_PAGES, 1u ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                   IT_RAMCFG_SRAM2_PAGES - 1u,
                                                   IT_RAMCFG_RANGE_TWO_PAGES ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, 1u, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, protection );
}


/**
 * \brief   The SRAMs of STM32H5 have no wait state setting.
 *
 * \details Sets and reads the wait states of every SRAM (the output holds a sentinel).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the output is not changed.
 */
void It_RamCfg_WaitStates_NotAvailable_ReturnError( void )
{
    ramCfg_WaitStates_t waitStates = IT_RAMCFG_SENTINEL;

    ItRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WaitStates( sramId, 1u ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WaitStates( sramId, &waitStates ) );
    }

    TEST_ASSERT_EQUAL_UINT8( IT_RAMCFG_SENTINEL, waitStates );
}


/**
 * \brief   The priority of the RAMCFG interrupt is passed to NVIC.
 *
 * \details Initializes the module, sets the priority and reads it back.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK for both functions, the priority reads as it was set.
 */
void It_RamCfg_Set_IrqPriority_Initialized_ReadsBack( void )
{
    ramCfg_IrqPrio_t irqPrio = IT_RAMCFG_SENTINEL;

    ItRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_IrqPriority( IT_RAMCFG_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( IT_RAMCFG_IRQ_PRIO, irqPrio );
}


/**
 * \brief   RamCfg_Deinit() stops the module and it can be initialized again.
 *
 * \details Initializes the module, deinitializes it, reads the check mode, initializes it again.
 *
 * \par Expected results
 * - RamCfg_Deinit() returns RAMCFG_REQUEST_OK, afterwards a getter returns RAMCFG_REQUEST_ERROR (module not
 *   initialized), the second initialization returns RAMCFG_REQUEST_OK.
 */
void It_RamCfg_Deinit_Initialized_ModuleStopsAndRestarts( void )
{
    ramCfg_CheckMode_t checkMode = RAMCFG_CHECK_CNT;

    ItRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Deinit() );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_CheckMode( RAMCFG_SRAM_2, &checkMode ) );

    ItRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_CheckMode( RAMCFG_SRAM_2, &checkMode ) );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/** Erase complete callback - counts the calls and remembers the SRAM */
static void ItRamCfg_EraseCallback( ramCfg_SramId_t sramId )
{
    itRamCfg_EraseCnt ++;
    itRamCfg_EraseSram = sramId;
}


/**
 * Returns whether the firmware does not use the memory: the SRAM overlaps neither the range of the firmware data
 * (start of the RAM up to the end of the vector table in RAM) nor the top of the stack (IT_RAMCFG_STACK_RESERVE bytes
 * below the end of the RAM).
 */
static ramCfg_FlagState_t ItRamCfg_Get_SramFree( ramCfg_SramId_t sramId )
{
    ramCfg_FlagState_t retState   = RAMCFG_FLAG_ACTIVE;
    const uint32_t     sramStart  = itRamCfg_SramList[ sramId ].Base;
    const uint32_t     sramEnd    = ( sramStart + itRamCfg_SramList[ sramId ].Size );
    const uint32_t     dataStart  = (uint32_t)&_ram_start;
    const uint32_t     dataEnd    = (uint32_t)&_irqVectorTable_RAM_End;
    const uint32_t     stackEnd   = (uint32_t)&_estack;
    const uint32_t     stackStart = ( stackEnd - IT_RAMCFG_STACK_RESERVE );

    if( ( sramStart < dataEnd   ) &&
        ( sramEnd   > dataStart )    )
    {
        /* The firmware data are in the memory */
        retState = RAMCFG_FLAG_INACTIVE;
    }
    else
    {
        /* No firmware data in the memory */
    }

    if( ( sramStart < stackEnd   ) &&
        ( sramEnd   > stackStart )    )
    {
        /* The stack of the firmware is in the memory */
        retState = RAMCFG_FLAG_INACTIVE;
    }
    else
    {
        /* No stack in the memory */
    }

    return ( retState );
}


/** Initializes the module with the default configuration and expects success */
static void ItRamCfg_Init_Default( void )
{
    ramCfg_Config_t config;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Init( &config ) );
}


/** Initializes the module with the default configuration and the erase complete callback, expects success */
static void ItRamCfg_Init_Callback( void )
{
    ramCfg_Config_t config;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_DefaultConfig( &config ) );

    config.EraseCompleteCallback = ItRamCfg_EraseCallback;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Init( &config ) );
}


/** Erases an SRAM without a callback (the function waits for the end of the erase) and expects success */
static void ItRamCfg_Erase_Sram( ramCfg_SramId_t sramId )
{
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Erase( sramId ) );
}


/** Returns a word of an SRAM */
static volatile uint32_t * ItRamCfg_Get_Word( ramCfg_SramId_t sramId, uint32_t wordIdx )
{
    return ( (volatile uint32_t *)( itRamCfg_SramList[ sramId ].Base + ( wordIdx * IT_RAMCFG_WORD_SIZE ) ) );
}


/** Checks that the first word, a middle word and the last word of an SRAM are zero */
static void ItRamCfg_Check_Erased( ramCfg_SramId_t sramId )
{
    const uint32_t wordCnt = ( itRamCfg_SramList[ sramId ].Size / IT_RAMCFG_WORD_SIZE );

    TEST_ASSERT_EQUAL_HEX32( 0u, *ItRamCfg_Get_Word( sramId, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, *ItRamCfg_Get_Word( sramId, wordCnt / IT_RAMCFG_MIDDLE_DIVIDER ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, *ItRamCfg_Get_Word( sramId, wordCnt - 1u ) );
}
