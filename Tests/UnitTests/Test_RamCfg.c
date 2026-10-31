/**
 * \author Mr.Nobody
 * \file Test_RamCfg.c
 * \ingroup RamCfg
 * \brief Unit tests of the SRAMs configuration controller (RAMCFG) module.
 *
 * RamCfg.c and RamCfg_Reg.c are compiled unchanged, RAMCFG registers are emulated by RegMem, the
 * RCC (peripheral clocks) and NVIC (interrupt, NMI vector) modules are replaced by stubs of CMock which
 * record the calls (a call without a stub fails the test), the main and process stack pointers are
 * emulated by CmsisHost. The memory of the SRAMs is the SRAM region of RegMem (0x20000000), the backup SRAM
 * (0x40036400) lies in the peripheral region of RegMem. The facts of the device lines (the SRAMs, their sizes,
 * the ECC, the pages of the write protection) are restated in this file from the reference manuals
 * (RM0481, RM0492, RM0517) by the device macros of the line, the register offsets and the bit masks are the
 * numbers of the reference manuals - they do not come from the module or from the CMSIS device header.
 *
 * \note Limitations of the register emulation (RegMem registers are plain memory):
 *       - The behavior of the hardware is emulated by one HW model of RegMem (UtRamCfg_HwModel), which runs after
 *         every store: the unlock sequence of the ECC enable bit (keys 0xAE, 0x75) and of the erase bit (keys
 *         0xCA, 0x53, a wrong or missing key makes the write ignored), the flags SEDC and DED cleared by writing 1
 *         to ICR (the bits of ICR read as the flags, the model keeps them in ICR together with a reserved marker
 *         bit so that a store of the module is told from the mirror; a read-modify-write of ICR with a flag already
 *         set cannot be told from the mirror by the model, it is caught by the tests of the register component, which
 *         preset ICR and check the written value), the erase (SRAMBUSY set, SRAMER cleared by the hardware, the
 *         first word of the SRAM zeroed), the registers which do not keep the written value (read-back
 *         mismatch). The model is evaluated after a store, not while the module waits: the end of an erase
 *         is emulated either at once or never (the test releases a held erase by clearing the bits).
 *       - The unlock sequences, the count of the stores of the interrupt handler and the mirror of ICR need the
 *         synchronous mode of the model (a store is evaluated at once, x86-64 Windows and Linux): where RegMem runs
 *         the model in a thread, the test cases which start the model are ignored (UtRamCfg_Start_Model).
 *       - The bits set only by software and cleared only by a reset (write protection, NMI redirection) are plain
 *         bits: the tests check that the module does not write them back, not that the hardware keeps them.
 *       - The write-back of the scrubbing is detected in the backup SRAM (peripheral region of RegMem) as an
 *         additional store seen by the model; for SRAM1 ... SRAM5 the data are plain memory and the write-back of
 *         the same value is not observable.
 *       - The corrected value of a read of a cell with a single error cannot be emulated, a single error is a flag
 *         and an address in the registers.
 *       - Every test case is executed in its own process (CTest), the runtime data of the module start from the
 *         reset state in every test.
 *
 * \note    Device errata bug AB#1838 (ES0561 2.2.20 "SRAM ECC error flags and addresses are updated only if interrupt
 *          is enabled"): the failing address and the flags are stored only while the address latch and the error
 *          interrupt are enabled (documentation erratum of STM32H503); the tests of the reporting prove that the module
 *          enables the address latch together with the error interrupt (the address registers are read only for the
 *          reported SRAMs).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "string.h"                         /* memset                         */
#include "stddef.h"                         /* offsetof                       */
#include "RegMem.h"                         /* Register memory emulation      */
#include "CmsisHost.h"                      /* Core intrinsics emulation      */
#include "RamCfg_Port.h"                    /* Module under test              */
#include "RamCfg_Reg.h"                     /* Register access of the module  */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "Stm32.h"                          /* RAMCFG registers definition    */
/* ============================= TYPEDEFS =================================== */

/** \brief Features of an SRAM of the device line (restated from the reference manual) */
typedef struct
{
    uint32_t               Base;      /**< First address of the memory (non-secure alias)               */
    uint32_t               Size;      /**< Size of the memory in bytes                                  */
    ramCfg_FunctionState_t Ecc;       /**< The RAMCFG controls the ECC of the memory                    */
    ramCfg_PageCnt_t       WpPages;   /**< Count of the pages of the write protection (0 - none)        */
    ramCfg_FunctionState_t HasClk;    /**< The memory has a clock enable bit in the Rcc module          */
    rcc_PeriphId_t         ClkPeriph; /**< Rcc identification of the clock of the memory                */
    uint32_t               RegOffset; /**< Offset of the registers of the memory in the RAMCFG          */
}   utRamCfg_Sram_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  UtRamCfg_NvicSetHandlerStub     ( nvic_PeriphIrqList_t irqId,
                                                              const nvic_IsrCallback_t irqHandler,
                                                              int callCnt );
static nvic_RequestState_t  UtRamCfg_NvicSetPrioStub        ( nvic_PeriphIrqList_t irqId,
                                                              nvic_IrqPrio_t irqPrio,
                                                              int callCnt );
static nvic_RequestState_t  UtRamCfg_NvicGetPrioStub        ( nvic_PeriphIrqList_t irqId,
                                                              nvic_IrqPrio_t * irqPrio,
                                                              int callCnt );
static nvic_RequestState_t  UtRamCfg_NvicActiveStub         ( nvic_PeriphIrqList_t irqId, int callCnt );
static nvic_RequestState_t  UtRamCfg_NvicInactiveStub       ( nvic_PeriphIrqList_t irqId, int callCnt );
static nvic_RequestState_t  UtRamCfg_NvicSetCoreStub        ( nvic_CoreIrqList_t irqId,
                                                              const nvic_IsrCallback_t irqHandler,
                                                              int callCnt );
static nvic_RequestState_t  UtRamCfg_NvicGetCoreStub        ( nvic_CoreIrqList_t irqId,
                                                              nvic_IsrCallback_t * irqHandler,
                                                              int callCnt );
static rcc_RequestState_t   UtRamCfg_RccActiveStub          ( rcc_PeriphId_t periphId, int callCnt );

static void                 UtRamCfg_PrevNmiHandler         ( void );
static void                 UtRamCfg_SingleCallback         ( ramCfg_SramId_t sramId, ramCfg_Address_t address );
static void                 UtRamCfg_DoubleCallback         ( ramCfg_SramId_t sramId, ramCfg_Address_t address );
static void                 UtRamCfg_EraseCallback          ( ramCfg_SramId_t sramId );

static const utRamCfg_Sram_t * UtRamCfg_Get_Expect          ( ramCfg_SramId_t sramId );
static RAMCFG_TypeDef *     UtRamCfg_Get_Inst               ( ramCfg_SramId_t sramId );
static volatile uint32_t *  UtRamCfg_Get_RegPtr             ( ramCfg_SramId_t sramId, uint32_t regOffset );
static ramCfg_Config_t      UtRamCfg_Get_Config             ( void );
static void                 UtRamCfg_Init_Config            ( const ramCfg_Config_t * const config );
static void                 UtRamCfg_Init_Default           ( void );
static void                 UtRamCfg_Init_Sram              ( ramCfg_SramId_t sramId,
                                                              ramCfg_CheckMode_t checkMode,
                                                              ramCfg_ReportMode_t reportMode,
                                                              ramCfg_FunctionState_t scrubbing );
static void                 UtRamCfg_Init_Callbacks         ( ramCfg_SramId_t sramId, ramCfg_ReportMode_t reportMode );
static void                 UtRamCfg_Fill_Block             ( uint32_t seed );
static void                 UtRamCfg_Snapshot               ( uint32_t * const block );
static void                 UtRamCfg_Check_BlockUnchanged   ( const uint32_t * const block );
static void                 UtRamCfg_Check_OnlyWord         ( const volatile uint32_t * const word, uint32_t value );
static void                 UtRamCfg_Check_NotInitialized   ( void );
static void                 UtRamCfg_Check_NoOtherModuleCalls( void );
static void                 UtRamCfg_Check_InitRejected     ( const ramCfg_Config_t * const config );
static void                 UtRamCfg_Check_InitFailed       ( const ramCfg_Config_t * const config );
static void                 UtRamCfg_Start_Model            ( uint32_t modelFlags );
static void                 UtRamCfg_Raise_Single           ( ramCfg_SramId_t sramId, uint32_t address );
static void                 UtRamCfg_Raise_Double           ( ramCfg_SramId_t sramId, uint32_t address );
static uint32_t             UtRamCfg_Run_SingleIsr          ( ramCfg_SramId_t sramId, uint32_t address );
static void                 UtRamCfg_Finish_Erase           ( ramCfg_SramId_t sramId );
static uint32_t             UtRamCfg_Get_PageMask           ( uint32_t wprIdx, uint32_t firstPage, uint32_t pageCnt );
static uint32_t             UtRamCfg_Get_WprReg             ( ramCfg_SramId_t sramId, uint32_t wprIdx );
static void                 UtRamCfg_Check_Wpr              ( ramCfg_SramId_t sramId,
                                                              uint32_t firstPage,
                                                              uint32_t pageCnt );
static void                 UtRamCfg_Clear_Wpr              ( ramCfg_SramId_t sramId );

static void                 UtRamCfg_HwModel                ( void );
static uint32_t             UtRamCfg_Model_Key              ( volatile uint32_t * const key,
                                                              volatile uint32_t * const prev,
                                                              volatile uint32_t * const state,
                                                              uint32_t locked,
                                                              uint32_t key1,
                                                              uint32_t key2 );
static void                 UtRamCfg_Model_Ecc              ( ramCfg_SramId_t sramId, RAMCFG_TypeDef * const inst );
static void                 UtRamCfg_Model_Erase            ( ramCfg_SramId_t sramId, RAMCFG_TypeDef * const inst );
static void                 UtRamCfg_Model_Flags            ( ramCfg_SramId_t sramId, RAMCFG_TypeDef * const inst );
static void                 UtRamCfg_Model_NotKept          ( ramCfg_SramId_t sramId, RAMCFG_TypeDef * const inst );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Expected version of the module */
#define UT_RAMCFG_VERSION_MAJOR             ( 0u )
#define UT_RAMCFG_VERSION_MINOR             ( 1u )
#define UT_RAMCFG_VERSION_PATCH             ( 0u )

/** Address of the first RAMCFG register (RAMCFG_M1CR, reference manuals: memory map) */
#define UT_RAMCFG_BLOCK_ADDR                ( 0x40026000u )

/** Size of the register block of the six memories (0x40 bytes per memory, the backup SRAM at 0x100, SRAM5 at 0x180) */
#define UT_RAMCFG_BLOCK_SIZE                ( 0x1C0u )

/** Size of a register in bytes */
#define UT_RAMCFG_WORD_SIZE                 ( 4u )

/** Count of the registers of the register block */
#define UT_RAMCFG_BLOCK_WORDS               ( UT_RAMCFG_BLOCK_SIZE / UT_RAMCFG_WORD_SIZE )

/** First address of the backup SRAM (all lines) */
#define UT_RAMCFG_BKP_BASE                  ( 0x40036400u )

/** Offset between the secure and the non-secure alias of the memories */
#define UT_RAMCFG_SECURE_ALIAS              ( 0x10000000u )

/** Offsets of the registers of a memory (RAMCFG_MxCR ... RAMCFG_MxERKEYR) */
#define UT_RAMCFG_OFFSET_CR                 ( 0x00u )
#define UT_RAMCFG_OFFSET_IER                ( 0x04u )
#define UT_RAMCFG_OFFSET_ISR                ( 0x08u )
#define UT_RAMCFG_OFFSET_SEAR               ( 0x0Cu )
#define UT_RAMCFG_OFFSET_DEAR               ( 0x10u )
#define UT_RAMCFG_OFFSET_ICR                ( 0x14u )
#define UT_RAMCFG_OFFSET_WPR1               ( 0x18u )
#define UT_RAMCFG_OFFSET_WPR2               ( 0x1Cu )
#define UT_RAMCFG_OFFSET_ECCKEY             ( 0x24u )
#define UT_RAMCFG_OFFSET_ERKEYR             ( 0x28u )

/** Bits of RAMCFG_MxCR */
#define UT_RAMCFG_CR_ECCE                   ( 0x00000001u )
#define UT_RAMCFG_CR_ALE                    ( 0x00000010u )
#define UT_RAMCFG_CR_SRAMER                 ( 0x00000100u )

/** Bits of RAMCFG_MxIER */
#define UT_RAMCFG_IER_SEIE                  ( 0x00000001u )
#define UT_RAMCFG_IER_DEIE                  ( 0x00000002u )
#define UT_RAMCFG_IER_ECCNMI                ( 0x00000008u )

/** Reporting by the interrupt: single error and double error interrupts */
#define UT_RAMCFG_IER_IRQ                   ( UT_RAMCFG_IER_SEIE | UT_RAMCFG_IER_DEIE )

/** Reporting by the NMI: single error interrupt and double error NMI */
#define UT_RAMCFG_IER_NMI                   ( UT_RAMCFG_IER_SEIE | UT_RAMCFG_IER_ECCNMI )

/** Bits of RAMCFG_MxISR */
#define UT_RAMCFG_ISR_SEDC                  ( 0x00000001u )
#define UT_RAMCFG_ISR_DED                   ( 0x00000002u )
#define UT_RAMCFG_ISR_SRAMBUSY              ( 0x00000100u )

/** Bits of RAMCFG_MxICR */
#define UT_RAMCFG_ICR_CSEDC                 ( 0x00000001u )
#define UT_RAMCFG_ICR_CDED                  ( 0x00000002u )

/** Keys of the unlock sequences */
#define UT_RAMCFG_ECC_KEY_1                 ( 0xAEu )
#define UT_RAMCFG_ECC_KEY_2                 ( 0x75u )
#define UT_RAMCFG_ERASE_KEY_1               ( 0xCAu )
#define UT_RAMCFG_ERASE_KEY_2               ( 0x53u )

/** Count of the pages in a write protection register */
#define UT_RAMCFG_PAGES_PER_WPR             ( 32u )

/** Count of the write protection registers of the line */
#define UT_RAMCFG_WPR_CNT                   ( ( UT_RAMCFG_SRAM2_PAGES + UT_RAMCFG_PAGES_PER_WPR - 1u ) / \
                                              UT_RAMCFG_PAGES_PER_WPR )

/** Size of a page of the write protection */
#define UT_RAMCFG_PAGE_SIZE                 ( 1024u )

/** First address of SRAM1 (all lines) */
#define UT_RAMCFG_SRAM1_BASE                ( 0x20000000u )

/** Offsets of the register sets of the memories in the RAMCFG register block (reference manuals: memory map) */
#define UT_RAMCFG_REGOFF_SRAM1              ( 0x000u )
#define UT_RAMCFG_REGOFF_SRAM2              ( 0x040u )
#define UT_RAMCFG_REGOFF_SRAM3              ( 0x080u )
#define UT_RAMCFG_REGOFF_BKP                ( 0x100u )
#define UT_RAMCFG_REGOFF_SRAM4              ( 0x140u )
#define UT_RAMCFG_REGOFF_SRAM5              ( 0x180u )

/** Count of the pages covered by the first two and by the first three write protection registers */
#define UT_RAMCFG_PAGES_WPR_1_2             ( 2u * UT_RAMCFG_PAGES_PER_WPR )
#define UT_RAMCFG_PAGES_WPR_1_3             ( 3u * UT_RAMCFG_PAGES_PER_WPR )

/** Pages of the write protection ranges used by the tests: the last page of the first, second and third write
 *  protection register (a range of two pages from it crosses the border to the next register) */
#define UT_RAMCFG_PAGE_BORDER_1             ( 31u )
#define UT_RAMCFG_PAGE_BORDER_2             ( 63u )
#define UT_RAMCFG_PAGE_BORDER_3             ( 95u )

/** Page before the border of the first and the second write protection register (a range of four pages crosses it) */
#define UT_RAMCFG_PAGE_BEFORE_BORDER        ( 30u )

/** Pages of the protected range 1 - 2: its last page and the first page beyond it */
#define UT_RAMCFG_PAGE_LAST_OF_RANGE        ( 2u )
#define UT_RAMCFG_PAGE_BEYOND_RANGE         ( 3u )

/** First and last page of the pages which use the bits 24 - 30 of the second (56 - 62) and third (88 - 94) register */
#define UT_RAMCFG_PAGE_HIGH_WPR2            ( 56u )
#define UT_RAMCFG_PAGE_LAST_HIGH_WPR2       ( 62u )
#define UT_RAMCFG_PAGE_HIGH_WPR3            ( 88u )
#define UT_RAMCFG_PAGE_LAST_HIGH_WPR3       ( 94u )

/** Counts of the pages of the write protection ranges used by the tests */
#define UT_RAMCFG_RANGE_TWO_PAGES           ( 2u )
#define UT_RAMCFG_RANGE_THREE_PAGES         ( 3u )
#define UT_RAMCFG_RANGE_FOUR_PAGES          ( 4u )
#define UT_RAMCFG_PAGES_HIGH                ( 7u )

/** Bits of a write protection register: the first page (bit 0), the last page (bit 31), both, pages 1 - 2, pages 0 - 1,
 *  pages 0 - 2 and pages 24 - 30 */
#define UT_RAMCFG_WPR_BIT_FIRST             ( 0x00000001u )
#define UT_RAMCFG_WPR_BIT_LAST              ( 0x80000000u )
#define UT_RAMCFG_WPR_BIT_LAST_AND_FIRST    ( 0x80000001u )
#define UT_RAMCFG_WPR_PAGES_1_2             ( 0x00000006u )
#define UT_RAMCFG_WPR_PAGES_0_1             ( 0x00000003u )
#define UT_RAMCFG_WPR_PAGES_0_2             ( 0x00000007u )
#define UT_RAMCFG_WPR_BITS_24_30            ( 0x7F000000u )

/** Indexes of the write protection registers WPR1 ... WPR4 in the list of the line */
#define UT_RAMCFG_WPR1_IDX                  ( 0u )
#define UT_RAMCFG_WPR2_IDX                  ( 1u )
#define UT_RAMCFG_WPR3_IDX                  ( 2u )
#define UT_RAMCFG_WPR4_IDX                  ( 3u )

/** Offsets of the failing words of the errors recorded by the tests (SRAM2) and of the one of the backup SRAM */
#define UT_RAMCFG_ERR_OFFSET_1              ( 0x10u )
#define UT_RAMCFG_ERR_OFFSET_2              ( 0x20u )
#define UT_RAMCFG_ERR_OFFSET_3              ( 0x30u )
#define UT_RAMCFG_ERR_OFFSET_4              ( 0x40u )
#define UT_RAMCFG_ERR_OFFSET_5              ( 0x50u )
#define UT_RAMCFG_ERR_OFFSET_6              ( 0x60u )
#define UT_RAMCFG_BKP_ERR_OFFSET            ( 0x08u )

/** Counts of the recorded errors expected by the statistics tests */
#define UT_RAMCFG_ERROR_CNT_TWO             ( 2u )
#define UT_RAMCFG_ERROR_CNT_THREE           ( 3u )

/** Value of the register ID beyond the end of the register list */
#define UT_RAMCFG_ID_OFFSET                 ( 1u )

/** Priority of the RAMCFG interrupt used by the tests */
#define UT_RAMCFG_IRQ_PRIO                  ( 2u )

/** Priority of the RAMCFG interrupt returned by the default configuration */
#define UT_RAMCFG_DEFAULT_IRQ_PRIO          ( 0u )

/** Seed of the fill pattern of the register block and the difference of two neighbour words of the pattern */
#define UT_RAMCFG_FILL_SEED                 ( 0x5A000000u )
#define UT_RAMCFG_FILL_STEP                 ( 0x00010101u )

/** Value of a single bit at the position zero (shifted to the position of a page) */
#define UT_RAMCFG_BIT_ONE                   ( 1u )

/** Value stored in an output which the function must not change on an error */
#define UT_RAMCFG_SENTINEL                  ( 0x77u )

/** Address of the failing word used by the tests (offset in the memory) */
#define UT_RAMCFG_WORD_OFFSET               ( 0x40u )

/** Second address of the failing word used by the tests (offset in the memory) */
#define UT_RAMCFG_WORD_OFFSET_2             ( 0x80u )

/** Pattern of a word of an SRAM (data which the erase destroys) */
#define UT_RAMCFG_DATA_PATTERN              ( 0xCAFE1234u )

/** Offset of a stack pointer inside a memory */
#define UT_RAMCFG_STACK_INSIDE              ( 0x10u )

/** Count of callbacks: none */
#define UT_RAMCFG_CALLS_NONE                ( 0u )

/** Count of callbacks: one */
#define UT_RAMCFG_CALLS_ONCE                ( 1u )

/** Count of callbacks: two */
#define UT_RAMCFG_CALLS_TWICE               ( 2u )

/** Size of the list of the clock activations recorded from the RCC stub */
#define UT_RAMCFG_RCC_LIST_SIZE             ( 16u )

/** Flags of the behavior of the HW model */
#define UT_RAMCFG_MODEL_NONE                ( 0x00u )
#define UT_RAMCFG_MODEL_ERASE_HOLD          ( 0x01u )   /**< the erase does not end until the test releases it        */
#define UT_RAMCFG_MODEL_ECC_STUCK           ( 0x02u )   /**< ECCE cannot be set                                       */
#define UT_RAMCFG_MODEL_ECC_LOCKED          ( 0x04u )   /**< the unlock sequence of ECCE is not accepted              */
#define UT_RAMCFG_MODEL_IER_NOT_KEPT        ( 0x08u )   /**< SEIE and DEIE read as zero                               */
#define UT_RAMCFG_MODEL_ALE_NOT_KEPT        ( 0x10u )   /**< ALE reads as zero                                        */
#define UT_RAMCFG_MODEL_WPR_NOT_KEPT        ( 0x20u )   /**< the write protection bits read as zero                   */
#define UT_RAMCFG_MODEL_IER_STUCK           ( 0x40u )   /**< clearing of SEIE and DEIE is reverted                    */

/** Value of the words of a structure or an output which the function under test must not touch (memory fill) */
#define UT_RAMCFG_SENTINEL_WORD             ( 0x77777777u )

/** Marker bit which the HW model adds to the mirror of the flags in RAMCFG_MxICR (reserved bit): a store of the
 *  software is recognized by the value which differs from the mirror */
#define UT_RAMCFG_ICR_MARK                  ( 0x80000000u )

/** State of the unlock sequence of a key register of the HW model */
#define UT_RAMCFG_KEY_LOCKED                ( 0u )      /**< no key or a wrong key was written                        */
#define UT_RAMCFG_KEY_FIRST                 ( 1u )      /**< the first key was written                                */
#define UT_RAMCFG_KEY_UNLOCKED              ( 2u )      /**< both keys were written in the right order                */

/** Value used for a flag of the HW model which is not set / set */
#define UT_RAMCFG_FLAG_CLEAR                ( 0u )
#define UT_RAMCFG_FLAG_SET                  ( 1u )

/* Facts of the device lines (reference manuals RM0481 table 42 and 44, RM0492 table 9, RM0517 table 34 and 35) */
#if defined(STM32H503xx)
    #define UT_RAMCFG_SRAM1_SIZE            ( 0x4000u )      /* 16 Kbytes             */
    #define UT_RAMCFG_SRAM2_BASE            ( 0x20004000u )
    #define UT_RAMCFG_SRAM2_SIZE            ( 0x4000u )      /* 16 Kbytes             */
    #define UT_RAMCFG_SRAM2_PAGES           ( 16u )
    #define UT_RAMCFG_BKP_SIZE              ( 0x800u )       /* 2 Kbytes              */
#elif defined(STM32H523xx) || \
      defined(STM32H533xx)
    #define UT_RAMCFG_SRAM1_SIZE            ( 0x20000u )     /* 128 Kbytes            */
    #define UT_RAMCFG_SRAM2_BASE            ( 0x20020000u )
    #define UT_RAMCFG_SRAM2_SIZE            ( 0x14000u )     /* 80 Kbytes             */
    #define UT_RAMCFG_SRAM2_PAGES           ( 80u )
    #define UT_RAMCFG_SRAM3_BASE            ( 0x20034000u )
    #define UT_RAMCFG_SRAM3_SIZE            ( 0x10000u )     /* 64 Kbytes, no ECC     */
    #define UT_RAMCFG_SRAM3_ECC             ( RAMCFG_FUNCTION_INACTIVE )
    #define UT_RAMCFG_BKP_SIZE              ( 0x800u )       /* 2 Kbytes              */
#elif defined(STM32H543xx) || \
      defined(STM32H553xx)
    #define UT_RAMCFG_SRAM1_SIZE            ( 0x20000u )     /* 128 Kbytes            */
    #define UT_RAMCFG_SRAM2_BASE            ( 0x20020000u )
    #define UT_RAMCFG_SRAM2_SIZE            ( 0x14000u )     /* 80 Kbytes             */
    #define UT_RAMCFG_SRAM2_PAGES           ( 80u )
    #define UT_RAMCFG_SRAM3_BASE            ( 0x20034000u )
    #define UT_RAMCFG_SRAM3_SIZE            ( 0x18000u )     /* 96 Kbytes             */
    #define UT_RAMCFG_SRAM3_ECC             ( RAMCFG_FUNCTION_ACTIVE )
    #define UT_RAMCFG_BKP_SIZE              ( 0x800u )       /* 2 Kbytes              */
#elif defined(STM32H562xx) || \
      defined(STM32H563xx) || \
      defined(STM32H573xx)
    #define UT_RAMCFG_SRAM1_SIZE            ( 0x40000u )     /* 256 Kbytes            */
    #define UT_RAMCFG_SRAM2_BASE            ( 0x20040000u )
    #define UT_RAMCFG_SRAM2_SIZE            ( 0x10000u )     /* 64 Kbytes             */
    #define UT_RAMCFG_SRAM2_PAGES           ( 64u )
    #define UT_RAMCFG_SRAM3_BASE            ( 0x20050000u )
    #define UT_RAMCFG_SRAM3_SIZE            ( 0x50000u )     /* 320 Kbytes            */
    #define UT_RAMCFG_SRAM3_ECC             ( RAMCFG_FUNCTION_ACTIVE )
    #define UT_RAMCFG_BKP_SIZE              ( 0x1000u )      /* 4 Kbytes              */
#elif defined(STM32H5E4xx) || \
      defined(STM32H5E5xx) || \
      defined(STM32H5F4xx) || \
      defined(STM32H5F5xx)
    #define UT_RAMCFG_SRAM1_SIZE            ( 0x40000u )     /* 256 Kbytes            */
    #define UT_RAMCFG_SRAM2_BASE            ( 0x20040000u )
    #define UT_RAMCFG_SRAM2_SIZE            ( 0x20000u )     /* 128 Kbytes            */
    #define UT_RAMCFG_SRAM2_PAGES           ( 128u )
    #define UT_RAMCFG_SRAM3_BASE            ( 0x20060000u )
    #define UT_RAMCFG_SRAM3_SIZE            ( 0x60000u )     /* 384 Kbytes            */
    #define UT_RAMCFG_SRAM3_ECC             ( RAMCFG_FUNCTION_ACTIVE )
    #define UT_RAMCFG_SRAM4_BASE            ( 0x200C0000u )
    #define UT_RAMCFG_SRAM4_SIZE            ( 0x60000u )     /* 384 Kbytes            */
    #define UT_RAMCFG_SRAM5_BASE            ( 0x20120000u )
    #define UT_RAMCFG_SRAM5_SIZE            ( 0x60000u )     /* 384 Kbytes            */
    #define UT_RAMCFG_BKP_SIZE              ( 0x1000u )      /* 4 Kbytes              */
#else
    #error "Unit tests of RamCfg: unknown STM32H5 line"
#endif

/** Count of the memories of the line: SRAM1, SRAM2, backup SRAM and SRAM3 (not on STM32H503), SRAM4 and SRAM5
 *  (STM32H5E4 ... STM32H5F5) */
#if defined(STM32H503xx)
    #define UT_RAMCFG_SRAM_CNT              ( 3u )
#elif defined(STM32H5E4xx) || \
      defined(STM32H5E5xx) || \
      defined(STM32H5F4xx) || \
      defined(STM32H5F5xx)
    #define UT_RAMCFG_SRAM_CNT              ( 6u )
#else
    #define UT_RAMCFG_SRAM_CNT              ( 4u )
#endif

/** Offset of the write protection register 3 (STM32H523 ... STM32H553: 0x20, STM32H5E4 ... STM32H5F5: 0x30) */
#if defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    #define UT_RAMCFG_OFFSET_WPR3           ( 0x30u )
#else
    #define UT_RAMCFG_OFFSET_WPR3           ( 0x20u )
#endif

/** Offset of the write protection register 4 (STM32H5E4 ... STM32H5F5 only) */
#define UT_RAMCFG_OFFSET_WPR4               ( 0x34u )

/* ========================== LOCAL VARIABLES =============================== */

/** Features of the memories of the line, indexed by the SRAM list of the module */
static const utRamCfg_Sram_t utRamCfg_SramList[ RAMCFG_SRAM_CNT ] =
{
    [ RAMCFG_SRAM_1 ] =
    {
        .Base = UT_RAMCFG_SRAM1_BASE, .Size = UT_RAMCFG_SRAM1_SIZE, .Ecc = RAMCFG_FUNCTION_INACTIVE, .WpPages = 0u,
        .HasClk = RAMCFG_FUNCTION_ACTIVE, .ClkPeriph = RCC_PERIPH_SRAM1, .RegOffset = UT_RAMCFG_REGOFF_SRAM1
    },
    [ RAMCFG_SRAM_2 ] =
    {
        .Base = UT_RAMCFG_SRAM2_BASE, .Size = UT_RAMCFG_SRAM2_SIZE, .Ecc = RAMCFG_FUNCTION_ACTIVE,
        .WpPages = UT_RAMCFG_SRAM2_PAGES,
        .HasClk = RAMCFG_FUNCTION_ACTIVE, .ClkPeriph = RCC_PERIPH_SRAM2, .RegOffset = UT_RAMCFG_REGOFF_SRAM2
    },
#if defined(RAMCFG_SRAM3)
    [ RAMCFG_SRAM_3 ] =
    {
        .Base = UT_RAMCFG_SRAM3_BASE, .Size = UT_RAMCFG_SRAM3_SIZE, .Ecc = UT_RAMCFG_SRAM3_ECC, .WpPages = 0u,
        .HasClk = RAMCFG_FUNCTION_ACTIVE, .ClkPeriph = RCC_PERIPH_SRAM3, .RegOffset = UT_RAMCFG_REGOFF_SRAM3
    },
#endif
#if defined(RAMCFG_SRAM4)
    [ RAMCFG_SRAM_4 ] =
    {
        .Base = UT_RAMCFG_SRAM4_BASE, .Size = UT_RAMCFG_SRAM4_SIZE, .Ecc = RAMCFG_FUNCTION_INACTIVE, .WpPages = 0u,
        .HasClk = RAMCFG_FUNCTION_INACTIVE, .ClkPeriph = RCC_PERIPH_RAMCFG, .RegOffset = UT_RAMCFG_REGOFF_SRAM4
    },
#endif
#if defined(RAMCFG_SRAM5)
    [ RAMCFG_SRAM_5 ] =
    {
        .Base = UT_RAMCFG_SRAM5_BASE, .Size = UT_RAMCFG_SRAM5_SIZE, .Ecc = RAMCFG_FUNCTION_INACTIVE, .WpPages = 0u,
        .HasClk = RAMCFG_FUNCTION_INACTIVE, .ClkPeriph = RCC_PERIPH_RAMCFG, .RegOffset = UT_RAMCFG_REGOFF_SRAM5
    },
#endif
    [ RAMCFG_SRAM_BACKUP ] =
    {
        .Base = UT_RAMCFG_BKP_BASE, .Size = UT_RAMCFG_BKP_SIZE, .Ecc = RAMCFG_FUNCTION_ACTIVE, .WpPages = 0u,
        .HasClk = RAMCFG_FUNCTION_ACTIVE, .ClkPeriph = RCC_PERIPH_BKPSRAM, .RegOffset = UT_RAMCFG_REGOFF_BKP
    },
};

/* The members of the register structure of the CMSIS device header are used to name the registers, their offsets are
 * the ones of the reference manuals (the addresses of the registers are always computed from the offsets) */
_Static_assert( UT_RAMCFG_OFFSET_CR     == offsetof( RAMCFG_TypeDef, CR     ), "RAMCFG_TypeDef: offset of CR" );
_Static_assert( UT_RAMCFG_OFFSET_IER    == offsetof( RAMCFG_TypeDef, IER    ), "RAMCFG_TypeDef: offset of IER" );
_Static_assert( UT_RAMCFG_OFFSET_ISR    == offsetof( RAMCFG_TypeDef, ISR    ), "RAMCFG_TypeDef: offset of ISR" );
_Static_assert( UT_RAMCFG_OFFSET_SEAR   == offsetof( RAMCFG_TypeDef, SEAR   ), "RAMCFG_TypeDef: offset of SEAR" );
_Static_assert( UT_RAMCFG_OFFSET_DEAR   == offsetof( RAMCFG_TypeDef, DEAR   ), "RAMCFG_TypeDef: offset of DEAR" );
_Static_assert( UT_RAMCFG_OFFSET_ICR    == offsetof( RAMCFG_TypeDef, ICR    ), "RAMCFG_TypeDef: offset of ICR" );
_Static_assert( UT_RAMCFG_OFFSET_WPR1   == offsetof( RAMCFG_TypeDef, WPR1   ), "RAMCFG_TypeDef: offset of WPR1" );
_Static_assert( UT_RAMCFG_OFFSET_ECCKEY == offsetof( RAMCFG_TypeDef, ECCKEY ), "RAMCFG_TypeDef: offset of ECCKEY" );
_Static_assert( UT_RAMCFG_OFFSET_ERKEYR == offsetof( RAMCFG_TypeDef, ERKEYR ), "RAMCFG_TypeDef: offset of ERKEYR" );

/** Offsets of the write protection registers of the line, indexed by the index of the register (RM: WPR1 ... WPR4) */
static const uint32_t utRamCfg_WprOffsetList[] =
{
    UT_RAMCFG_OFFSET_WPR1,
#if ( UT_RAMCFG_WPR_CNT > 1u )
    UT_RAMCFG_OFFSET_WPR2,
#endif
#if ( UT_RAMCFG_WPR_CNT > 2u )
    UT_RAMCFG_OFFSET_WPR3,
#endif
#if ( UT_RAMCFG_WPR_CNT > 3u )
    UT_RAMCFG_OFFSET_WPR4,
#endif
};

_Static_assert( ( sizeof( utRamCfg_WprOffsetList ) / sizeof( utRamCfg_WprOffsetList[ 0u ] ) ) == UT_RAMCFG_WPR_CNT,
                "Unit tests: list of the write protection registers has incorrect size." );

/** ISR registered in NVIC (RAMCFG interrupt) */
static nvic_IsrCallback_t   utRamCfg_Isr;

/** NMI handler registered in NVIC by the module */
static nvic_IsrCallback_t   utRamCfg_NmiIsr;

/** NMI handler present in NVIC before the module (returned by the stub of the getter) */
static nvic_IsrCallback_t   utRamCfg_PrevNmi;

/** Count of the NMI handler installations and of the readings of the NMI handler */
static uint32_t             utRamCfg_CoreSetCnt;
static uint32_t             utRamCfg_CoreGetCnt;

/** The handler of the module is the one installed in the NMI vector */
static volatile uint32_t    utRamCfg_NmiInstalled;

/** The NMI handler was installed at the moment the NMI redirection was enabled (recorded by the HW model) */
static volatile uint32_t    utRamCfg_NmiInstalledAtEnable;

/** The NMI redirection was seen by the HW model */
static volatile uint32_t    utRamCfg_NmiEnableSeen;

/** Results returned by the NVIC stubs */
static nvic_RequestState_t  utRamCfg_NvicHandlerRet;
static nvic_RequestState_t  utRamCfg_NvicPrioRet;
static nvic_RequestState_t  utRamCfg_NvicGetPrioRet;
static nvic_RequestState_t  utRamCfg_NvicActiveRet;
static nvic_RequestState_t  utRamCfg_NvicInactiveRet;
static nvic_RequestState_t  utRamCfg_NvicSetCoreRet;
static nvic_RequestState_t  utRamCfg_NvicGetCoreRet;

/** Result returned by the RCC stub */
static rcc_RequestState_t   utRamCfg_RccRet;

/** Counts of the NVIC calls */
static uint32_t             utRamCfg_HandlerCnt;
static uint32_t             utRamCfg_PrioCnt;
static uint32_t             utRamCfg_ActiveCnt;
static uint32_t             utRamCfg_InactiveCnt;
static uint32_t             utRamCfg_GetPrioCnt;

/** The RAMCFG interrupt is active (the last call of the activation was the activation) */
static uint32_t             utRamCfg_VectorActive;

/** Priority of the RAMCFG interrupt in NVIC */
static nvic_IrqPrio_t       utRamCfg_Prio;

/** Clocks activated by the module through the RCC stub (in the order of the calls) and their count */
static rcc_PeriphId_t       utRamCfg_RccList[ UT_RAMCFG_RCC_LIST_SIZE ];
static uint32_t             utRamCfg_RccCnt;

/** Single error callback record */
static uint32_t             utRamCfg_SingleCnt;
static ramCfg_SramId_t      utRamCfg_SingleSram;
static ramCfg_Address_t     utRamCfg_SingleAddr;
static uint32_t             utRamCfg_SingleFlagAtCall;

/** Double error callback record */
static uint32_t             utRamCfg_DoubleCnt;
static ramCfg_SramId_t      utRamCfg_DoubleSram;
static ramCfg_Address_t     utRamCfg_DoubleAddr;
static uint32_t             utRamCfg_DoubleFlagAtCall;

/** Erase complete callback record */
static uint32_t             utRamCfg_EraseCnt;
static ramCfg_SramId_t      utRamCfg_EraseSram;

/** Behavior of the HW model (UT_RAMCFG_MODEL_xxx) */
static volatile uint32_t    utRamCfg_ModelFlags;

/** Count of the evaluations of the HW model (every store to the emulated registers) */
static volatile uint32_t    utRamCfg_ModelCalls;

/** Count of the evaluations of the HW model at the moment the flag SEDC was cleared by a write of the clear register */
static volatile uint32_t    utRamCfg_ClearAtCall;

/** Previous values of the interrupt enable bits (SEIE, DEIE) and the mirror of the flags in the clear register */
static volatile uint32_t    utRamCfg_IerPrev[ RAMCFG_SRAM_CNT ];
static volatile uint32_t    utRamCfg_IcrMirror[ RAMCFG_SRAM_CNT ];

/** The error interrupts were enabled (seen by the HW model) and the address latch was set at that moment */
static volatile uint32_t    utRamCfg_IerSeen[ RAMCFG_SRAM_CNT ];
static volatile uint32_t    utRamCfg_AleAtIerEnable[ RAMCFG_SRAM_CNT ];

/** State of the unlock sequence of ECCE, the previous value of the key register and of ECCE, count of the unlocks */
static volatile uint32_t    utRamCfg_EccKeyState[ RAMCFG_SRAM_CNT ];
static volatile uint32_t    utRamCfg_EccKeyPrev[ RAMCFG_SRAM_CNT ];
static volatile uint32_t    utRamCfg_EccePrev[ RAMCFG_SRAM_CNT ];
static volatile uint32_t    utRamCfg_EccUnlockCnt[ RAMCFG_SRAM_CNT ];

/** State of the unlock sequence of SRAMER, the previous value of the key register and of SRAMER, count of the erases */
static volatile uint32_t    utRamCfg_EraseKeyState[ RAMCFG_SRAM_CNT ];
static volatile uint32_t    utRamCfg_EraseKeyPrev[ RAMCFG_SRAM_CNT ];
static volatile uint32_t    utRamCfg_EraseBitPrev[ RAMCFG_SRAM_CNT ];
static volatile uint32_t    utRamCfg_EraseStartCnt[ RAMCFG_SRAM_CNT ];

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    CmsisHost_Reset();

    utRamCfg_Isr                  = NULL;
    utRamCfg_NmiIsr               = NULL;
    utRamCfg_PrevNmi              = UtRamCfg_PrevNmiHandler;
    utRamCfg_CoreSetCnt           = UT_RAMCFG_CALLS_NONE;
    utRamCfg_CoreGetCnt           = UT_RAMCFG_CALLS_NONE;
    utRamCfg_NmiInstalled         = UT_RAMCFG_FLAG_CLEAR;
    utRamCfg_NmiInstalledAtEnable = UT_RAMCFG_FLAG_CLEAR;
    utRamCfg_NmiEnableSeen        = UT_RAMCFG_FLAG_CLEAR;

    utRamCfg_NvicHandlerRet       = NVIC_REQUEST_OK;
    utRamCfg_NvicPrioRet          = NVIC_REQUEST_OK;
    utRamCfg_NvicGetPrioRet       = NVIC_REQUEST_OK;
    utRamCfg_NvicActiveRet        = NVIC_REQUEST_OK;
    utRamCfg_NvicInactiveRet      = NVIC_REQUEST_OK;
    utRamCfg_NvicSetCoreRet       = NVIC_REQUEST_OK;
    utRamCfg_NvicGetCoreRet       = NVIC_REQUEST_OK;
    utRamCfg_RccRet               = RCC_REQUEST_OK;

    utRamCfg_HandlerCnt           = UT_RAMCFG_CALLS_NONE;
    utRamCfg_PrioCnt              = UT_RAMCFG_CALLS_NONE;
    utRamCfg_ActiveCnt            = UT_RAMCFG_CALLS_NONE;
    utRamCfg_InactiveCnt          = UT_RAMCFG_CALLS_NONE;
    utRamCfg_GetPrioCnt           = UT_RAMCFG_CALLS_NONE;
    utRamCfg_VectorActive         = UT_RAMCFG_FLAG_CLEAR;
    utRamCfg_Prio                 = 0u;
    utRamCfg_RccCnt               = UT_RAMCFG_CALLS_NONE;

    utRamCfg_SingleCnt            = UT_RAMCFG_CALLS_NONE;
    utRamCfg_SingleSram           = RAMCFG_SRAM_UNUSED;
    utRamCfg_SingleAddr           = RAMCFG_ADDRESS_UNUSED;
    utRamCfg_SingleFlagAtCall     = UT_RAMCFG_FLAG_CLEAR;
    utRamCfg_DoubleCnt            = UT_RAMCFG_CALLS_NONE;
    utRamCfg_DoubleSram           = RAMCFG_SRAM_UNUSED;
    utRamCfg_DoubleAddr           = RAMCFG_ADDRESS_UNUSED;
    utRamCfg_DoubleFlagAtCall     = UT_RAMCFG_FLAG_CLEAR;
    utRamCfg_EraseCnt             = UT_RAMCFG_CALLS_NONE;
    utRamCfg_EraseSram            = RAMCFG_SRAM_UNUSED;

    utRamCfg_ModelFlags           = UT_RAMCFG_MODEL_NONE;
    utRamCfg_ModelCalls           = 0u;
    utRamCfg_ClearAtCall          = 0u;

    for( uint32_t sramIdx = 0u; RAMCFG_SRAM_CNT > sramIdx; sramIdx ++ )
    {
        utRamCfg_EccKeyState[ sramIdx ]      = UT_RAMCFG_KEY_LOCKED;
        utRamCfg_EccKeyPrev[ sramIdx ]       = 0u;
        utRamCfg_EccePrev[ sramIdx ]         = 0u;
        utRamCfg_EccUnlockCnt[ sramIdx ]     = 0u;
        utRamCfg_EraseKeyState[ sramIdx ]    = UT_RAMCFG_KEY_LOCKED;
        utRamCfg_EraseKeyPrev[ sramIdx ]     = 0u;
        utRamCfg_EraseBitPrev[ sramIdx ]     = 0u;
        utRamCfg_EraseStartCnt[ sramIdx ]    = 0u;
        utRamCfg_IerPrev[ sramIdx ]          = 0u;
        utRamCfg_IcrMirror[ sramIdx ]        = 0u;
        utRamCfg_IerSeen[ sramIdx ]          = UT_RAMCFG_FLAG_CLEAR;
        utRamCfg_AleAtIerEnable[ sramIdx ]   = UT_RAMCFG_FLAG_CLEAR;
    }

    Nvic_Set_PeriphIrq_Handler_Stub( UtRamCfg_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_Stub( UtRamCfg_NvicSetPrioStub );
    Nvic_Get_PeriphIrq_Prio_Stub( UtRamCfg_NvicGetPrioStub );
    Nvic_Set_PeriphIrq_Active_Stub( UtRamCfg_NvicActiveStub );
    Nvic_Set_PeriphIrq_Inactive_Stub( UtRamCfg_NvicInactiveStub );
    Nvic_Set_CoreIrq_Handler_Stub( UtRamCfg_NvicSetCoreStub );
    Nvic_Get_CoreIrq_Handler_Stub( UtRamCfg_NvicGetCoreStub );
    Rcc_Set_PeriphActive_Stub( UtRamCfg_RccActiveStub );
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   RamCfg_Get_ModuleVersion() returns version of the module.
 *
 * \details Reads the module version structure.
 *
 * \par Expected results
 * - Version is 0.1.0 (Major 0, Minor 1, Patch 0).
 */
void Ut_RamCfg_Get_ModuleVersion_ReturnsVersion( void )
{
    ramCfg_ModuleVersion_t version = RamCfg_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( UT_RAMCFG_VERSION_MAJOR, version.Major );
    TEST_ASSERT_EQUAL_UINT8( UT_RAMCFG_VERSION_MINOR, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( UT_RAMCFG_VERSION_PATCH, version.Patch );
}

/* ============================ SRAM LIST =================================== */

/**
 * \brief   The list of the SRAMs contains exactly the memories of the device line.
 *
 * \details Compares the count of the list and the position of the memories with the reference manual of the line.
 *
 * \par Expected results
 * - The count of the SRAMs is 3 (STM32H503), 4 (STM32H523 ... STM32H573) or 6 (STM32H5E4 ... STM32H5F5).
 * - SRAM1 and SRAM2 are the first two items, the backup SRAM is the last one, the item "unused" is outside of the list.
 */
void Ut_RamCfg_SramList_MatchesDeviceLine_CountAndOrder( void )
{
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_SRAM_CNT, RAMCFG_SRAM_CNT );

    TEST_ASSERT_EQUAL_UINT32( 0u, RAMCFG_SRAM_1 );
    TEST_ASSERT_EQUAL_UINT32( 1u, RAMCFG_SRAM_2 );
    TEST_ASSERT_EQUAL_UINT32( RAMCFG_SRAM_CNT - 1u, RAMCFG_SRAM_BACKUP );
    TEST_ASSERT_TRUE( RAMCFG_SRAM_CNT <= RAMCFG_SRAM_UNUSED );
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   RamCfg_Get_DefaultConfig() fills default configuration.
 *
 * \details Reads the default configuration into a structure filled with a pattern, then calls the function with NULL.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK; for every SRAM: check mode unchanged, reporting by the interrupt for the SRAMs with ECC and
 *   no reporting for the others, scrubbing inactive, no write protection requested.
 * - No callback, priority 0.
 * - NULL pointer: RAMCFG_REQUEST_ERROR. No register is accessed and no other module is called.
 */
void Ut_RamCfg_Get_DefaultConfig_ReturnsDefaults( void )
{
    ramCfg_Config_t config;

    (void)memset( &config, UT_RAMCFG_SENTINEL, sizeof( config ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_DefaultConfig( &config ) );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        TEST_ASSERT_EQUAL( RAMCFG_CHECK_UNCHANGED, config.Sram[ sramId ].CheckMode );
        TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, config.Sram[ sramId ].Scrubbing );
        TEST_ASSERT_EQUAL_UINT16( 0u, config.Sram[ sramId ].ProtectionFirstPage );
        TEST_ASSERT_EQUAL_UINT16( RAMCFG_PAGE_CNT_UNUSED, config.Sram[ sramId ].ProtectionPageCnt );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REPORT_IRQ, config.Sram[ sramId ].ReportMode );
        }
        else
        {
            TEST_ASSERT_EQUAL( RAMCFG_REPORT_NONE, config.Sram[ sramId ].ReportMode );
        }
    }

    TEST_ASSERT_NULL( config.SingleErrorCallback );
    TEST_ASSERT_NULL( config.DoubleErrorCallback );
    TEST_ASSERT_NULL( config.EraseCompleteCallback );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_DEFAULT_IRQ_PRIO, config.IrqPriority );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_DefaultConfig( NULL ) );

    UtRamCfg_Check_NoOtherModuleCalls();
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   RamCfg_Init() with the default configuration reports the SRAMs with ECC by the interrupt.
 *
 * \details The ECC enable bits are preset as the option bytes would set them. Initializes the module with the default
 *          configuration (check mode unchanged, reporting by the interrupt for the SRAMs with ECC).
 *
 * \note    Device errata bug AB#1838 (ES0561 2.2.20 "SRAM ECC error flags and addresses are updated only if interrupt
 *          is enabled"): the failing address and the flags are stored only while the address latch and the error
 *          interrupt are enabled (documentation erratum of STM32H503); the test proves that the default reporting
 *          enables the address latch together with the single and the double error interrupt of every SRAM with ECC.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the clock of the RAMCFG peripheral activated (and no other clock), the interrupt deactivated
 *   first, then handler and priority configured and the interrupt activated. The NMI vector is not touched.
 * - SRAMs with ECC: single and double error interrupts and the address latch enabled, ECC enable bit kept.
 * - SRAMs without ECC: registers untouched.
 */
void Ut_RamCfg_Init_DefaultConfig_ReportsEccSramsByInterrupt( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            UtRamCfg_Get_Inst( sramId )->CR = UT_RAMCFG_CR_ECCE;
        }
        else
        {
            /* The ECC of the SRAM is not controlled */
        }
    }

    UtRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );
        const RAMCFG_TypeDef * const  inst   = UtRamCfg_Get_Inst( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_IRQ, inst->IER );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE | UT_RAMCFG_CR_ALE, inst->CR );
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->IER );
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->CR );
        }
    }

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_RccCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_RAMCFG, utRamCfg_RccList[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_HandlerCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_PrioCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_ActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_DEFAULT_IRQ_PRIO, utRamCfg_Prio );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_VectorActive );
    TEST_ASSERT_NOT_NULL( utRamCfg_Isr );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_CoreSetCnt );
}


/**
 * \brief   RamCfg_Init() without any reporting keeps the interrupt inactive.
 *
 * \details The error interrupts and the address latch are preset as enabled. Initializes the module with the
 *          reporting disabled for all SRAMs.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the interrupt deactivated (once at the start of the reconfiguration, once when no SRAM is
 *   reported) and not activated, no handler and no priority configured.
 * - The error interrupts and the address latch of the SRAMs with ECC are disabled.
 */
void Ut_RamCfg_Init_NoReporting_KeepsInterruptInactive( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_NONE;

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            UtRamCfg_Get_Inst( sramId )->IER = UT_RAMCFG_IER_IRQ;
            UtRamCfg_Get_Inst( sramId )->CR  = UT_RAMCFG_CR_ALE;
        }
        else
        {
            /* The ECC of the SRAM is not controlled */
        }
    }

    UtRamCfg_Init_Config( &config );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( sramId )->IER );
        TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( sramId )->CR );
    }

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_TWICE, utRamCfg_InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_HandlerCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_PrioCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_ActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_CLEAR, utRamCfg_VectorActive );
}


/**
 * \brief   RamCfg_Init() enables the ECC of the SRAMs which have it.
 *
 * \details Initializes the module with the check mode ECC for every SRAM with ECC (the other SRAMs are left unchanged).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the ECC enable bit set in every SRAM with ECC.
 * - The registers of the other SRAMs are not changed (the ECC enable bit is reserved there).
 */
void Ut_RamCfg_Init_CheckModeEcc_EnablesEccOfSramsWithEcc( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            config.Sram[ sramId ].CheckMode = RAMCFG_CHECK_ECC;
        }
        else
        {
            /* The ECC is not available, the default check mode stays */
        }
    }

    UtRamCfg_Init_Config( &config );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );
        const RAMCFG_TypeDef * const  inst   = UtRamCfg_Get_Inst( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE, inst->CR & UT_RAMCFG_CR_ECCE );
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->CR );
        }
    }
}


/**
 * \brief   RamCfg_Init() disables the ECC with the unlock sequence of the hardware.
 *
 * \details The ECC enable bit is preset as the option bytes would set it, the HW model accepts the clearing of the
 *          bit only after the keys 0xAE and 0x75 were written to the key register. Initializes the module with the
 *          check mode none for every SRAM.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the ECC enable bit cleared in every SRAM with ECC, the unlock sequence accepted once per SRAM,
 *   the key register holds the second key.
 * - The check mode none of an SRAM without ECC is accepted without any write.
 */
void Ut_RamCfg_Init_CheckModeNone_DisablesEccWithUnlockSequence( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        config.Sram[ sramId ].CheckMode  = RAMCFG_CHECK_NONE;
        config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_NONE;

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            UtRamCfg_Get_Inst( sramId )->CR = UT_RAMCFG_CR_ECCE;
        }
        else
        {
            /* The ECC of the SRAM is not controlled */
        }
    }

    UtRamCfg_Init_Config( &config );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );
        const RAMCFG_TypeDef * const  inst   = UtRamCfg_Get_Inst( sramId );

        TEST_ASSERT_EQUAL_HEX32( 0u, inst->CR );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_EccUnlockCnt[ sramId ] );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ECC_KEY_2, inst->ECCKEY );
        }
        else
        {
            TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_EccUnlockCnt[ sramId ] );
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->ECCKEY );
        }
    }
}


/**
 * \brief   RamCfg_Init() rejects a null configuration pointer without any access.
 *
 * \details Initializes with NULL after the register block was filled with a pattern.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, no register written, no other module called, module not initialized.
 */
void Ut_RamCfg_Init_NullConfig_ReturnsErrorWithoutAccess( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Init( NULL ) );

    UtRamCfg_Check_BlockUnchanged( block );
    UtRamCfg_Check_NoOtherModuleCalls();
    UtRamCfg_Check_NotInitialized();
}


/**
 * \brief   RamCfg_Init() rejects a check mode which the hardware does not have.
 *
 * \details Requests the parity check (the SRAMs of STM32H5 have the ECC only), a value outside of the list and the
 *          ECC of every SRAM which has none.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, before any register write: block unchanged, no other module called, module
 *   not initialized.
 */
void Ut_RamCfg_Init_UnsupportedCheckMode_ReturnsErrorWithoutAccess( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].CheckMode = RAMCFG_CHECK_PARITY;
    UtRamCfg_Check_InitRejected( &config );

    config.Sram[ RAMCFG_SRAM_2 ].CheckMode = RAMCFG_CHECK_CNT;
    UtRamCfg_Check_InitRejected( &config );

    config.Sram[ RAMCFG_SRAM_2 ].CheckMode = RAMCFG_CHECK_UNCHANGED;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            config.Sram[ sramId ].CheckMode = RAMCFG_CHECK_ECC;
            UtRamCfg_Check_InitRejected( &config );
            config.Sram[ sramId ].CheckMode = RAMCFG_CHECK_UNCHANGED;
        }
        else
        {
            /* The SRAM has the ECC */
        }
    }
}


/**
 * \brief   RamCfg_Init() rejects a reporting mode which the SRAM cannot have.
 *
 * \details Requests a value outside of the list and the reporting by the interrupt and by the NMI for every SRAM
 *          without ECC.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, before any register write: block unchanged, no other module called, module
 *   not initialized.
 */
void Ut_RamCfg_Init_UnsupportedReportMode_ReturnsErrorWithoutAccess( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].ReportMode = RAMCFG_REPORT_CNT;
    UtRamCfg_Check_InitRejected( &config );

    config.Sram[ RAMCFG_SRAM_2 ].ReportMode = RAMCFG_REPORT_IRQ;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_IRQ;
            UtRamCfg_Check_InitRejected( &config );

            config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_NMI;
            UtRamCfg_Check_InitRejected( &config );

            config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_NONE;
        }
        else
        {
            /* The SRAM has the ECC */
        }
    }
}


/**
 * \brief   RamCfg_Init() rejects a scrubbing which the SRAM cannot have.
 *
 * \details Requests a state outside of the list and the scrubbing of every SRAM without ECC.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, before any register write: block unchanged, no other module called, module
 *   not initialized.
 */
void Ut_RamCfg_Init_UnsupportedScrubbing_ReturnsErrorWithoutAccess( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].Scrubbing = (ramCfg_FunctionState_t)( RAMCFG_FUNCTION_ACTIVE + UT_RAMCFG_ID_OFFSET );
    UtRamCfg_Check_InitRejected( &config );

    config.Sram[ RAMCFG_SRAM_2 ].Scrubbing = RAMCFG_FUNCTION_INACTIVE;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            config.Sram[ sramId ].Scrubbing = RAMCFG_FUNCTION_ACTIVE;
            UtRamCfg_Check_InitRejected( &config );
            config.Sram[ sramId ].Scrubbing = RAMCFG_FUNCTION_INACTIVE;
        }
        else
        {
            /* The SRAM has the ECC */
        }
    }
}


/**
 * \brief   RamCfg_Init() rejects a write protection which the SRAM cannot have.
 *
 * \details Requests one page of every SRAM without the write protection (every SRAM except SRAM2), and for SRAM2 a
 *          range which starts beyond the pages, ends beyond the pages and a count beyond the pages.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, before any register write: block unchanged, no other module called, module
 *   not initialized.
 */
void Ut_RamCfg_Init_UnsupportedProtection_ReturnsErrorWithoutAccess( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( 0u == expect->WpPages )
        {
            config.Sram[ sramId ].ProtectionFirstPage = 0u;
            config.Sram[ sramId ].ProtectionPageCnt   = 1u;
            UtRamCfg_Check_InitRejected( &config );
            config.Sram[ sramId ].ProtectionPageCnt   = RAMCFG_PAGE_CNT_UNUSED;
        }
        else
        {
            /* The SRAM has the write protection */
        }
    }

    config.Sram[ RAMCFG_SRAM_2 ].ProtectionFirstPage = UT_RAMCFG_SRAM2_PAGES;
    config.Sram[ RAMCFG_SRAM_2 ].ProtectionPageCnt   = 1u;
    UtRamCfg_Check_InitRejected( &config );

    config.Sram[ RAMCFG_SRAM_2 ].ProtectionFirstPage = UT_RAMCFG_SRAM2_PAGES - 1u;
    config.Sram[ RAMCFG_SRAM_2 ].ProtectionPageCnt   = UT_RAMCFG_RANGE_TWO_PAGES;
    UtRamCfg_Check_InitRejected( &config );

    config.Sram[ RAMCFG_SRAM_2 ].ProtectionFirstPage = 0u;
    config.Sram[ RAMCFG_SRAM_2 ].ProtectionPageCnt   = UT_RAMCFG_SRAM2_PAGES + 1u;
    UtRamCfg_Check_InitRejected( &config );
}


/**
 * \brief   RamCfg_Init() rejects a write protection of pages which hold the stack of the caller.
 *
 * \details Sets the main stack pointer, then the process stack pointer into the first page of SRAM2 and requests the
 *          protection of that page (the secure alias of the stack pointer is used for the second case).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases, before any register write: block unchanged, no other module called, module
 *   not initialized.
 */
void Ut_RamCfg_Init_ProtectionOfStackPages_ReturnsErrorWithoutAccess( void )
{
    ramCfg_Config_t               config = UtRamCfg_Get_Config();
    const utRamCfg_Sram_t * const sram2  = UtRamCfg_Get_Expect( RAMCFG_SRAM_2 );

    config.Sram[ RAMCFG_SRAM_2 ].ProtectionFirstPage = 0u;
    config.Sram[ RAMCFG_SRAM_2 ].ProtectionPageCnt   = 1u;

    __set_MSP( sram2->Base + UT_RAMCFG_STACK_INSIDE );
    UtRamCfg_Check_InitRejected( &config );

    __set_MSP( 0u );
    __set_PSP( ( sram2->Base | UT_RAMCFG_SECURE_ALIAS ) + UT_RAMCFG_STACK_INSIDE );
    UtRamCfg_Check_InitRejected( &config );
}


/**
 * \brief   RamCfg_Init() reports an error of the peripheral clock and deinitializes the module.
 *
 * \details The RCC stub refuses the activation of the clock of the RAMCFG peripheral (the register block is filled
 *          with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, no register written, the module is deinitialized (the interrupt deactivated by the
 *   deinitialization as well), module not initialized.
 */
void Ut_RamCfg_Init_RccError_ReturnsErrorWithoutRegisterWrite( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();
    uint32_t        block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    utRamCfg_RccRet = RCC_REQUEST_ERROR;

    UtRamCfg_Check_InitFailed( &config );

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_RccCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_TWICE, utRamCfg_InactiveCnt );
}


/**
 * \brief   RamCfg_Init() reports a refusal of the interrupt deactivation and does not touch the registers.
 *
 * \details The NVIC stub refuses the deactivation of the interrupt, which is the first step of the initialization.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, no register written, the clock is not activated, module not initialized.
 */
void Ut_RamCfg_Init_NvicInactiveError_ReturnsErrorWithoutRegisterWrite( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();
    uint32_t        block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    utRamCfg_NvicInactiveRet = NVIC_REQUEST_ERROR;

    UtRamCfg_Check_InitFailed( &config );

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}


/**
 * \brief   RamCfg_Init() reports a refusal of the handler, the priority or the activation of the interrupt.
 *
 * \details The NVIC stub refuses in turn the registration of the handler, the priority and the activation of the
 *          interrupt after the registers were written.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the module is deinitialized: module not initialized, the interrupt inactive,
 *   the error interrupts of the SRAMs disabled again.
 */
void Ut_RamCfg_Init_NvicRefusal_ReturnsErrorAndDeinitializes( void )
{
    nvic_RequestState_t * const refusalList[] =
    {
        &utRamCfg_NvicHandlerRet,
        &utRamCfg_NvicPrioRet,
        &utRamCfg_NvicActiveRet
    };

    for( uint32_t caseIdx = 0u; ( sizeof( refusalList ) / sizeof( refusalList[ 0u ] ) ) > caseIdx; caseIdx ++ )
    {
        ramCfg_Config_t config = UtRamCfg_Get_Config();

        *refusalList[ caseIdx ] = NVIC_REQUEST_ERROR;

        UtRamCfg_Check_InitFailed( &config );

        TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_CLEAR, utRamCfg_VectorActive );
        TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER & UT_RAMCFG_IER_IRQ );
        TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_BACKUP )->IER & UT_RAMCFG_IER_IRQ );

        *refusalList[ caseIdx ] = NVIC_REQUEST_OK;
    }
}


/**
 * \brief   RamCfg_Init() detects an ECC enable bit which does not stay set.
 *
 * \details The HW model keeps the ECC enable bit at zero. Initializes with the check mode ECC for SRAM2.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR (read-back mismatch), the module is deinitialized, module not initialized.
 */
void Ut_RamCfg_Init_EccReadBackMismatch_ReturnsErrorAndDeinitializes( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].CheckMode = RAMCFG_CHECK_ECC;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ECC_STUCK );

    UtRamCfg_Check_InitFailed( &config );
}


/**
 * \brief   RamCfg_Init() detects an ECC enable bit which cannot be cleared.
 *
 * \details The ECC is enabled, the HW model does not accept the unlock sequence. Initializes with the check mode none
 *          for SRAM2.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR (read-back mismatch), the module is deinitialized, module not initialized.
 */
void Ut_RamCfg_Init_EccCannotBeDisabled_ReturnsErrorAndDeinitializes( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].CheckMode = RAMCFG_CHECK_NONE;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ECC_LOCKED );
    UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->CR = UT_RAMCFG_CR_ECCE;

    UtRamCfg_Check_InitFailed( &config );
}


/**
 * \brief   RamCfg_Init() detects error interrupt enables and an address latch which do not stay set.
 *
 * \details The HW model keeps the error interrupt enables at zero, then the address latch at zero. Initializes with the
 *          default configuration (reporting by the interrupt) in both cases.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases (read-back mismatch), the module is deinitialized, module not initialized.
 */
void Ut_RamCfg_Init_ReportingReadBackMismatch_ReturnsErrorAndDeinitializes( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_IER_NOT_KEPT );
    UtRamCfg_Check_InitFailed( &config );

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ALE_NOT_KEPT );
    UtRamCfg_Check_InitFailed( &config );
}


/**
 * \brief   RamCfg_Init() protects the requested pages of SRAM2.
 *
 * \details Initializes with the protection of two pages starting at the page 1 of SRAM2.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the bits of the pages 1 and 2 are set in the first write protection register (0x00000006), all
 *   other write protection bits are clear.
 */
void Ut_RamCfg_Init_Protection_ProtectsRequestedPages( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].ProtectionFirstPage = 1u;
    config.Sram[ RAMCFG_SRAM_2 ].ProtectionPageCnt   = UT_RAMCFG_RANGE_TWO_PAGES;

    UtRamCfg_Init_Config( &config );

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_PAGES_1_2, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR1_IDX ) );
    UtRamCfg_Check_Wpr( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES );
}


/**
 * \brief   RamCfg_Init() detects write protection bits which do not stay set.
 *
 * \details The HW model keeps the write protection bits at zero. Initializes with the protection of one page of SRAM2.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR (read-back mismatch), the module is deinitialized, module not initialized.
 */
void Ut_RamCfg_Init_ProtectionReadBackMismatch_ReturnsErrorAndDeinitializes( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].ProtectionFirstPage = 0u;
    config.Sram[ RAMCFG_SRAM_2 ].ProtectionPageCnt   = 1u;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_WPR_NOT_KEPT );

    UtRamCfg_Check_InitFailed( &config );
}


/**
 * \brief   RamCfg_Init() stores the callbacks and the interrupt priority.
 *
 * \details Initializes with the callbacks and the priority 2, raises a single error and a double error in SRAM2 and
 *          runs the interrupt handler registered in NVIC.
 *
 * \par Expected results
 * - The priority 2 is passed to NVIC, the handler registered in NVIC calls both callbacks once with the SRAM and the
 *   address.
 */
void Ut_RamCfg_Init_CallbacksAndPriority_AreStored( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.SingleErrorCallback = UtRamCfg_SingleCallback;
    config.DoubleErrorCallback = UtRamCfg_DoubleCallback;
    config.IrqPriority         = UT_RAMCFG_IRQ_PRIO;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Config( &config );

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_IRQ_PRIO, utRamCfg_Prio );
    TEST_ASSERT_NOT_NULL( utRamCfg_Isr );

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET );
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET_2 );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_SingleCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_2, utRamCfg_SingleSram );
    TEST_ASSERT_EQUAL_HEX32( UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET, utRamCfg_SingleAddr );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_DoubleCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_2, utRamCfg_DoubleSram );
    TEST_ASSERT_EQUAL_HEX32( UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET_2,
                             utRamCfg_DoubleAddr );
}


/**
 * \brief   A repeated RamCfg_Init() starts the statistics from zero.
 *
 * \details Initializes the module, records a single error in SRAM2, initializes the module again.
 *
 * \par Expected results
 * - Before the second initialization the statistics show one single error, after it no error and no address.
 */
void Ut_RamCfg_Init_Repeated_ClearsStatistics( void )
{
    ramCfg_ErrorStatistics_t statistics;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.SingleErrorCnt );

    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_UNUSED, statistics.LastErrorSram );
    TEST_ASSERT_EQUAL_HEX32( RAMCFG_ADDRESS_UNUSED, statistics.LastErrorAddress );
}


/**
 * \brief   RamCfg_Init() with the NMI reporting installs the NMI handler before the NMI redirection is enabled.
 *
 * \details Initializes with the reporting by the NMI for SRAM2 (the HW model records whether the NMI handler of the
 *          module was installed when the NMI redirection bit appeared).
 *
 * \note    Device errata bug AB#1838 (ES0561 2.2.20 "SRAM ECC error flags and addresses are updated only if interrupt
 *          is enabled"): the failing address and the flags are stored only while the address latch and the error
 *          interrupt are enabled (documentation erratum of STM32H503); the test proves that the NMI reporting enables
 *          the address latch together with the single error interrupt and the NMI redirection.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the NMI handler of the module installed once, the handler already installed when the redirection
 *   was enabled, the interrupt activated for the single error.
 * - SRAM2: single error interrupt and NMI redirection set, double error interrupt clear, address latch set.
 */
void Ut_RamCfg_Init_NmiMode_InstallsHandlerBeforeEnablingNmi( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].ReportMode = RAMCFG_REPORT_NMI;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Config( &config );

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_CoreSetCnt );
    TEST_ASSERT_NOT_NULL( utRamCfg_NmiIsr );
    TEST_ASSERT_TRUE( UtRamCfg_PrevNmiHandler != utRamCfg_NmiIsr );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_NmiEnableSeen );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_NmiInstalledAtEnable );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_VectorActive );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_NMI, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ALE, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->CR & UT_RAMCFG_CR_ALE );
}


/**
 * \brief   RamCfg_Init() with the NMI reporting does not enable the NMI redirection if the NMI vector cannot be taken
 *          over.
 *
 * \details The NVIC stub refuses the reading of the NMI handler, then the installation of the module handler.
 *          Initializes with the reporting by the NMI for SRAM2.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases, the NMI redirection bit stays clear (it cannot be undone), module not
 *   initialized.
 */
void Ut_RamCfg_Init_NmiHandlerRefused_ReturnsErrorWithoutNmiRedirection( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].ReportMode = RAMCFG_REPORT_NMI;

    utRamCfg_NvicGetCoreRet = NVIC_REQUEST_ERROR;
    UtRamCfg_Check_InitFailed( &config );
    TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER & UT_RAMCFG_IER_ECCNMI );

    utRamCfg_NvicGetCoreRet = NVIC_REQUEST_OK;
    utRamCfg_NvicSetCoreRet = NVIC_REQUEST_ERROR;
    UtRamCfg_Check_InitFailed( &config );
    TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER & UT_RAMCFG_IER_ECCNMI );
}

/* =========================== DEINITIALIZATION ============================= */

/**
 * \brief   RamCfg_Deinit() disables the error reporting and removes the callbacks.
 *
 * \details Initializes with ECC and the protection of two pages of SRAM2 and with the callbacks, then deinitializes the
 *          module and runs the interrupt handler with the error flags set.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the interrupt inactive, the error interrupts and the address latch disabled in every SRAM with
 *   ECC, module not initialized. The clock of the RAMCFG peripheral is not disabled (a call of the RCC stub would fail
 *   the test).
 * - What the hardware cannot undo stays: the ECC enable bits and the write protection bits.
 * - The handler of a former initialization neither calls a callback nor acknowledges a flag.
 */
void Ut_RamCfg_Deinit_Initialized_DisablesReportingAndRemovesCallbacks( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].CheckMode           = RAMCFG_CHECK_ECC;
    config.Sram[ RAMCFG_SRAM_2 ].ProtectionFirstPage = 0u;
    config.Sram[ RAMCFG_SRAM_2 ].ProtectionPageCnt   = UT_RAMCFG_RANGE_TWO_PAGES;
    config.SingleErrorCallback = UtRamCfg_SingleCallback;
    config.DoubleErrorCallback = UtRamCfg_DoubleCallback;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Config( &config );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Deinit() );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );
        const RAMCFG_TypeDef * const  inst   = UtRamCfg_Get_Inst( sramId );

        TEST_ASSERT_EQUAL_HEX32( 0u, inst->IER );
        TEST_ASSERT_EQUAL_HEX32( 0u, inst->CR & UT_RAMCFG_CR_ALE );

        if( RAMCFG_SRAM_2 == sramId )
        {
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE, inst->CR & UT_RAMCFG_CR_ECCE );
        }
        else if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->CR );
        }
        else
        {
            /* The ECC enable bit of the other SRAMs was not requested */
        }
    }

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_PAGES_0_1, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR1_IDX ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_CLEAR, utRamCfg_VectorActive );
    UtRamCfg_Check_NotInitialized();

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base );
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_SingleCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_DoubleCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR );
}


/**
 * \brief   RamCfg_Deinit() of a module which is not initialized only deactivates the interrupt.
 *
 * \details Fills the register block with a pattern and deinitializes the module which was not initialized.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the interrupt deactivated once, no register written, no other call of NVIC or RCC.
 */
void Ut_RamCfg_Deinit_NotInitialized_OnlyDeactivatesInterrupt( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Deinit() );

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_ActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_CoreSetCnt );
}


/**
 * \brief   RamCfg_Deinit() reports a refusal of the interrupt deactivation and still deinitializes the module.
 *
 * \details Initializes with the default configuration, the NVIC stub refuses the deactivation, deinitializes.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, the error interrupts disabled, module not initialized.
 */
void Ut_RamCfg_Deinit_NvicError_ReturnsErrorAndStillDeinitializes( void )
{
    UtRamCfg_Init_Default();

    utRamCfg_NvicInactiveRet = NVIC_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Deinit() );

    TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER );
    UtRamCfg_Check_NotInitialized();
}


/**
 * \brief   RamCfg_Deinit() reports error interrupt enables which cannot be cleared and still deinitializes the module.
 *
 * \details Initializes with the default configuration, the HW model keeps the error interrupt enables set,
 *          deinitializes.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR (read-back mismatch), the interrupt deactivated, module not initialized.
 */
void Ut_RamCfg_Deinit_ReadBackMismatch_ReturnsErrorAndStillDeinitializes( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    utRamCfg_ModelFlags = UT_RAMCFG_MODEL_IER_STUCK;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Deinit() );

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_CLEAR, utRamCfg_VectorActive );
    UtRamCfg_Check_NotInitialized();
}


/**
 * \brief   RamCfg_Deinit() restores the NMI handler which was installed before the module.
 *
 * \details Initializes with the reporting by the NMI for SRAM2, deinitializes the module (once with success, once with
 *          the refusal of the restoration by the NVIC stub).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the previous NMI handler installed again, the NMI redirection bit stays set (the hardware cannot
 *   undo it, the module does not write it), module not initialized.
 * - With the refusal RAMCFG_REQUEST_ERROR.
 */
void Ut_RamCfg_Deinit_NmiMode_RestoresPreviousNmiHandler( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ RAMCFG_SRAM_2 ].ReportMode = RAMCFG_REPORT_NMI;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Config( &config );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Deinit() );

    TEST_ASSERT_TRUE( UtRamCfg_PrevNmiHandler == utRamCfg_NmiIsr );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_TWICE, utRamCfg_CoreSetCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_ECCNMI, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER );
    UtRamCfg_Check_NotInitialized();

    UtRamCfg_Init_Config( &config );
    utRamCfg_NvicSetCoreRet = NVIC_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Deinit() );
    UtRamCfg_Check_NotInitialized();
}

/* ============================== CHECK MODE ================================ */

/**
 * \brief   RamCfg_Set_CheckMode() enables the ECC of the SRAMs which have it.
 *
 * \details Initializes the module with the default configuration, sets the check mode ECC for every SRAM with ECC and
 *          reads it back.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the ECC enable bit set in the SRAM, RamCfg_Get_CheckMode() returns RAMCFG_CHECK_ECC.
 */
void Ut_RamCfg_Set_CheckMode_Ecc_EnablesEccOfSramsWithEcc( void )
{
    UtRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            ramCfg_CheckMode_t checkMode = RAMCFG_CHECK_NONE;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_CheckMode( sramId, RAMCFG_CHECK_ECC ) );

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE, UtRamCfg_Get_Inst( sramId )->CR & UT_RAMCFG_CR_ECCE );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_CheckMode( sramId, &checkMode ) );
            TEST_ASSERT_EQUAL( RAMCFG_CHECK_ECC, checkMode );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   RamCfg_Set_CheckMode() disables the ECC with the unlock sequence of the hardware.
 *
 * \details The HW model accepts the clearing of the ECC enable bit only after the keys 0xAE and 0x75. The ECC is
 *          enabled in every SRAM with ECC, then the check mode none is set.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the ECC enable bit cleared, the unlock sequence accepted once, the key register holds the second
 *   key, RamCfg_Get_CheckMode() returns RAMCFG_CHECK_NONE.
 */
void Ut_RamCfg_Set_CheckMode_None_UsesUnlockSequence( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            ramCfg_CheckMode_t checkMode = RAMCFG_CHECK_ECC;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_CheckMode( sramId, RAMCFG_CHECK_ECC ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_CheckMode( sramId, RAMCFG_CHECK_NONE ) );

            TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( sramId )->CR & UT_RAMCFG_CR_ECCE );
            TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_EccUnlockCnt[ sramId ] );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ECC_KEY_2, UtRamCfg_Get_Inst( sramId )->ECCKEY );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_CheckMode( sramId, &checkMode ) );
            TEST_ASSERT_EQUAL( RAMCFG_CHECK_NONE, checkMode );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   RamCfg_Set_CheckMode() refuses the ECC of an SRAM which has none.
 *
 * \details Requests the check mode ECC for every SRAM without ECC (the registers are filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, no register written.
 */
void Ut_RamCfg_Set_CheckMode_EccOnSramWithoutEcc_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( sramId, RAMCFG_CHECK_ECC ) );
        }
        else
        {
            /* The SRAM has ECC */
        }
    }

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_CheckMode() accepts the modes none and unchanged of an SRAM without ECC and the mode unchanged
 *          everywhere without any register write.
 *
 * \details Requests the check mode none for every SRAM without ECC and the check mode unchanged for every SRAM (the
 *          registers are filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, no register written.
 */
void Ut_RamCfg_Set_CheckMode_NoneOrUnchanged_ReturnsOkWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_CheckMode( sramId, RAMCFG_CHECK_UNCHANGED ) );

        if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_CheckMode( sramId, RAMCFG_CHECK_NONE ) );
        }
        else
        {
            /* The SRAM has ECC, the mode none writes the registers */
        }
    }

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_CheckMode() refuses the parity check, a value outside of the list and an SRAM outside of the
 *          list.
 *
 * \details Requests the parity check and an invalid value for SRAM2, the check mode ECC for the SRAM beyond the list
 *          and for the item "unused" (the registers are filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, no register written.
 */
void Ut_RamCfg_Set_CheckMode_InvalidRequest_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( RAMCFG_SRAM_2, RAMCFG_CHECK_PARITY ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( RAMCFG_SRAM_2, RAMCFG_CHECK_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( RAMCFG_SRAM_CNT, RAMCFG_CHECK_ECC ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( RAMCFG_SRAM_UNUSED, RAMCFG_CHECK_NONE ) );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_CheckMode() needs an initialized module.
 *
 * \details Requests the check mode ECC before the initialization (the registers are filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, no register written, no other module called.
 */
void Ut_RamCfg_Set_CheckMode_NotInitialized_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( RAMCFG_SRAM_2, RAMCFG_CHECK_ECC ) );

    UtRamCfg_Check_BlockUnchanged( block );
    UtRamCfg_Check_NoOtherModuleCalls();
}


/**
 * \brief   RamCfg_Set_CheckMode() detects an ECC enable bit which does not stay set and one which cannot be cleared.
 *
 * \details The HW model keeps the ECC enable bit at zero, then (ECC enabled) does not accept the unlock sequence.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases (read-back mismatch).
 */
void Ut_RamCfg_Set_CheckMode_ReadBackMismatch_ReturnsError( void )
{
    UtRamCfg_Init_Default();

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ECC_STUCK );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( RAMCFG_SRAM_2, RAMCFG_CHECK_ECC ) );

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ECC_LOCKED );
    UtRamCfg_Get_Inst( RAMCFG_SRAM_BACKUP )->CR = UT_RAMCFG_CR_ECCE;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( RAMCFG_SRAM_BACKUP, RAMCFG_CHECK_NONE ) );
}


/**
 * \brief   RamCfg_Get_CheckMode() returns the state of the ECC enable bit.
 *
 * \details Reads the check mode of every SRAM with the ECC enable bit clear and set. The bit of the SRAMs without ECC
 *          is set in the register (it is reserved there).
 *
 * \par Expected results
 * - RAMCFG_CHECK_NONE with the bit clear and RAMCFG_CHECK_ECC with the bit set for the SRAMs with ECC.
 * - RAMCFG_CHECK_NONE for the SRAMs without ECC whatever the register holds.
 * - The getter does not enable any clock.
 */
void Ut_RamCfg_Get_CheckMode_ReturnsEccState( void )
{
    UtRamCfg_Init_Default();
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect    = UtRamCfg_Get_Expect( sramId );
        ramCfg_CheckMode_t            checkMode = RAMCFG_CHECK_CNT;

        UtRamCfg_Get_Inst( sramId )->CR = 0u;

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_CheckMode( sramId, &checkMode ) );
        TEST_ASSERT_EQUAL( RAMCFG_CHECK_NONE, checkMode );

        UtRamCfg_Get_Inst( sramId )->CR = UT_RAMCFG_CR_ECCE;

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_CheckMode( sramId, &checkMode ) );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_CHECK_ECC, checkMode );
        }
        else
        {
            TEST_ASSERT_EQUAL( RAMCFG_CHECK_NONE, checkMode );
        }
    }

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}


/**
 * \brief   RamCfg_Get_CheckMode() refuses invalid arguments and a module which is not initialized, without side
 *          effects.
 *
 * \details Calls the getter with a NULL pointer, an SRAM beyond the list and before the initialization.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the output is not changed, no other module is called (no clock is enabled).
 */
void Ut_RamCfg_Get_CheckMode_InvalidRequest_ReturnsErrorOutputUnchanged( void )
{
    ramCfg_CheckMode_t checkMode = (ramCfg_CheckMode_t)UT_RAMCFG_SENTINEL;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_CheckMode( RAMCFG_SRAM_2, &checkMode ) );
    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, checkMode );

    UtRamCfg_Init_Default();
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_CheckMode( RAMCFG_SRAM_2, NULL ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_CheckMode( RAMCFG_SRAM_CNT, &checkMode ) );
    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, checkMode );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}

/* =========================== REPORT MODE ================================== */

/**
 * \brief   RamCfg_Set_ReportMode() enables the error interrupts and the address latch and activates the interrupt.
 *
 * \details Initializes the module without any reporting, then sets the reporting by the interrupt for every SRAM with
 *          ECC.
 *
 * \note    Device errata bug AB#1838 (ES0561 2.2.20 "SRAM ECC error flags and addresses are updated only if interrupt
 *          is enabled"): the failing address and the flags are stored only while the address latch and the error
 *          interrupt are enabled (documentation erratum of STM32H503); the test proves that the reporting by the
 *          interrupt enables the address latch together with the single and the double error interrupt.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, single and double error interrupts and the address latch enabled, RamCfg_Get_ReportMode()
 *   returns RAMCFG_REPORT_IRQ.
 * - The interrupt handler and the priority are passed to NVIC and the interrupt is activated.
 */
void Ut_RamCfg_Set_ReportMode_Irq_EnablesInterruptsAndActivatesVector( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_NONE;
    }

    UtRamCfg_Init_Config( &config );

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_ActiveCnt );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            ramCfg_ReportMode_t reportMode = RAMCFG_REPORT_NONE;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( sramId, RAMCFG_REPORT_IRQ ) );

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_IRQ, UtRamCfg_Get_Inst( sramId )->IER );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ALE, UtRamCfg_Get_Inst( sramId )->CR & UT_RAMCFG_CR_ALE );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode( sramId, &reportMode ) );
            TEST_ASSERT_EQUAL( RAMCFG_REPORT_IRQ, reportMode );
            TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_VectorActive );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }

    TEST_ASSERT_NOT_NULL( utRamCfg_Isr );
    TEST_ASSERT_TRUE( UT_RAMCFG_CALLS_NONE < utRamCfg_HandlerCnt );
    TEST_ASSERT_TRUE( UT_RAMCFG_CALLS_NONE < utRamCfg_PrioCnt );
}


/**
 * \brief   RamCfg_Set_ReportMode() deactivates the interrupt when the last reported SRAM is switched off.
 *
 * \details Initializes the module with the default configuration (every SRAM with ECC reported by the interrupt),
 *          switches off the reporting of the SRAMs with ECC one after another.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the error interrupts and the address latch of the switched off SRAM cleared.
 * - The interrupt stays active while at least one SRAM is reported and is inactive after the last one.
 */
void Ut_RamCfg_Set_ReportMode_None_VectorFollowsReportedSrams( void )
{
    uint32_t eccCnt = 0u;

    UtRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            eccCnt ++;
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            eccCnt --;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( sramId, RAMCFG_REPORT_NONE ) );

            TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( sramId )->IER );
            TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( sramId )->CR & UT_RAMCFG_CR_ALE );

            if( 0u < eccCnt )
            {
                TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_VectorActive );
            }
            else
            {
                TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_CLEAR, utRamCfg_VectorActive );
            }
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   RamCfg_Set_ReportMode() takes over the NMI vector for the reporting by the NMI.
 *
 * \details Sets the reporting by the NMI for SRAM2, then for the backup SRAM.
 *
 * \note    Device errata bug AB#1838 (ES0561 2.2.20 "SRAM ECC error flags and addresses are updated only if interrupt
 *          is enabled"): the failing address and the flags are stored only while the address latch and the error
 *          interrupt are enabled (documentation erratum of STM32H503); the test proves that the reporting by the NMI
 *          enables the address latch together with the single error interrupt and the NMI redirection.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, single error interrupt and NMI redirection set, double error interrupt clear, address latch set,
 *   RamCfg_Get_ReportMode() returns RAMCFG_REPORT_NMI.
 * - The NMI handler of the module is installed once (before the redirection is enabled) and not again for the second
 *   SRAM.
 */
void Ut_RamCfg_Set_ReportMode_Nmi_TakesOverNmiVector( void )
{
    ramCfg_ReportMode_t reportMode = RAMCFG_REPORT_NONE;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_NMI ) );

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_NMI, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ALE, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->CR & UT_RAMCFG_CR_ALE );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode( RAMCFG_SRAM_2, &reportMode ) );
    TEST_ASSERT_EQUAL( RAMCFG_REPORT_NMI, reportMode );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_CoreSetCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_NmiInstalledAtEnable );
    TEST_ASSERT_NOT_NULL( utRamCfg_NmiIsr );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( RAMCFG_SRAM_BACKUP, RAMCFG_REPORT_NMI ) );

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_NMI, UtRamCfg_Get_Inst( RAMCFG_SRAM_BACKUP )->IER );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_CoreSetCnt );
}


/**
 * \brief   RamCfg_Set_ReportMode() refuses to leave the NMI mode (the NMI redirection cannot be cleared by software).
 *
 * \details Sets the reporting by the NMI for SRAM2, requests the reporting by the interrupt and none (the registers are
 *          compared before and after), requests the NMI again.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR for the other modes, no register written, RamCfg_Get_ReportMode() still returns
 *   RAMCFG_REPORT_NMI.
 * - RAMCFG_REQUEST_OK for the NMI mode again.
 */
void Ut_RamCfg_Set_ReportMode_LeavingNmi_ReturnsErrorWithoutWrite( void )
{
    uint32_t            block[ UT_RAMCFG_BLOCK_WORDS ];
    ramCfg_ReportMode_t reportMode = RAMCFG_REPORT_NONE;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_NMI ) );

    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_NONE ) );

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode( RAMCFG_SRAM_2, &reportMode ) );
    TEST_ASSERT_EQUAL( RAMCFG_REPORT_NMI, reportMode );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_NMI ) );
}


/**
 * \brief   RamCfg_Set_ReportMode() of an SRAM without ECC accepts only "no reporting".
 *
 * \details Requests the reporting none, by the interrupt and by the NMI for every SRAM without ECC (the registers are
 *          filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK for none, RAMCFG_REQUEST_ERROR for the others, no register written.
 */
void Ut_RamCfg_Set_ReportMode_SramWithoutEcc_AcceptsOnlyNone( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( sramId, RAMCFG_REPORT_NONE ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( sramId, RAMCFG_REPORT_IRQ ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( sramId, RAMCFG_REPORT_NMI ) );
        }
        else
        {
            /* The SRAM has ECC */
        }
    }

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_ReportMode() refuses invalid requests and a module which is not initialized.
 *
 * \details Requests a value outside of the list, an SRAM beyond the list and a mode before the initialization (the
 *          registers are filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, no register written, no other module called before the initialization.
 */
void Ut_RamCfg_Set_ReportMode_InvalidRequest_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );
    UtRamCfg_Check_BlockUnchanged( block );
    UtRamCfg_Check_NoOtherModuleCalls();

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_CNT, RAMCFG_REPORT_IRQ ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_UNUSED, RAMCFG_REPORT_NONE ) );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_ReportMode() detects error interrupt enables and an address latch which do not stay set.
 *
 * \details Initializes the module without reporting, the HW model keeps the error interrupt enables at zero, then the
 *          address latch at zero, and the reporting by the interrupt is requested.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases (read-back mismatch), the interrupt is not activated.
 */
void Ut_RamCfg_Set_ReportMode_ReadBackMismatch_ReturnsError( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_NONE;
    }

    UtRamCfg_Init_Config( &config );

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_IER_NOT_KEPT );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ALE_NOT_KEPT );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_ActiveCnt );
}


/**
 * \brief   RamCfg_Set_ReportMode() reports a refusal of the interrupt activation.
 *
 * \details Initializes the module without reporting, the NVIC stub refuses the activation, the reporting by the
 *          interrupt is requested.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR.
 */
void Ut_RamCfg_Set_ReportMode_NvicRefusal_ReturnsError( void )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        config.Sram[ sramId ].ReportMode = RAMCFG_REPORT_NONE;
    }

    UtRamCfg_Init_Config( &config );

    utRamCfg_NvicActiveRet = NVIC_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );
}


/**
 * \brief   RamCfg_Get_ReportMode() returns the reporting mode from the interrupt enable bits.
 *
 * \details Writes the enable bits of SRAM2 for none, the interrupt and the NMI, and combinations which are not a mode
 *          of the module (single error interrupt only, double error interrupt only, NMI redirection without the single
 *          error interrupt). The enable bits of the SRAMs without ECC are set in the register (reserved there).
 *
 * \par Expected results
 * - RAMCFG_REPORT_NONE, RAMCFG_REPORT_IRQ and RAMCFG_REPORT_NMI for the three modes.
 * - RAMCFG_REQUEST_ERROR and the output not changed for the other combinations.
 * - RAMCFG_REPORT_NONE for an SRAM without ECC.
 * - The getter does not enable any clock.
 */
void Ut_RamCfg_Get_ReportMode_ReturnsModeOfEnableBits( void )
{
    ramCfg_ReportMode_t   reportMode = RAMCFG_REPORT_NONE;
    const uint32_t        modeBits[]   = { 0u, UT_RAMCFG_IER_IRQ, UT_RAMCFG_IER_NMI };
    const ramCfg_ReportMode_t modeList[] = { RAMCFG_REPORT_NONE, RAMCFG_REPORT_IRQ, RAMCFG_REPORT_NMI };
    const uint32_t        otherBits[]  = { UT_RAMCFG_IER_SEIE, UT_RAMCFG_IER_DEIE, UT_RAMCFG_IER_ECCNMI,
                                           UT_RAMCFG_IER_DEIE | UT_RAMCFG_IER_ECCNMI };

    UtRamCfg_Init_Default();
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    for( uint32_t modeIdx = 0u; ( sizeof( modeBits ) / sizeof( modeBits[ 0u ] ) ) > modeIdx; modeIdx ++ )
    {
        reportMode = RAMCFG_REPORT_CNT;

        UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER = modeBits[ modeIdx ];

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode( RAMCFG_SRAM_2, &reportMode ) );
        TEST_ASSERT_EQUAL( modeList[ modeIdx ], reportMode );
    }

    for( uint32_t otherIdx = 0u; ( sizeof( otherBits ) / sizeof( otherBits[ 0u ] ) ) > otherIdx; otherIdx ++ )
    {
        reportMode = (ramCfg_ReportMode_t)UT_RAMCFG_SENTINEL;

        UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER = otherBits[ otherIdx ];

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ReportMode( RAMCFG_SRAM_2, &reportMode ) );
        TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, reportMode );
    }

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            reportMode = RAMCFG_REPORT_CNT;

            UtRamCfg_Get_Inst( sramId )->IER = UT_RAMCFG_IER_IRQ;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ReportMode( sramId, &reportMode ) );
            TEST_ASSERT_EQUAL( RAMCFG_REPORT_NONE, reportMode );
        }
        else
        {
            /* The SRAM has ECC */
        }
    }

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}


/**
 * \brief   RamCfg_Get_ReportMode() refuses invalid arguments and a module which is not initialized, without side
 *          effects.
 *
 * \details Calls the getter with a NULL pointer, an SRAM beyond the list and before the initialization.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the output is not changed, no other module is called.
 */
void Ut_RamCfg_Get_ReportMode_InvalidRequest_ReturnsErrorOutputUnchanged( void )
{
    ramCfg_ReportMode_t reportMode = (ramCfg_ReportMode_t)UT_RAMCFG_SENTINEL;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ReportMode( RAMCFG_SRAM_2, &reportMode ) );
    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, reportMode );

    UtRamCfg_Init_Default();
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ReportMode( RAMCFG_SRAM_2, NULL ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ReportMode( RAMCFG_SRAM_CNT, &reportMode ) );
    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, reportMode );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}

/* ============================== SCRUBBING ================================= */

/**
 * \brief   RamCfg_Set_Scrubbing() accepts the scrubbing of the SRAMs with ECC only.
 *
 * \details Activates and deactivates the scrubbing of every SRAM (the registers are filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK for the activation and the deactivation of an SRAM with ECC and for the deactivation of an SRAM
 *   without ECC, RAMCFG_REQUEST_ERROR for the activation of an SRAM without ECC. No register is written.
 */
void Ut_RamCfg_Set_Scrubbing_AcceptsEccSramsOnly( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( sramId, RAMCFG_FUNCTION_INACTIVE ) );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( sramId, RAMCFG_FUNCTION_ACTIVE ) );
        }
        else
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Scrubbing( sramId, RAMCFG_FUNCTION_ACTIVE ) );
        }
    }

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_Scrubbing() refuses invalid requests and a module which is not initialized.
 *
 * \details Requests a state outside of the list, an SRAM beyond the list and the activation before the initialization.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case.
 */
void Ut_RamCfg_Set_Scrubbing_InvalidRequest_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Scrubbing( RAMCFG_SRAM_2, RAMCFG_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Scrubbing( RAMCFG_SRAM_2, RAMCFG_FUNCTION_INACTIVE ) );

    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Set_Scrubbing(
                           RAMCFG_SRAM_2,
                           (ramCfg_FunctionState_t)( RAMCFG_FUNCTION_ACTIVE + UT_RAMCFG_ID_OFFSET ) ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Scrubbing( RAMCFG_SRAM_CNT, RAMCFG_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Scrubbing( RAMCFG_SRAM_UNUSED, RAMCFG_FUNCTION_INACTIVE ) );
}

/* ========================== ERROR HANDLING (ISR) ========================== */

/**
 * \brief   The interrupt handler services a single error: counts it, records the address, acknowledges the flag, calls
 *          the callback.
 *
 * \details Initializes the module with the callbacks, raises a single error in SRAM2 and runs the handler registered in
 *          NVIC.
 *
 * \par Expected results
 * - The single error counter is 1, the double error counter 0, the last error is the one of SRAM2 with the reported
 *   address.
 * - The flag SEDC is cleared by the write of the clear bit and was already cleared when the callback was called, the
 *   callback was called once with SRAM2 and the address.
 */
void Ut_RamCfg_Isr_SingleError_CountsRecordsAndCallsCallback( void )
{
    ramCfg_ErrorStatistics_t statistics;
    const uint32_t           address = UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ );

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, address );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_2, statistics.LastErrorSram );
    TEST_ASSERT_EQUAL_HEX32( address, statistics.LastErrorAddress );

    TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR & UT_RAMCFG_ISR_SEDC );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_SingleCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_2, utRamCfg_SingleSram );
    TEST_ASSERT_EQUAL_HEX32( address, utRamCfg_SingleAddr );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_CLEAR, utRamCfg_SingleFlagAtCall );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_DoubleCnt );
}


/**
 * \brief   The interrupt handler services a double error: counts it, records the address, acknowledges the flag, calls
 *          the callback.
 *
 * \details Initializes the module with the callbacks, raises a double error in SRAM2 and runs the handler registered in
 *          NVIC.
 *
 * \par Expected results
 * - The double error counter is 1, the single error counter 0, the last error is the one of SRAM2 with the reported
 *   address.
 * - The flag DED is cleared and was already cleared when the callback was called, the callback was called once with
 *   SRAM2 and the address.
 */
void Ut_RamCfg_Isr_DoubleError_CountsRecordsAndCallsCallback( void )
{
    ramCfg_ErrorStatistics_t statistics;
    const uint32_t           address = UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET_2;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ );

    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, address );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_2, statistics.LastErrorSram );
    TEST_ASSERT_EQUAL_HEX32( address, statistics.LastErrorAddress );

    TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR & UT_RAMCFG_ISR_DED );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_DoubleCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_2, utRamCfg_DoubleSram );
    TEST_ASSERT_EQUAL_HEX32( address, utRamCfg_DoubleAddr );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_CLEAR, utRamCfg_DoubleFlagAtCall );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_SingleCnt );
}


/**
 * \brief   The interrupt handler services a single and a double error flagged together.
 *
 * \details The bits of the clear register read as the flags (HW model): a read-modify-write of the clear register would
 *          clear both flags. Raises both errors in SRAM2 and runs the handler once.
 *
 * \par Expected results
 * - Both errors are counted, both callbacks called once, both flags cleared.
 */
void Ut_RamCfg_Isr_SingleAndDoubleError_AreServicedIndependently( void )
{
    ramCfg_ErrorStatistics_t statistics;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ );

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET );
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET_2 );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_SingleCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_DoubleCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR & ( UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED ) );
}


/**
 * \brief   The interrupt handler does nothing when no error is flagged.
 *
 * \details Runs the handler registered in NVIC without any flag set.
 *
 * \par Expected results
 * - No callback, the clear registers are not written, the statistics stay clear.
 */
void Ut_RamCfg_Isr_NoFlag_DoesNothing( void )
{
    ramCfg_ErrorStatistics_t statistics;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ );

    utRamCfg_ModelCalls = 0u;

    utRamCfg_Isr();

    TEST_ASSERT_EQUAL_UINT32( 0u, utRamCfg_ModelCalls );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_SingleCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_DoubleCnt );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.DoubleErrorCnt );
}


/**
 * \brief   The interrupt handler does not service the errors of an SRAM which is not reported.
 *
 * \details Initializes the module with no reporting for SRAM2, raises a single and a double error in SRAM2 and runs the
 *          handler.
 *
 * \note    Device errata bug AB#1838 (ES0561 2.2.20 "SRAM ECC error flags and addresses are updated only if interrupt
 *          is enabled"): the hardware stores the failing address and the flags only while the error interrupt is
 *          enabled (documentation erratum of STM32H503), so an SRAM without reporting has no usable error data; the
 *          test proves that the handler neither counts nor acknowledges the errors of such an SRAM.
 *
 * \par Expected results
 * - No callback, the flags stay set (the clear registers are not written), the statistics of SRAM2 stay clear.
 */
void Ut_RamCfg_Isr_ReportNone_ErrorsNotServiced( void )
{
    ramCfg_ErrorStatistics_t statistics;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_NONE );

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base );
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_SingleCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_DoubleCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.DoubleErrorCnt );
}


/**
 * \brief   The interrupt handler services every SRAM with ECC with its own registers.
 *
 * \details Initializes the module with the default reporting and the callbacks, raises a single error with a different
 *          address in every SRAM with ECC, runs the handler once.
 *
 * \par Expected results
 * - One callback per SRAM with ECC, every SRAM has one single error with its own address, the other counters are clear.
 */
void Ut_RamCfg_Isr_EverySramWithEcc_IsServicedWithOwnRegisters( void )
{
    uint32_t eccCnt = 0u;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            UtRamCfg_Raise_Single( sramId, expect->Base + ( UT_RAMCFG_WORD_OFFSET * ( sramId + 1u ) ) );
            eccCnt ++;
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }

    utRamCfg_Isr();

    TEST_ASSERT_EQUAL_UINT32( eccCnt, utRamCfg_SingleCnt );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            ramCfg_ErrorStatistics_t statistics;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( sramId, &statistics ) );
            TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.SingleErrorCnt );
            TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, statistics.DoubleErrorCnt );
            TEST_ASSERT_EQUAL( sramId, statistics.LastErrorSram );
            TEST_ASSERT_EQUAL_HEX32( expect->Base + ( UT_RAMCFG_WORD_OFFSET * ( sramId + 1u ) ),
                                     statistics.LastErrorAddress );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   The interrupt handler counts the errors when no callback is configured.
 *
 * \details Initializes the module with the default configuration (no callbacks), raises a single and a double error in
 *          SRAM2, runs the handler.
 *
 * \par Expected results
 * - Both errors counted, both flags cleared.
 */
void Ut_RamCfg_Isr_NoCallbacks_CountsOnly( void )
{
    ramCfg_ErrorStatistics_t statistics;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base );
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR & ( UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED ) );
}


/**
 * \brief   With the reporting by the NMI the single error is serviced by the interrupt handler and the double error by
 *          the NMI handler.
 *
 * \details Initializes the module with the reporting by the NMI for SRAM2, raises a single and a double error in SRAM2,
 *          runs the interrupt handler, then the NMI handler registered in NVIC, then the NMI handler without a flag.
 *
 * \par Expected results
 * - The interrupt handler services the single error only (the flag DED stays set, no double error callback).
 * - The NMI handler services the double error (counter, callback, flag cleared) and does not count the single error
 *   again.
 * - The NMI handler without a flag does nothing.
 */
void Ut_RamCfg_Isr_NmiMode_SingleByInterruptDoubleByNmi( void )
{
    ramCfg_ErrorStatistics_t statistics;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_NMI );

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET );
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET_2 );

    utRamCfg_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_SingleCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_DoubleCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ISR_DED,
                             UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR & ( UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED ) );

    TEST_ASSERT_NOT_NULL( utRamCfg_NmiIsr );
    utRamCfg_NmiIsr();

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_SingleCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_DoubleCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR & UT_RAMCFG_ISR_DED );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, statistics.DoubleErrorCnt );

    utRamCfg_ModelCalls = 0u;
    utRamCfg_NmiIsr();

    TEST_ASSERT_EQUAL_UINT32( 0u, utRamCfg_ModelCalls );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_DoubleCnt );
}

/* ============================== SCRUBBING ================================= */

/**
 * \brief   The scrubbing writes the failing word back before the flag is acknowledged.
 *
 * \details The failing word is in the backup SRAM, whose registers and memory lie in the peripheral region of RegMem:
 *          the HW model sees every store. Runs the same single error with the scrubbing inactive and active.
 *
 * \par Expected results
 * - The scrubbing adds exactly one store (the write-back) to the stores of the handler.
 * - The acknowledgement of the flag is the last store of the handler (the write-back is done before it).
 */
void Ut_RamCfg_Isr_Scrubbing_WritesFailingWordBack( void )
{
    const uint32_t address = UT_RAMCFG_BKP_BASE + UT_RAMCFG_WORD_OFFSET;
    uint32_t       storesOff;
    uint32_t       storesOn;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( RAMCFG_SRAM_BACKUP, RAMCFG_FUNCTION_INACTIVE ) );
    storesOff = UtRamCfg_Run_SingleIsr( RAMCFG_SRAM_BACKUP, address );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( RAMCFG_SRAM_BACKUP, RAMCFG_FUNCTION_ACTIVE ) );
    storesOn = UtRamCfg_Run_SingleIsr( RAMCFG_SRAM_BACKUP, address );

    TEST_ASSERT_EQUAL_UINT32( storesOff + 1u, storesOn );
    TEST_ASSERT_EQUAL_UINT32( utRamCfg_ModelCalls, utRamCfg_ClearAtCall );
}


/**
 * \brief   The scrubbing accepts the address of the hardware as an address of the memory map in both aliases and as an
 *          offset.
 *
 * \details The failing word is in the backup SRAM (the HW model sees every store). Reports the address in the
 *          non-secure alias, in the secure alias and as an offset in the SRAM, with the scrubbing active.
 *
 * \par Expected results
 * - In every form the word is written back (one store more than without the scrubbing).
 */
void Ut_RamCfg_Isr_Scrubbing_AcceptsAliasesAndOffset( void )
{
    const uint32_t addressList[] =
    {
        UT_RAMCFG_BKP_BASE + UT_RAMCFG_WORD_OFFSET,
        ( UT_RAMCFG_BKP_BASE | UT_RAMCFG_SECURE_ALIAS ) + UT_RAMCFG_WORD_OFFSET,
        UT_RAMCFG_WORD_OFFSET
    };
    uint32_t storesOff;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( RAMCFG_SRAM_BACKUP, RAMCFG_FUNCTION_INACTIVE ) );
    storesOff = UtRamCfg_Run_SingleIsr( RAMCFG_SRAM_BACKUP, UT_RAMCFG_BKP_BASE );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( RAMCFG_SRAM_BACKUP, RAMCFG_FUNCTION_ACTIVE ) );

    for( uint32_t addrIdx = 0u; ( sizeof( addressList ) / sizeof( addressList[ 0u ] ) ) > addrIdx; addrIdx ++ )
    {
        TEST_ASSERT_EQUAL_UINT32( storesOff + 1u,
                                  UtRamCfg_Run_SingleIsr( RAMCFG_SRAM_BACKUP, addressList[ addrIdx ] ) );
    }
}


/**
 * \brief   The scrubbing does not write an address which is not in the SRAM.
 *
 * \details Reports the first address behind the backup SRAM and the highest address, with the scrubbing active.
 *
 * \par Expected results
 * - No write-back (the same count of stores as without the scrubbing), the error is counted and the callback is called.
 */
void Ut_RamCfg_Isr_Scrubbing_AddressOutsideSram_IsNotWritten( void )
{
    const uint32_t addressList[] =
    {
        UT_RAMCFG_BKP_BASE + UT_RAMCFG_BKP_SIZE,
        RAMCFG_ADDRESS_UNUSED
    };
    uint32_t storesOff;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_BACKUP, RAMCFG_REPORT_IRQ );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( RAMCFG_SRAM_BACKUP, RAMCFG_FUNCTION_INACTIVE ) );
    storesOff = UtRamCfg_Run_SingleIsr( RAMCFG_SRAM_BACKUP, UT_RAMCFG_BKP_BASE );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Scrubbing( RAMCFG_SRAM_BACKUP, RAMCFG_FUNCTION_ACTIVE ) );

    for( uint32_t addrIdx = 0u; ( sizeof( addressList ) / sizeof( addressList[ 0u ] ) ) > addrIdx; addrIdx ++ )
    {
        TEST_ASSERT_EQUAL_UINT32( storesOff, UtRamCfg_Run_SingleIsr( RAMCFG_SRAM_BACKUP, addressList[ addrIdx ] ) );
    }

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE + UT_RAMCFG_CALLS_TWICE, utRamCfg_SingleCnt );
}


/**
 * \brief   The scrubbing of a word in an SRAM keeps the data.
 *
 * \details A word of SRAM2 (plain memory of RegMem) holds a pattern. Reports a single error at the word with the
 *          scrubbing active and runs the handler.
 *
 * \par Expected results
 * - The word holds the same pattern (the write-back writes the value which was read), the error is counted.
 */
void Ut_RamCfg_Isr_Scrubbing_KeepsDataOfSram( void )
{
    volatile uint32_t * const word =
        (volatile uint32_t *)(uintptr_t)( UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET );

    *word = UT_RAMCFG_DATA_PATTERN;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Sram( RAMCFG_SRAM_2, RAMCFG_CHECK_UNCHANGED, RAMCFG_REPORT_IRQ, RAMCFG_FUNCTION_ACTIVE );

    (void)UtRamCfg_Run_SingleIsr( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_WORD_OFFSET );

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_DATA_PATTERN, *word );
}

/* ============================== STATISTICS ================================ */

/**
 * \brief   RamCfg_Get_ErrorStatistics() returns the counters and the last error of the SRAM.
 *
 * \details Records three single errors and two double errors in SRAM2 (single, double, single, double, single), one
 *          error in the backup SRAM, then another double error in SRAM2.
 *
 * \par Expected results
 * - SRAM2: three single and two double errors, the last error is the last one recorded in SRAM2 (address and SRAM),
 *   after the further double error three and three.
 * - The backup SRAM has one error of its own, the statistics of the SRAMs are independent.
 */
void Ut_RamCfg_Get_ErrorStatistics_AfterErrors_ReturnsCountersAndLastError( void )
{
    ramCfg_ErrorStatistics_t statistics;
    const uint32_t           base = UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, base + UT_RAMCFG_ERR_OFFSET_1 );
    utRamCfg_Isr();
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, base + UT_RAMCFG_ERR_OFFSET_2 );
    utRamCfg_Isr();
    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, base + UT_RAMCFG_ERR_OFFSET_3 );
    utRamCfg_Isr();
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, base + UT_RAMCFG_ERR_OFFSET_4 );
    utRamCfg_Isr();
    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, base + UT_RAMCFG_ERR_OFFSET_5 );
    utRamCfg_Isr();
    UtRamCfg_Raise_Single( RAMCFG_SRAM_BACKUP, UT_RAMCFG_BKP_BASE + UT_RAMCFG_BKP_ERR_OFFSET );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_ERROR_CNT_THREE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_ERROR_CNT_TWO, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_2, statistics.LastErrorSram );
    TEST_ASSERT_EQUAL_HEX32( base + UT_RAMCFG_ERR_OFFSET_5, statistics.LastErrorAddress );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_BACKUP, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_BACKUP, statistics.LastErrorSram );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_BKP_BASE + UT_RAMCFG_BKP_ERR_OFFSET, statistics.LastErrorAddress );

    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, base + UT_RAMCFG_ERR_OFFSET_6 );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_ERROR_CNT_THREE, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_ERROR_CNT_THREE, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( base + UT_RAMCFG_ERR_OFFSET_6, statistics.LastErrorAddress );
}


/**
 * \brief   RamCfg_Get_ErrorStatistics() without any recorded error returns zero counters and no address.
 *
 * \details Initializes the module and reads the statistics of every SRAM with ECC.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, both counters zero, the last SRAM "unused", the last address "unused".
 */
void Ut_RamCfg_Get_ErrorStatistics_NoError_ReturnsZeroAndUnusedAddress( void )
{
    UtRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            ramCfg_ErrorStatistics_t statistics;

            (void)memset( &statistics, UT_RAMCFG_SENTINEL, sizeof( statistics ) );

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( sramId, &statistics ) );
            TEST_ASSERT_EQUAL_UINT32( 0u, statistics.SingleErrorCnt );
            TEST_ASSERT_EQUAL_UINT32( 0u, statistics.DoubleErrorCnt );
            TEST_ASSERT_EQUAL( RAMCFG_SRAM_UNUSED, statistics.LastErrorSram );
            TEST_ASSERT_EQUAL_HEX32( RAMCFG_ADDRESS_UNUSED, statistics.LastErrorAddress );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   The functions of the error statistics refuse invalid requests without side effects.
 *
 * \details Calls the statistics getter, the clear function and the failing address getter before the initialization,
 *          with a NULL pointer, for an SRAM beyond the list and for every SRAM without ECC (the outputs hold a
 *          sentinel).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the outputs are not changed.
 */
void Ut_RamCfg_ErrorStatistics_InvalidRequest_ReturnsErrorOutputUnchanged( void )
{
    ramCfg_ErrorStatistics_t statistics;
    ramCfg_Address_t         address = UT_RAMCFG_SENTINEL;

    (void)memset( &statistics, UT_RAMCFG_SENTINEL, sizeof( statistics ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ErrorStatisticsClear( RAMCFG_SRAM_2 ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_FailingAddress( RAMCFG_SRAM_2, &address ) );

    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, NULL ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_FailingAddress( RAMCFG_SRAM_2, NULL ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_CNT, &statistics ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ErrorStatisticsClear( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_FailingAddress( RAMCFG_SRAM_UNUSED, &address ) );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_INACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_ErrorStatistics( sramId, &statistics ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_ErrorStatisticsClear( sramId ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_FailingAddress( sramId, &address ) );
        }
        else
        {
            /* The SRAM has ECC */
        }
    }

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_SENTINEL_WORD, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_SENTINEL_WORD, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_SENTINEL_WORD, statistics.LastErrorAddress );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_SENTINEL, address );
}


/**
 * \brief   RamCfg_Set_ErrorStatisticsClear() clears the counters and the address of an SRAM only.
 *
 * \details Records a single and a double error in SRAM2 and in the backup SRAM, clears the statistics of SRAM2.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the statistics of SRAM2 are clear (the address "unused"), the statistics of the backup SRAM
 *   unchanged.
 */
void Ut_RamCfg_Set_ErrorStatisticsClear_ClearsOneSram( void )
{
    ramCfg_ErrorStatistics_t statistics;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base );
    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base );
    UtRamCfg_Raise_Single( RAMCFG_SRAM_BACKUP, UT_RAMCFG_BKP_BASE );
    UtRamCfg_Raise_Double( RAMCFG_SRAM_BACKUP, UT_RAMCFG_BKP_BASE );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ErrorStatisticsClear( RAMCFG_SRAM_2 ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_2, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, statistics.DoubleErrorCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_UNUSED, statistics.LastErrorSram );
    TEST_ASSERT_EQUAL_HEX32( RAMCFG_ADDRESS_UNUSED, statistics.LastErrorAddress );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_ErrorStatistics( RAMCFG_SRAM_BACKUP, &statistics ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, statistics.SingleErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, statistics.DoubleErrorCnt );
}


/**
 * \brief   RamCfg_Get_FailingAddress() returns the address of the last error of the SRAM.
 *
 * \details Reads the address before any error, after a single error and after a double error of SRAM2, and for the
 *          backup SRAM.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the address "unused" before an error, then the address of the last error (single or double),
 *   the backup SRAM keeps its own state.
 */
void Ut_RamCfg_Get_FailingAddress_ReturnsAddressOfLastError( void )
{
    ramCfg_Address_t address = 0u;
    const uint32_t   base    = UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_FailingAddress( RAMCFG_SRAM_2, &address ) );
    TEST_ASSERT_EQUAL_HEX32( RAMCFG_ADDRESS_UNUSED, address );

    UtRamCfg_Raise_Single( RAMCFG_SRAM_2, base + UT_RAMCFG_ERR_OFFSET_1 );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_FailingAddress( RAMCFG_SRAM_2, &address ) );
    TEST_ASSERT_EQUAL_HEX32( base + UT_RAMCFG_ERR_OFFSET_1, address );

    UtRamCfg_Raise_Double( RAMCFG_SRAM_2, base + UT_RAMCFG_ERR_OFFSET_2 );
    utRamCfg_Isr();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_FailingAddress( RAMCFG_SRAM_2, &address ) );
    TEST_ASSERT_EQUAL_HEX32( base + UT_RAMCFG_ERR_OFFSET_2, address );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_FailingAddress( RAMCFG_SRAM_BACKUP, &address ) );
    TEST_ASSERT_EQUAL_HEX32( RAMCFG_ADDRESS_UNUSED, address );
}

/* ========================== WRITE PROTECTION ============================== */

/**
 * \brief   RamCfg_Set_WriteProtection() protects the first page of SRAM2.
 *
 * \details Initializes the module and protects the page 0.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the bit of the page 0 is set (0x00000001 in the first write protection register), no other page
 *   is protected.
 */
void Ut_RamCfg_Set_WriteProtection_FirstPage_SetsBitOfPage( void )
{
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, 1u ) );

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BIT_FIRST, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR1_IDX ) );
    UtRamCfg_Check_Wpr( RAMCFG_SRAM_2, 0u, 1u );
}


/**
 * \brief   RamCfg_Set_WriteProtection() protects the last page of SRAM2.
 *
 * \details Initializes the module and protects the last page of the SRAM2 of the line.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, only the bit of the last page is set (the bit of the page modulo 32 of the register of the
 *   page).
 */
void Ut_RamCfg_Set_WriteProtection_LastPage_SetsBitOfPage( void )
{
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, UT_RAMCFG_SRAM2_PAGES - 1u, 1u ) );

    UtRamCfg_Check_Wpr( RAMCFG_SRAM_2, UT_RAMCFG_SRAM2_PAGES - 1u, 1u );
}


/**
 * \brief   RamCfg_Set_WriteProtection() protects all pages of SRAM2 in all write protection registers of the line.
 *
 * \details Initializes the module and protects the whole SRAM2.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, all bits of the pages of the line are set (the registers beyond the last page are not written).
 */
void Ut_RamCfg_Set_WriteProtection_AllPages_SetsAllBitsOfLine( void )
{
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, UT_RAMCFG_SRAM2_PAGES ) );

    UtRamCfg_Check_Wpr( RAMCFG_SRAM_2, 0u, UT_RAMCFG_SRAM2_PAGES );
}


/**
 * \brief   RamCfg_Set_WriteProtection() protects a range which crosses the border of two write protection registers.
 *
 * \details Initializes the module and protects the pages 31 and 32 (lines with more than 32 pages), 63 and 64 (more
 *          than 64 pages), 95 and 96 (more than 96 pages) one after another.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the page 31 is the bit 31 of the first register, the page 32 the bit 0 of the second one and so
 *   on.
 * - Lines with one write protection register: ignored.
 */
void Ut_RamCfg_Set_WriteProtection_AcrossRegisterBorder_SetsBitsInBothRegisters( void )
{
#if ( UT_RAMCFG_SRAM2_PAGES > UT_RAMCFG_PAGES_PER_WPR )
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                   UT_RAMCFG_PAGE_BORDER_1,
                                                   UT_RAMCFG_RANGE_TWO_PAGES ) );

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BIT_LAST, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR1_IDX ) );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BIT_FIRST, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR2_IDX ) );

    #if ( UT_RAMCFG_SRAM2_PAGES > UT_RAMCFG_PAGES_WPR_1_2 )
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                       UT_RAMCFG_PAGE_BORDER_2,
                                                       UT_RAMCFG_RANGE_TWO_PAGES ) );

        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BIT_LAST_AND_FIRST,
                                 UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR2_IDX ) );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BIT_FIRST, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR3_IDX ) );
    #endif

    #if ( UT_RAMCFG_SRAM2_PAGES > UT_RAMCFG_PAGES_WPR_1_3 )
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                       UT_RAMCFG_PAGE_BORDER_3,
                                                       UT_RAMCFG_RANGE_TWO_PAGES ) );

        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BIT_LAST_AND_FIRST,
                                 UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR3_IDX ) );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BIT_FIRST, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR4_IDX ) );
    #endif
#else
    TEST_IGNORE_MESSAGE( "SRAM2 of the line has one write protection register" );
#endif
}


/**
 * \brief   RamCfg_Set_WriteProtection() sets the bits of the pages 56 - 62 (and 88 - 94) at the positions of the
 *          reference manual.
 *
 * \details The CMSIS device headers define these page bits one position too high. Initializes the module and protects
 *          the pages 56 - 62 (lines with more than 62 pages) and the pages 88 - 94 (lines with more than 94 pages).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the bits 24 - 30 (0x7F000000) of the second (third) write protection register are set, the bit
 *   31 is clear.
 * - Lines with 62 pages or less: ignored.
 */
void Ut_RamCfg_Set_WriteProtection_PagesAboveBit23_UseReferenceManualBits( void )
{
#if ( UT_RAMCFG_SRAM2_PAGES > UT_RAMCFG_PAGE_LAST_HIGH_WPR2 )
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, UT_RAMCFG_PAGE_HIGH_WPR2, UT_RAMCFG_PAGES_HIGH ) );

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BITS_24_30, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR2_IDX ) );

    #if ( UT_RAMCFG_SRAM2_PAGES > UT_RAMCFG_PAGE_LAST_HIGH_WPR3 )
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                           RamCfg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                       UT_RAMCFG_PAGE_HIGH_WPR3,
                                                       UT_RAMCFG_PAGES_HIGH ) );

        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_BITS_24_30, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR3_IDX ) );
    #endif
#else
    TEST_IGNORE_MESSAGE( "SRAM2 of the line has no page above 62" );
#endif
}


/**
 * \brief   RamCfg_Set_WriteProtection() keeps the pages which are protected already.
 *
 * \details Initializes the module, protects the pages 0 - 1, then the pages 1 - 2.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK both times, the pages 0 - 2 are protected (0x00000007), nothing is removed.
 */
void Ut_RamCfg_Set_WriteProtection_Repeated_KeepsProtectedPages( void )
{
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, UT_RAMCFG_RANGE_TWO_PAGES ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES ) );

    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_WPR_PAGES_0_2, UtRamCfg_Get_WprReg( RAMCFG_SRAM_2, UT_RAMCFG_WPR1_IDX ) );
}


/**
 * \brief   RamCfg_Set_WriteProtection() refuses a range which is empty or beyond the pages.
 *
 * \details Pages are protected, then the function is called without a page (this would remove the protection), with the
 *          first page beyond the pages, with a range which ends beyond the pages, with a count beyond the pages and
 *          with a range which would overflow the 16-bit page type.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, no register written (the protection stays).
 */
void Ut_RamCfg_Set_WriteProtection_InvalidRange_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, 1u ) );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, RAMCFG_PAGE_CNT_UNUSED ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, UT_RAMCFG_SRAM2_PAGES, 1u ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                   UT_RAMCFG_SRAM2_PAGES - 1u,
                                                   UT_RAMCFG_RANGE_TWO_PAGES ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, UT_RAMCFG_SRAM2_PAGES + 1u ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, UINT16_MAX, UT_RAMCFG_RANGE_TWO_PAGES ) );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_WriteProtection() refuses an SRAM which has no write protection.
 *
 * \details Requests the protection of the first page of every SRAM except SRAM2 (the registers are filled with a
 *          pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR for every SRAM without the write protection, no register written.
 */
void Ut_RamCfg_Set_WriteProtection_SramWithoutProtection_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( 0u == expect->WpPages )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( sramId, 0u, 1u ) );
        }
        else
        {
            /* The SRAM has the write protection */
        }
    }

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_WriteProtection() refuses an SRAM beyond the list and a module which is not initialized.
 *
 * \details Requests the protection before the initialization and for the SRAM beyond the list (the registers are filled
 *          with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, no register written.
 */
void Ut_RamCfg_Set_WriteProtection_InvalidSramOrNotInitialized_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, 1u ) );
    UtRamCfg_Check_BlockUnchanged( block );

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_CNT, 0u, 1u ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_UNUSED, 0u, 1u ) );
    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Set_WriteProtection() refuses pages which hold the stack of the caller.
 *
 * \details Requests the protection of the pages 1 - 2 of SRAM2 with the main stack pointer at the first and the last
 *          word of the range, with the process stack pointer inside the range (secure alias of the address), then with
 *          the stack pointers just outside of the range (page 0 and the first address behind the range).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR and no register written while a stack pointer is inside the range.
 * - RAMCFG_REQUEST_OK with the stack pointers outside (the address behind the range does not belong to it).
 */
void Ut_RamCfg_Set_WriteProtection_StackInRange_ReturnsErrorWithoutWrite( void )
{
    uint32_t       block[ UT_RAMCFG_BLOCK_WORDS ];
    const uint32_t rangeBase = UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UT_RAMCFG_PAGE_SIZE;
    const uint32_t rangeEnd  = rangeBase + ( 2u * UT_RAMCFG_PAGE_SIZE );

    UtRamCfg_Init_Default();
    UtRamCfg_Snapshot( block );

    __set_MSP( rangeBase );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES ) );

    __set_MSP( rangeEnd - UT_RAMCFG_WORD_SIZE );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES ) );

    __set_MSP( 0u );
    __set_PSP( ( rangeBase | UT_RAMCFG_SECURE_ALIAS ) + UT_RAMCFG_STACK_INSIDE );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES ) );

    UtRamCfg_Check_BlockUnchanged( block );

    __set_PSP( rangeBase - UT_RAMCFG_WORD_SIZE );
    __set_MSP( rangeEnd );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES ) );

    UtRamCfg_Check_Wpr( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES );
}


/**
 * \brief   RamCfg_Set_WriteProtection() detects protection bits which do not stay set.
 *
 * \details The HW model keeps the write protection bits at zero.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR (read-back mismatch).
 */
void Ut_RamCfg_Set_WriteProtection_ReadBackMismatch_ReturnsError( void )
{
    UtRamCfg_Init_Default();

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_WPR_NOT_KEPT );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, 1u ) );
}


/**
 * \brief   RamCfg_Get_WriteProtection() returns whether every page of a range is protected.
 *
 * \details Protects the pages 1 - 2 of SRAM2 and asks for ranges inside, around and next to the protected pages. On the
 *          lines with more than 32 pages the pages 31 - 32 are protected as well and asked for across the register
 *          border.
 *
 * \par Expected results
 * - RAMCFG_FUNCTION_ACTIVE for ranges inside the protected pages, RAMCFG_FUNCTION_INACTIVE for ranges which contain a
 *   page which is not protected.
 */
void Ut_RamCfg_Get_WriteProtection_ReturnsStateOfRange( void )
{
    ramCfg_FunctionState_t protection = (ramCfg_FunctionState_t)UT_RAMCFG_SENTINEL;

    UtRamCfg_Init_Default();
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_WriteProtection( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 1u, UT_RAMCFG_RANGE_TWO_PAGES, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_ACTIVE, protection );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, UT_RAMCFG_PAGE_LAST_OF_RANGE, 1u, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_ACTIVE, protection );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, UT_RAMCFG_RANGE_THREE_PAGES, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, protection );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, UT_RAMCFG_PAGE_BEYOND_RANGE, 1u, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, protection );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, 1u, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, protection );

#if ( UT_RAMCFG_SRAM2_PAGES > UT_RAMCFG_PAGES_PER_WPR )
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                   UT_RAMCFG_PAGE_BORDER_1,
                                                   UT_RAMCFG_RANGE_TWO_PAGES ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2,
                                                   UT_RAMCFG_PAGE_BORDER_1,
                                                   UT_RAMCFG_RANGE_TWO_PAGES,
                                                   &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_ACTIVE, protection );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2,
                                                   UT_RAMCFG_PAGE_BEFORE_BORDER,
                                                   UT_RAMCFG_RANGE_FOUR_PAGES,
                                                   &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, protection );
#endif
}


/**
 * \brief   RamCfg_Get_WriteProtection() refuses invalid requests without side effects.
 *
 * \details Calls the getter before the initialization, with a NULL pointer, with an empty range, with ranges beyond the
 *          pages, for an SRAM beyond the list and for every SRAM without the write protection (the output holds a
 *          sentinel).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the output is not changed, no other module is called (no clock is enabled).
 */
void Ut_RamCfg_Get_WriteProtection_InvalidRequest_ReturnsErrorOutputUnchanged( void )
{
    ramCfg_FunctionState_t protection = (ramCfg_FunctionState_t)UT_RAMCFG_SENTINEL;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, 1u, &protection ) );

    UtRamCfg_Init_Default();
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, 1u, NULL ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, RAMCFG_PAGE_CNT_UNUSED, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, UT_RAMCFG_SRAM2_PAGES, 1u, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, UT_RAMCFG_SRAM2_PAGES + 1u, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WriteProtection( RAMCFG_SRAM_CNT, 0u, 1u, &protection ) );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( 0u == expect->WpPages )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WriteProtection( sramId, 0u, 1u, &protection ) );
        }
        else
        {
            /* The SRAM has the write protection */
        }
    }

    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, protection );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}

/* ============================== ERASE ===================================== */

/**
 * \brief   RamCfg_Set_Erase() without an erase complete callback starts the erase with the unlock keys and waits for
 *          its end.
 *
 * \details The HW model accepts the erase bit only after the keys 0xCA and 0x53, ends the erase at once and zeroes the
 *          first word of the SRAM. Erases every SRAM of the line (the first word holds a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the erase started once, the first word of the SRAM is zero, the state is "done", the erase key
 *   register holds the second key.
 * - The clock of the memory is activated through the Rcc module where the memory has an enable bit (none for SRAM4 and
 *   SRAM5).
 */
void Ut_RamCfg_Set_Erase_NoCallback_StartsWithKeysAndWaitsForEnd( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect      = UtRamCfg_Get_Expect( sramId );
        volatile uint32_t * const     word        = (volatile uint32_t *)(uintptr_t)expect->Base;
        const uint32_t                clockCnt    = utRamCfg_RccCnt;
        ramCfg_EraseState_t           eraseState  = RAMCFG_ERASE_CNT;

        *word = UT_RAMCFG_DATA_PATTERN;

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( sramId, &eraseState ) );
        TEST_ASSERT_EQUAL( RAMCFG_ERASE_IDLE, eraseState );

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Erase( sramId ) );

        TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_EraseStartCnt[ sramId ] );
        TEST_ASSERT_EQUAL_HEX32( 0u, *word );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ERASE_KEY_2, UtRamCfg_Get_Inst( sramId )->ERKEYR );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( sramId, &eraseState ) );
        TEST_ASSERT_EQUAL( RAMCFG_ERASE_DONE, eraseState );

        if( RAMCFG_FUNCTION_ACTIVE == expect->HasClk )
        {
            TEST_ASSERT_EQUAL_UINT32( clockCnt + 1u, utRamCfg_RccCnt );
            TEST_ASSERT_EQUAL( expect->ClkPeriph, utRamCfg_RccList[ clockCnt ] );
        }
        else
        {
            TEST_ASSERT_EQUAL_UINT32( clockCnt, utRamCfg_RccCnt );
        }
    }
}


/**
 * \brief   RamCfg_Set_Erase() with an erase complete callback returns at once and RamCfg_Task() reports the end.
 *
 * \details The HW model holds the erase until the test releases it. Erases SRAM2 and the backup SRAM, runs the task
 *          while the erases run, releases the erase of SRAM2, runs the task, releases the other one, runs the task
 *          twice.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK without waiting, the state "busy" while the erase runs, "done" after its end.
 * - The task does not report a running erase, reports every finished erase once, with the SRAM.
 */
void Ut_RamCfg_Set_Erase_WithCallback_ReturnsAtOnceAndTaskReportsEnd( void )
{
    ramCfg_EraseState_t eraseState = RAMCFG_ERASE_CNT;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ERASE_HOLD );
    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Erase( RAMCFG_SRAM_2 ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Erase( RAMCFG_SRAM_BACKUP ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_BUSY, eraseState );

    RamCfg_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_EraseCnt );

    UtRamCfg_Finish_Erase( RAMCFG_SRAM_2 );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_DONE, eraseState );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_BACKUP, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_BUSY, eraseState );

    RamCfg_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_EraseCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_2, utRamCfg_EraseSram );

    UtRamCfg_Finish_Erase( RAMCFG_SRAM_BACKUP );

    RamCfg_Task();
    RamCfg_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_TWICE, utRamCfg_EraseCnt );
    TEST_ASSERT_EQUAL( RAMCFG_SRAM_BACKUP, utRamCfg_EraseSram );
}


/**
 * \brief   RamCfg_Set_Erase() without a callback reports a timeout and leaves the erase to the hardware.
 *
 * \details The HW model never ends the erase.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, the state stays "busy" (the erase is not stopped).
 */
void Ut_RamCfg_Set_Erase_NoCallback_Timeout_ReturnsErrorStateBusy( void )
{
    ramCfg_EraseState_t eraseState = RAMCFG_ERASE_CNT;

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ERASE_HOLD );
    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( RAMCFG_SRAM_2 ) );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_BUSY, eraseState );
}


/**
 * \brief   RamCfg_Set_Erase() refuses a new erase while an erase of the SRAM runs.
 *
 * \details The busy flag of SRAM2 is set (an erase of the hardware), then the erase bit of the backup SRAM (an erase of
 *          the software). Requests an erase of both.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases, no register written (the keys are not written), no clock activated.
 */
void Ut_RamCfg_Set_Erase_EraseRunning_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();

    UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR      = UT_RAMCFG_ISR_SRAMBUSY;
    UtRamCfg_Get_Inst( RAMCFG_SRAM_BACKUP )->CR |= UT_RAMCFG_CR_SRAMER;
    UtRamCfg_Snapshot( block );
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( RAMCFG_SRAM_2 ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( RAMCFG_SRAM_BACKUP ) );

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}


/**
 * \brief   RamCfg_Set_Erase() refuses the erase of an SRAM which holds the stack of the caller.
 *
 * \details For every SRAM sets the main stack pointer at the first word and at the last word of the SRAM, the process
 *          stack pointer inside the SRAM (secure alias of the address), and requests the erase. Then the stack pointers
 *          are put just outside of the SRAM2 (the word before the SRAM and the first address behind it) and the erase
 *          of SRAM2 is requested.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR and no register written, no clock activated, state "idle" while a stack pointer is inside the
 *   SRAM.
 * - RAMCFG_REQUEST_OK with the stack pointers outside (the address behind the SRAM does not belong to it).
 */
void Ut_RamCfg_Set_Erase_StackInSram_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );
    UtRamCfg_Init_Default();
    UtRamCfg_Snapshot( block );
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect     = UtRamCfg_Get_Expect( sramId );
        ramCfg_EraseState_t           eraseState = RAMCFG_ERASE_CNT;

        __set_PSP( 0u );
        __set_MSP( expect->Base );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( sramId ) );

        __set_MSP( expect->Base + expect->Size - UT_RAMCFG_WORD_SIZE );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( sramId ) );

        __set_MSP( 0u );
        __set_PSP( ( expect->Base | UT_RAMCFG_SECURE_ALIAS ) + UT_RAMCFG_STACK_INSIDE );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( sramId ) );

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( sramId, &eraseState ) );
        TEST_ASSERT_EQUAL( RAMCFG_ERASE_IDLE, eraseState );
    }

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );

    __set_PSP( UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base - UT_RAMCFG_WORD_SIZE );
    __set_MSP( UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Base + UtRamCfg_Get_Expect( RAMCFG_SRAM_2 )->Size );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Erase( RAMCFG_SRAM_2 ) );
}


/**
 * \brief   RamCfg_Set_Erase() reports a refusal of the clock activation and does not start the erase.
 *
 * \details The RCC stub refuses the clock of the memory.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, no register written (the keys are not written), state "idle".
 */
void Ut_RamCfg_Set_Erase_RccError_ReturnsErrorWithoutWrite( void )
{
    uint32_t            block[ UT_RAMCFG_BLOCK_WORDS ];
    ramCfg_EraseState_t eraseState = RAMCFG_ERASE_CNT;

    UtRamCfg_Init_Default();
    UtRamCfg_Snapshot( block );

    utRamCfg_RccRet = RCC_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( RAMCFG_SRAM_2 ) );

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_IDLE, eraseState );
}


/**
 * \brief   RamCfg_Set_Erase() refuses an SRAM beyond the list and a module which is not initialized.
 *
 * \details Requests the erase before the initialization and for the SRAM beyond the list (the registers are filled with
 *          a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, no register written, no clock activated.
 */
void Ut_RamCfg_Set_Erase_InvalidSramOrNotInitialized_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( RAMCFG_SRAM_2 ) );
    UtRamCfg_Check_BlockUnchanged( block );
    UtRamCfg_Check_NoOtherModuleCalls();

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_Erase( RAMCFG_SRAM_UNUSED ) );

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}


/**
 * \brief   RamCfg_Get_EraseState() reports idle, busy and done.
 *
 * \details Reads the state of SRAM2 after the initialization, with the busy flag set (an erase of the hardware, nothing
 *          requested by the module), with the erase bit set (registers written directly, no HW model), then starts the
 *          HW model, requests an erase and reads the state after its end.
 *
 * \par Expected results
 * - "Idle" before any request, "busy" while the busy flag or the erase bit is set, "done" after the requested erase.
 * - The function does not enable any clock.
 */
void Ut_RamCfg_Get_EraseState_ReportsIdleBusyDone( void )
{
    ramCfg_EraseState_t eraseState = RAMCFG_ERASE_CNT;

    UtRamCfg_Init_Default();
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_IDLE, eraseState );

    UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR = UT_RAMCFG_ISR_SRAMBUSY;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_BUSY, eraseState );

    UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->ISR = 0u;
    UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->CR  = UT_RAMCFG_CR_SRAMER;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_BUSY, eraseState );

    UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->CR = 0u;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_IDLE, eraseState );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_Erase( RAMCFG_SRAM_2 ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( RAMCFG_ERASE_DONE, eraseState );
}


/**
 * \brief   RamCfg_Get_EraseState() refuses invalid requests without side effects.
 *
 * \details Calls the getter before the initialization, with a NULL pointer and for an SRAM beyond the list.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the output is not changed, no other module is called.
 */
void Ut_RamCfg_Get_EraseState_InvalidRequest_ReturnsErrorOutputUnchanged( void )
{
    ramCfg_EraseState_t eraseState = (ramCfg_EraseState_t)UT_RAMCFG_SENTINEL;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_EraseState( RAMCFG_SRAM_2, &eraseState ) );
    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, eraseState );

    UtRamCfg_Init_Default();
    utRamCfg_RccCnt = UT_RAMCFG_CALLS_NONE;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_EraseState( RAMCFG_SRAM_2, NULL ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_EraseState( RAMCFG_SRAM_CNT, &eraseState ) );
    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, eraseState );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
}

/* ================================= TASK =================================== */

/**
 * \brief   RamCfg_Task() does nothing before the initialization and without a started erase.
 *
 * \details Runs the task before the initialization and after it (the erase complete callback is configured, no erase
 *          was started).
 *
 * \par Expected results
 * - No callback, no register is written.
 */
void Ut_RamCfg_Task_NothingToReport_DoesNothing( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    RamCfg_Task();

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_EraseCnt );

    UtRamCfg_Init_Callbacks( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ );
    UtRamCfg_Snapshot( block );

    RamCfg_Task();

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_EraseCnt );
}

/* ============================= WAIT STATES ================================ */

/**
 * \brief   The wait states functions return an error (STM32H5 has no wait state setting of the SRAMs).
 *
 * \details Sets and reads the wait states of every SRAM before and after the initialization (the output holds a
 *          sentinel).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the output is not changed, no register written, no other module called.
 */
void Ut_RamCfg_WaitStates_NotAvailable_ReturnError( void )
{
    uint32_t            block[ UT_RAMCFG_BLOCK_WORDS ];
    ramCfg_WaitStates_t waitStates = (ramCfg_WaitStates_t)UT_RAMCFG_SENTINEL;

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WaitStates( sramId, 1u ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WaitStates( sramId, &waitStates ) );
    }

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WaitStates( RAMCFG_SRAM_2, NULL ) );
    UtRamCfg_Check_BlockUnchanged( block );
    UtRamCfg_Check_NoOtherModuleCalls();

    UtRamCfg_Init_Default();
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_WaitStates( sramId, 0u ) );
        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WaitStates( sramId, &waitStates ) );
    }

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_SENTINEL, waitStates );
}

/* ============================ INTERRUPT PRIORITY ========================== */

/**
 * \brief   RamCfg_Set_IrqPriority() passes the priority to NVIC and keeps it when the interrupt is reconfigured.
 *
 * \details Initializes the module, sets the priority 2, reads it, switches the reporting of SRAM2 off and on again.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the priority 2 in NVIC and returned by RamCfg_Get_IrqPriority(), the priority 2 is used again
 *   when the interrupt is activated for the reporting switched on again.
 */
void Ut_RamCfg_Set_IrqPriority_Initialized_PassesPriorityToNvic( void )
{
    ramCfg_IrqPrio_t irqPrio = 0u;

    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_IrqPriority( UT_RAMCFG_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_IRQ_PRIO, utRamCfg_Prio );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_IRQ_PRIO, irqPrio );

    utRamCfg_Prio = 0u;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_NONE ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_ReportMode( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_IRQ_PRIO, utRamCfg_Prio );
}


/**
 * \brief   RamCfg_Set_IrqPriority() reports a refusal of NVIC and needs an initialized module.
 *
 * \details Sets the priority before the initialization, then after it with the NVIC stub refusing the priority.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases, before the initialization NVIC is not called.
 */
void Ut_RamCfg_Set_IrqPriority_NotInitializedOrRefused_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_IrqPriority( UT_RAMCFG_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_PrioCnt );

    UtRamCfg_Init_Default();

    utRamCfg_NvicPrioRet = NVIC_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_IrqPriority( UT_RAMCFG_IRQ_PRIO ) );
}


/**
 * \brief   RamCfg_Get_IrqPriority() refuses invalid requests without side effects.
 *
 * \details Calls the getter before the initialization, with a NULL pointer and with the NVIC stub refusing the
 *          priority.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the output is not changed, before the initialization NVIC is not called.
 */
void Ut_RamCfg_Get_IrqPriority_InvalidRequest_ReturnsErrorOutputUnchanged( void )
{
    ramCfg_IrqPrio_t irqPrio = UT_RAMCFG_SENTINEL;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_GetPrioCnt );

    UtRamCfg_Init_Default();

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_IrqPriority( NULL ) );

    utRamCfg_NvicGetPrioRet = NVIC_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_SENTINEL, irqPrio );
}

/* ========================= FEATURES OF THE SRAMS ========================== */

/**
 * \brief   The features of every SRAM of the line follow the reference manual.
 *
 * \details Checks for every SRAM the ECC (the check mode ECC is accepted or refused), the write protection (the range
 *          of all pages of the line is accepted by the getter, one more page is refused) and the clock of the memory
 *          (activated by the erase where the Rcc module has an enable bit).
 *
 * \par Expected results
 * - SRAM1: no ECC, no write protection. SRAM2: ECC, write protection of 16 (STM32H503), 80 (STM32H523 ... STM32H553),
 *   64 (STM32H562, STM32H563, STM32H573) or 128 pages (STM32H5E4 ... STM32H5F5). SRAM3: ECC except STM32H523 and
 *   STM32H533, no write protection. SRAM4 and SRAM5: neither. Backup SRAM: ECC, no write protection.
 */
void Ut_RamCfg_Features_FollowReferenceManualOfLine( void )
{
    UtRamCfg_Init_Default();

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect     = UtRamCfg_Get_Expect( sramId );
        ramCfg_FunctionState_t        protection = RAMCFG_FUNCTION_INACTIVE;

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Set_CheckMode( sramId, RAMCFG_CHECK_ECC ) );
        }
        else
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Set_CheckMode( sramId, RAMCFG_CHECK_ECC ) );
        }

        if( 0u < expect->WpPages )
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                               RamCfg_Get_WriteProtection( sramId, 0u, expect->WpPages, &protection ) );
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                               RamCfg_Get_WriteProtection( sramId, 0u, expect->WpPages + 1u, &protection ) );
        }
        else
        {
            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_WriteProtection( sramId, 0u, 1u, &protection ) );
        }
    }
}



/**
 * \brief   The getters of the module have no side effects.
 *
 * \details Initializes the module with the default configuration, copies the register block and the counters of the
 *          calls of the RCC and NVIC modules, then calls every getter for every SRAM (valid requests and requests the
 *          SRAM refuses).
 *
 * \par Expected results
 * - No register of the block is written, no clock is enabled, the interrupt is not configured or switched, the NMI
 *   vector is not touched (the getter of the interrupt priority reads the priority from NVIC and nothing else).
 */
void Ut_RamCfg_Getters_ValidRequests_HaveNoSideEffects( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Init_Default();
    UtRamCfg_Snapshot( block );

    const uint32_t rccCnt      = utRamCfg_RccCnt;
    const uint32_t handlerCnt  = utRamCfg_HandlerCnt;
    const uint32_t prioCnt     = utRamCfg_PrioCnt;
    const uint32_t activeCnt   = utRamCfg_ActiveCnt;
    const uint32_t inactiveCnt = utRamCfg_InactiveCnt;
    const uint32_t coreSetCnt  = utRamCfg_CoreSetCnt;
    const uint32_t coreGetCnt  = utRamCfg_CoreGetCnt;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        ramCfg_CheckMode_t       checkMode  = RAMCFG_CHECK_CNT;
        ramCfg_ReportMode_t      reportMode = RAMCFG_REPORT_CNT;
        ramCfg_ErrorStatistics_t statistics;
        ramCfg_Address_t         address    = 0u;
        ramCfg_FunctionState_t   protection = RAMCFG_FUNCTION_INACTIVE;
        ramCfg_EraseState_t      eraseState = RAMCFG_ERASE_CNT;
        ramCfg_WaitStates_t      waitStates = 0u;

        (void)RamCfg_Get_CheckMode( sramId, &checkMode );
        (void)RamCfg_Get_ReportMode( sramId, &reportMode );
        (void)RamCfg_Get_ErrorStatistics( sramId, &statistics );
        (void)RamCfg_Get_FailingAddress( sramId, &address );
        (void)RamCfg_Get_WriteProtection( sramId, 0u, 1u, &protection );
        (void)RamCfg_Get_EraseState( sramId, &eraseState );
        (void)RamCfg_Get_WaitStates( sramId, &waitStates );
    }

    UtRamCfg_Check_BlockUnchanged( block );
    TEST_ASSERT_EQUAL_UINT32( rccCnt, utRamCfg_RccCnt );
    TEST_ASSERT_EQUAL_UINT32( handlerCnt, utRamCfg_HandlerCnt );
    TEST_ASSERT_EQUAL_UINT32( prioCnt, utRamCfg_PrioCnt );
    TEST_ASSERT_EQUAL_UINT32( activeCnt, utRamCfg_ActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( inactiveCnt, utRamCfg_InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( coreSetCnt, utRamCfg_CoreSetCnt );
    TEST_ASSERT_EQUAL_UINT32( coreGetCnt, utRamCfg_CoreGetCnt );
}

/* ========================== REGISTER COMPONENT ============================ */

/** Count of the registers which do not depend on the line: CR, IER, ISR, SEAR, DEAR, ICR */
#define UT_RAMCFG_REG_STATUS_CNT            ( 6u )

/** Count of the key registers: ECCKEYR, ERKEYR */
#define UT_RAMCFG_REG_KEY_CNT               ( 2u )

/** Value written to a register by the tests of the generic accessors */
#define UT_RAMCFG_VALUE_RAW                 ( 0x12345678u )

/** Pattern of a register and masks used by the tests of the generic accessors */
#define UT_RAMCFG_VALUE_PATTERN             ( 0xA5A5A5A5u )
#define UT_RAMCFG_VALUE_ALL_ONES            ( 0xFFFFFFFFu )
#define UT_RAMCFG_MASK_BITS                 ( 0x0000FF00u )
#define UT_RAMCFG_MASK_FIELD                ( 0x00FFFF00u )
#define UT_RAMCFG_MASK_LOW_HALF             ( 0x0000FFFFu )
#define UT_RAMCFG_MASK_CLEAR                ( 0x00FF00F0u )
#define UT_RAMCFG_MASK_PICK                 ( 0x0F0F0F0Fu )

/** Addresses of the failing words stored by the tests of the address getters */
#define UT_RAMCFG_VALUE_SEAR                ( 0x12345678u )
#define UT_RAMCFG_VALUE_DEAR                ( 0x9ABCDEF0u )

/** Pages of SRAM2 which are not protected in the test of the getter (every page whose number modulo the period is the
 *  hole) */
#define UT_RAMCFG_HOLE_PERIOD               ( 7u )
#define UT_RAMCFG_HOLE_PAGE                 ( 3u )

/** Registers of one memory with the offsets of the reference manual (the identifications come from the module) */
typedef struct
{
    ramCfg_RegId_t RegId;  /**< Identification of the register in the module               */
    uint32_t       Offset; /**< Offset of the register in the registers of a memory (RM)   */
}   utRamCfg_Reg_t;

static const utRamCfg_Reg_t utRamCfg_RegList[] =
{
    { RAMCFG_REG_CR    , UT_RAMCFG_OFFSET_CR     },
    { RAMCFG_REG_IER   , UT_RAMCFG_OFFSET_IER    },
    { RAMCFG_REG_ISR   , UT_RAMCFG_OFFSET_ISR    },
    { RAMCFG_REG_SEAR  , UT_RAMCFG_OFFSET_SEAR   },
    { RAMCFG_REG_DEAR  , UT_RAMCFG_OFFSET_DEAR   },
    { RAMCFG_REG_ICR   , UT_RAMCFG_OFFSET_ICR    },
    { RAMCFG_REG_WPR1  , UT_RAMCFG_OFFSET_WPR1   },
#if ( UT_RAMCFG_WPR_CNT > 1u )
    { RAMCFG_REG_WPR2  , UT_RAMCFG_OFFSET_WPR2   },
#endif
#if ( UT_RAMCFG_WPR_CNT > 2u )
    { RAMCFG_REG_WPR3  , UT_RAMCFG_OFFSET_WPR3   },
#endif
#if ( UT_RAMCFG_WPR_CNT > 3u )
    { RAMCFG_REG_WPR4  , UT_RAMCFG_OFFSET_WPR4   },
#endif
    { RAMCFG_REG_ECCKEY, UT_RAMCFG_OFFSET_ECCKEY },
    { RAMCFG_REG_ERKEYR, UT_RAMCFG_OFFSET_ERKEYR },
};

/** Count of the registers of one memory on the line */
#define UT_RAMCFG_REG_LIST_CNT              ( sizeof( utRamCfg_RegList ) / sizeof( utRamCfg_RegList[ 0u ] ) )

/**
 * \brief   The list of the registers and the count of the write protection pages follow the device line.
 *
 * \details Compares the count of the registers, the count of the write protection registers and the count of the pages
 *          of SRAM2 of the module with the reference manual of the line.
 *
 * \par Expected results
 * - The write protection pages are 16 (STM32H503), 80 (STM32H523 ... STM32H553), 64 (STM32H562, STM32H563, STM32H573)
 *   or 128 (STM32H5E4 ... STM32H5F5), the page size is 1 Kbyte, the write protection registers are one per 32 pages.
 * - The list holds the six registers of the status, the write protection registers of the line and the two key
 *   registers.
 */
void Ut_RamCfg_Reg_RegisterList_MatchesLine_CountsOfRegistersAndPages( void )
{
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_SRAM2_PAGES, RAMCFG_REG_WP_PAGE_CNT );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_PAGE_SIZE, RAMCFG_PAGE_SIZE );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_PAGES_PER_WPR, RAMCFG_REG_PAGES_PER_WPR );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_WPR_CNT, RAMCFG_REG_WPR_CNT );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_REG_STATUS_CNT + UT_RAMCFG_WPR_CNT + UT_RAMCFG_REG_KEY_CNT, RAMCFG_REG_CNT );
    TEST_ASSERT_EQUAL_UINT32( RAMCFG_REG_CNT, UT_RAMCFG_REG_LIST_CNT );
}


/**
 * \brief   The registers of every SRAM are written at the addresses of the reference manual.
 *
 * \details Writes a value to every register of every SRAM with RamCfg_Set_RegRaw(), reads it at the address (RAMCFG
 *          block address + offset of the memory + offset of the register) and checks that no other word of the block
 *          changed.
 *
 * \par Expected results
 * - The register holds the written value, all other words of the block are zero.
 */
void Ut_RamCfg_Reg_Set_RegRaw_AllRegistersOfAllSrams_WriteAtAddressOfReferenceManual( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        for( uint32_t regIdx = 0u; UT_RAMCFG_REG_LIST_CNT > regIdx; regIdx ++ )
        {
            volatile uint32_t * const regPtr = UtRamCfg_Get_RegPtr( sramId, utRamCfg_RegList[ regIdx ].Offset );

            RamCfg_Set_RegRaw( sramId, utRamCfg_RegList[ regIdx ].RegId, UT_RAMCFG_VALUE_RAW );

            UtRamCfg_Check_OnlyWord( regPtr, UT_RAMCFG_VALUE_RAW );

            *regPtr = 0u;
        }
    }
}


/**
 * \brief   RamCfg_Set_RegRaw() writes the whole register without reading it.
 *
 * \details Presets every register of every SRAM with all bits set and writes a value with a single bit set.
 *
 * \par Expected results
 * - The register holds exactly the written value (a read-modify-write would keep the other bits).
 */
void Ut_RamCfg_Reg_Set_RegRaw_PresetRegister_ReplacesWholeValue( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        for( uint32_t regIdx = 0u; UT_RAMCFG_REG_LIST_CNT > regIdx; regIdx ++ )
        {
            volatile uint32_t * const regPtr = UtRamCfg_Get_RegPtr( sramId, utRamCfg_RegList[ regIdx ].Offset );

            *regPtr = UT_RAMCFG_VALUE_ALL_ONES;

            RamCfg_Set_RegRaw( sramId, utRamCfg_RegList[ regIdx ].RegId, UT_RAMCFG_BIT_ONE );

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_BIT_ONE, *regPtr );
        }
    }
}


/**
 * \brief   RamCfg_Set_RegBit() sets the bits of the mask and keeps the other bits.
 *
 * \details Presets every register of every SRAM with a pattern and sets a mask.
 *
 * \par Expected results
 * - The register holds the pattern with the bits of the mask set.
 */
void Ut_RamCfg_Reg_Set_RegBit_PresetRegister_SetsOnlyMaskedBits( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        for( uint32_t regIdx = 0u; UT_RAMCFG_REG_LIST_CNT > regIdx; regIdx ++ )
        {
            volatile uint32_t * const regPtr = UtRamCfg_Get_RegPtr( sramId, utRamCfg_RegList[ regIdx ].Offset );

            *regPtr = UT_RAMCFG_VALUE_PATTERN;

            RamCfg_Set_RegBit( sramId, utRamCfg_RegList[ regIdx ].RegId, UT_RAMCFG_MASK_BITS );

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_PATTERN | UT_RAMCFG_MASK_BITS, *regPtr );
        }
    }
}


/**
 * \brief   RamCfg_Reset_RegBit() clears the bits of the mask and keeps the other bits.
 *
 * \details Presets every register of every SRAM with all bits set and clears a mask.
 *
 * \par Expected results
 * - The register holds all bits except the ones of the mask.
 */
void Ut_RamCfg_Reg_Reset_RegBit_PresetRegister_ClearsOnlyMaskedBits( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        for( uint32_t regIdx = 0u; UT_RAMCFG_REG_LIST_CNT > regIdx; regIdx ++ )
        {
            volatile uint32_t * const regPtr = UtRamCfg_Get_RegPtr( sramId, utRamCfg_RegList[ regIdx ].Offset );

            *regPtr = UT_RAMCFG_VALUE_ALL_ONES;

            RamCfg_Reset_RegBit( sramId, utRamCfg_RegList[ regIdx ].RegId, UT_RAMCFG_MASK_CLEAR );

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_ALL_ONES & ~UT_RAMCFG_MASK_CLEAR, *regPtr );
        }
    }
}


/**
 * \brief   RamCfg_Get_RegBit() returns the bits of the mask only.
 *
 * \details Presets every register of every SRAM with a pattern and reads it with a mask and with an empty mask.
 *
 * \par Expected results
 * - The pattern with the mask applied, zero for the empty mask, the register is not changed.
 */
void Ut_RamCfg_Reg_Get_RegBit_PresetRegister_ReturnsMaskedBits( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        for( uint32_t regIdx = 0u; UT_RAMCFG_REG_LIST_CNT > regIdx; regIdx ++ )
        {
            volatile uint32_t * const regPtr = UtRamCfg_Get_RegPtr( sramId, utRamCfg_RegList[ regIdx ].Offset );

            *regPtr = UT_RAMCFG_VALUE_PATTERN;

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_PATTERN & UT_RAMCFG_MASK_PICK,
                                     RamCfg_Get_RegBit( sramId,
                                                        utRamCfg_RegList[ regIdx ].RegId,
                                                        UT_RAMCFG_MASK_PICK ) );
            TEST_ASSERT_EQUAL_HEX32( 0u, RamCfg_Get_RegBit( sramId, utRamCfg_RegList[ regIdx ].RegId, 0u ) );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_PATTERN, *regPtr );
        }
    }
}


/**
 * \brief   RamCfg_Set_RegVal() replaces the bit field of the mask and keeps the other bits.
 *
 * \details Presets every register of every SRAM with a pattern and writes a value with a mask which is narrower than
 *          the value.
 *
 * \par Expected results
 * - The bits of the mask hold the bits of the value, the other bits hold the pattern (the bits of the value outside of
 *   the mask are ignored).
 */
void Ut_RamCfg_Reg_Set_RegVal_PresetRegister_ReplacesOnlyMaskedField( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        for( uint32_t regIdx = 0u; UT_RAMCFG_REG_LIST_CNT > regIdx; regIdx ++ )
        {
            volatile uint32_t * const regPtr = UtRamCfg_Get_RegPtr( sramId, utRamCfg_RegList[ regIdx ].Offset );

            *regPtr = UT_RAMCFG_VALUE_PATTERN;

            RamCfg_Set_RegVal( sramId, utRamCfg_RegList[ regIdx ].RegId, UT_RAMCFG_MASK_LOW_HALF, UT_RAMCFG_VALUE_RAW );

            TEST_ASSERT_EQUAL_HEX32( ( UT_RAMCFG_VALUE_PATTERN & ~UT_RAMCFG_MASK_LOW_HALF ) |
                                     ( UT_RAMCFG_VALUE_RAW & UT_RAMCFG_MASK_LOW_HALF ),
                                     *regPtr );
        }
    }
}


/**
 * \brief   RamCfg_Get_RegVal() returns the bit field of the mask.
 *
 * \details Presets every register of every SRAM with a value and reads it with a mask.
 *
 * \par Expected results
 * - The value with the mask applied (not shifted), the register is not changed.
 */
void Ut_RamCfg_Reg_Get_RegVal_PresetRegister_ReturnsMaskedField( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        for( uint32_t regIdx = 0u; UT_RAMCFG_REG_LIST_CNT > regIdx; regIdx ++ )
        {
            volatile uint32_t * const regPtr = UtRamCfg_Get_RegPtr( sramId, utRamCfg_RegList[ regIdx ].Offset );

            *regPtr = UT_RAMCFG_VALUE_RAW;

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_RAW & UT_RAMCFG_MASK_FIELD,
                                     RamCfg_Get_RegVal( sramId,
                                                        utRamCfg_RegList[ regIdx ].RegId,
                                                        UT_RAMCFG_MASK_FIELD ) );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_RAW, *regPtr );
        }
    }
}


/**
 * \brief   The generic accessors do not access a register for an invalid SRAM or register identification.
 *
 * \details Calls every accessor with an SRAM beyond the list, the item "unused", a register beyond the list and the
 *          register after it, the register block is filled with a pattern.
 *
 * \par Expected results
 * - The getters return zero (no bit set), no register of the block is written.
 */
void Ut_RamCfg_Reg_Accessors_InvalidSramOrRegister_NoAccess( void )
{
    const ramCfg_SramId_t sramList[] = { RAMCFG_SRAM_CNT, RAMCFG_SRAM_UNUSED, RAMCFG_SRAM_2, RAMCFG_SRAM_2,
                                         RAMCFG_SRAM_CNT };
    const ramCfg_RegId_t  regList[]  = { RAMCFG_REG_CR, RAMCFG_REG_CR, RAMCFG_REG_CNT,
                                         (ramCfg_RegId_t)( RAMCFG_REG_CNT + UT_RAMCFG_ID_OFFSET ), RAMCFG_REG_CNT };
    uint32_t              block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    for( uint32_t caseIdx = 0u; ( sizeof( sramList ) / sizeof( sramList[ 0u ] ) ) > caseIdx; caseIdx ++ )
    {
        RamCfg_Set_RegBit( sramList[ caseIdx ], regList[ caseIdx ], UT_RAMCFG_VALUE_ALL_ONES );
        RamCfg_Reset_RegBit( sramList[ caseIdx ], regList[ caseIdx ], UT_RAMCFG_VALUE_ALL_ONES );
        RamCfg_Set_RegVal( sramList[ caseIdx ], regList[ caseIdx ], UT_RAMCFG_VALUE_ALL_ONES, UT_RAMCFG_VALUE_RAW );
        RamCfg_Set_RegRaw( sramList[ caseIdx ], regList[ caseIdx ], UT_RAMCFG_VALUE_RAW );

        TEST_ASSERT_EQUAL_HEX32( 0u,
                                 RamCfg_Get_RegBit( sramList[ caseIdx ],
                                                    regList[ caseIdx ],
                                                    UT_RAMCFG_VALUE_ALL_ONES ) );
        TEST_ASSERT_EQUAL_HEX32( 0u,
                                 RamCfg_Get_RegVal( sramList[ caseIdx ],
                                                    regList[ caseIdx ],
                                                    UT_RAMCFG_VALUE_ALL_ONES ) );
    }

    UtRamCfg_Check_BlockUnchanged( block );
}

/* ----------------------------------- ECC ---------------------------------- */

/**
 * \brief   RamCfg_Reg_Set_Ecc() enables the ECC by setting the enable bit without any key.
 *
 * \details Presets the control register of every SRAM with ECC with the address latch and enables the ECC.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the enable bit is set, the address latch is kept, the key register is not written.
 */
void Ut_RamCfg_Reg_Set_Ecc_Enable_SetsBitWithoutKeys( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

            inst->CR = UT_RAMCFG_CR_ALE;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Ecc( sramId, RAMCFG_FUNCTION_ACTIVE ) );

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE | UT_RAMCFG_CR_ALE, inst->CR );
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->ECCKEY );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   RamCfg_Reg_Set_Ecc() disables the ECC with the two keys written in the right order.
 *
 * \details The HW model accepts the clearing of the enable bit only after the keys 0xAE and 0x75 were written one after
 *          another. For every SRAM with ECC the enable bit is set, a clearing without the keys is tried (the model
 *          keeps the bit), then the ECC is disabled by the function.
 *
 * \par Expected results
 * - The clearing without the keys does not change the bit, RAMCFG_REQUEST_OK for the function, the bit is cleared, the
 *   address latch is kept, the unlock sequence was accepted once, the key register holds the second key.
 */
void Ut_RamCfg_Reg_Set_Ecc_Disable_WritesKeysInOrder( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

            inst->CR = UT_RAMCFG_CR_ECCE | UT_RAMCFG_CR_ALE;
            inst->CR &= ~UT_RAMCFG_CR_ECCE;

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE, inst->CR & UT_RAMCFG_CR_ECCE );
            TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_EccUnlockCnt[ sramId ] );

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Ecc( sramId, RAMCFG_FUNCTION_INACTIVE ) );

            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ALE, inst->CR );
            TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_EccUnlockCnt[ sramId ] );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ECC_KEY_2, inst->ECCKEY );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   RamCfg_Reg_Set_Ecc() detects an enable bit which does not stay set and one which cannot be cleared.
 *
 * \details The HW model keeps the bit at zero, then (bit set) does not accept the unlock sequence.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases (read-back mismatch).
 */
void Ut_RamCfg_Reg_Set_Ecc_ReadBackMismatch_ReturnsError( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ECC_STUCK );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Ecc( RAMCFG_SRAM_2, RAMCFG_FUNCTION_ACTIVE ) );

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ECC_LOCKED );
    UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->CR = UT_RAMCFG_CR_ECCE;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Ecc( RAMCFG_SRAM_2, RAMCFG_FUNCTION_INACTIVE ) );
}


/**
 * \brief   RamCfg_Reg_Set_Ecc() refuses an invalid state and an invalid SRAM without any write.
 *
 * \details Requests a state outside of the list and the SRAMs beyond the list (the register block is filled with a
 *          pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, no register written.
 */
void Ut_RamCfg_Reg_Set_Ecc_InvalidStateOrSram_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Set_Ecc( RAMCFG_SRAM_2,
                                           (ramCfg_FunctionState_t)( RAMCFG_FUNCTION_ACTIVE + UT_RAMCFG_ID_OFFSET ) ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Ecc( RAMCFG_SRAM_CNT, RAMCFG_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Ecc( RAMCFG_SRAM_UNUSED, RAMCFG_FUNCTION_INACTIVE ) );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Reg_Get_Ecc() returns the state of the enable bit only.
 *
 * \details Reads the state of every SRAM with ECC with the enable bit clear, set and with only the other bits of the
 *          control register set, and for the SRAMs beyond the list.
 *
 * \par Expected results
 * - RAMCFG_FUNCTION_ACTIVE only while the enable bit is set, RAMCFG_FUNCTION_INACTIVE for the SRAMs beyond the list.
 */
void Ut_RamCfg_Reg_Get_Ecc_EnableBit_ReturnsState( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

            inst->CR = 0u;
            TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, RamCfg_Reg_Get_Ecc( sramId ) );

            inst->CR = UT_RAMCFG_CR_ECCE;
            TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_ACTIVE, RamCfg_Reg_Get_Ecc( sramId ) );

            inst->CR = UT_RAMCFG_CR_ALE | UT_RAMCFG_CR_SRAMER;
            TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, RamCfg_Reg_Get_Ecc( sramId ) );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );

    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, RamCfg_Reg_Get_Ecc( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_FUNCTION_INACTIVE, RamCfg_Reg_Get_Ecc( RAMCFG_SRAM_UNUSED ) );
}

/* -------------------------------- Reporting ------------------------------- */

/**
 * \brief   RamCfg_Reg_Set_Reporting() writes the enable bits and the address latch of the mode.
 *
 * \details For every SRAM with ECC (the ECC enable bit is set) selects none, the interrupt, none again and the NMI.
 *
 * \note    Device errata bug AB#1838 (ES0561 2.2.20 "SRAM ECC error flags and addresses are updated only if interrupt
 *          is enabled"): the failing address and the flags are stored only while the address latch and the error
 *          interrupt are enabled (documentation erratum of STM32H503); the test proves that every reporting mode writes
 *          the address latch together with its interrupt enables and that the mode none writes neither.
 *
 * \par Expected results
 * - None: no enable bit, no address latch. Interrupt: single and double error interrupt and address latch. NMI: single
 *   error interrupt, NMI redirection and address latch, the double error interrupt is clear.
 * - The ECC enable bit is kept in every mode.
 */
void Ut_RamCfg_Reg_Set_Reporting_Modes_WriteEnableBitsAndLatch( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

            inst->CR  = UT_RAMCFG_CR_ECCE;
            inst->IER = UT_RAMCFG_IER_IRQ;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Reporting( sramId, RAMCFG_REPORT_NONE ) );
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->IER );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE, inst->CR );

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Reporting( sramId, RAMCFG_REPORT_IRQ ) );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_IRQ, inst->IER );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE | UT_RAMCFG_CR_ALE, inst->CR );

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Reporting( sramId, RAMCFG_REPORT_NONE ) );
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->IER );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE, inst->CR );

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Reporting( sramId, RAMCFG_REPORT_NMI ) );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_NMI, inst->IER );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE | UT_RAMCFG_CR_ALE, inst->CR );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   RamCfg_Reg_Set_Reporting() sets the address latch before it enables the error interrupts.
 *
 * \details The HW model records the state of the address latch at the moment the first error interrupt enable appears.
 *          Selects the interrupt for SRAM2 and the NMI for the backup SRAM.
 *
 * \note    Device errata bug AB#1838 (ES0561 2.2.20 "SRAM ECC error flags and addresses are updated only if interrupt
 *          is enabled"): the failing address and the flags are stored only while the address latch and the error
 *          interrupt are enabled (documentation erratum of STM32H503); the test proves that the address latch is
 *          already set when the error interrupt enable appears, so the address of the first error is stored.
 *
 * \par Expected results
 * - The enables were seen once, the address latch was already set at that moment.
 */
void Ut_RamCfg_Reg_Set_Reporting_Enable_SetsAddressLatchBeforeInterrupt( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_IerSeen[ RAMCFG_SRAM_2 ] );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_AleAtIerEnable[ RAMCFG_SRAM_2 ] );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_BACKUP, RAMCFG_REPORT_NMI ) );

    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_IerSeen[ RAMCFG_SRAM_BACKUP ] );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_FLAG_SET, utRamCfg_AleAtIerEnable[ RAMCFG_SRAM_BACKUP ] );
}


/**
 * \brief   RamCfg_Reg_Set_Reporting() refuses every mode but the NMI while the NMI redirection is set.
 *
 * \details The NMI redirection is set by the software only and cleared by a reset of the peripheral. Presets the enable
 *          bits of SRAM2 with the NMI mode and requests none, the interrupt and the NMI (the registers are compared
 *          before the NMI).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR for none and for the interrupt, no register written. RAMCFG_REQUEST_OK for the NMI.
 */
void Ut_RamCfg_Reg_Set_Reporting_NmiRedirectionSet_RefusesOtherModesWithoutWrite( void )
{
    RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( RAMCFG_SRAM_2 );
    uint32_t               block[ UT_RAMCFG_BLOCK_WORDS ];

    inst->CR  = UT_RAMCFG_CR_ECCE | UT_RAMCFG_CR_ALE;
    inst->IER = UT_RAMCFG_IER_NMI;

    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_2, RAMCFG_REPORT_NONE ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );

    UtRamCfg_Check_BlockUnchanged( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_2, RAMCFG_REPORT_NMI ) );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_NMI, inst->IER );
}


/**
 * \brief   RamCfg_Reg_Set_Reporting() refuses an invalid mode and an invalid SRAM without any write.
 *
 * \details Requests a mode outside of the list and the SRAMs beyond the list (the register block is filled with a
 *          pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, no register written.
 */
void Ut_RamCfg_Reg_Set_Reporting_InvalidModeOrSram_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_2, RAMCFG_REPORT_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_2,
                                                 (ramCfg_ReportMode_t)( RAMCFG_REPORT_CNT + UT_RAMCFG_ID_OFFSET ) ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_CNT, RAMCFG_REPORT_IRQ ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_UNUSED, RAMCFG_REPORT_NONE ) );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Reg_Set_Reporting() detects error interrupt enables and an address latch which do not stay set.
 *
 * \details The HW model keeps the error interrupt enables at zero, then the address latch at zero.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in both cases (read-back mismatch).
 */
void Ut_RamCfg_Reg_Set_Reporting_ReadBackMismatch_ReturnsError( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_IER_NOT_KEPT );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ALE_NOT_KEPT );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_Reporting( RAMCFG_SRAM_2, RAMCFG_REPORT_IRQ ) );
}


/**
 * \brief   RamCfg_Reg_Set_ReportingOff() clears the error interrupt enables and the address latch and keeps the NMI
 *          redirection.
 *
 * \details Presets the enable bits of every SRAM with ECC with the NMI mode (the redirection cannot be cleared) and the
 *          control register with the ECC enable bit and the address latch, then switches the reporting off. Then
 *          presets the interrupt mode and switches it off again.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the single and double error interrupt enables and the address latch cleared, the NMI redirection
 *   and the ECC enable bit unchanged.
 */
void Ut_RamCfg_Reg_Set_ReportingOff_EnabledBits_ClearsAllButNmiRedirection( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

            inst->CR  = UT_RAMCFG_CR_ECCE | UT_RAMCFG_CR_ALE;
            inst->IER = UT_RAMCFG_IER_IRQ;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_ReportingOff( sramId ) );
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->IER );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE, inst->CR );

            inst->CR  = UT_RAMCFG_CR_ECCE | UT_RAMCFG_CR_ALE;
            inst->IER = UT_RAMCFG_IER_NMI;

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_ReportingOff( sramId ) );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_IER_ECCNMI, inst->IER );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_ECCE, inst->CR );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   RamCfg_Reg_Set_ReportingOff() detects enables which cannot be cleared and refuses an invalid SRAM.
 *
 * \details The HW model keeps the error interrupt enables set once they were enabled. Switches the reporting of SRAM2
 *          off, then stops the model and requests the SRAMs beyond the list (the register block is filled with a
 *          pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR for the enables which stay set (read-back mismatch), RAMCFG_REQUEST_ERROR and no register
 *   written for the invalid SRAMs.
 */
void Ut_RamCfg_Reg_Set_ReportingOff_ReadBackMismatchOrInvalidSram_ReturnsError( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_IER_STUCK );
    UtRamCfg_Get_Inst( RAMCFG_SRAM_2 )->IER = UT_RAMCFG_IER_IRQ;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_ReportingOff( RAMCFG_SRAM_2 ) );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );
    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_ReportingOff( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_ReportingOff( RAMCFG_SRAM_UNUSED ) );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Reg_Get_Reporting() returns the mode of the enable bits and refuses the other combinations.
 *
 * \details Writes the enable bits of every SRAM for none, the interrupt and the NMI and for combinations which are not
 *          a mode (single error interrupt only, double error interrupt only, NMI redirection alone, double error
 *          interrupt with the NMI redirection), then calls the function with a NULL pointer and with the SRAMs beyond
 *          the list.
 *
 * \par Expected results
 * - RAMCFG_REPORT_NONE, RAMCFG_REPORT_IRQ and RAMCFG_REPORT_NMI for the three modes (RAMCFG_REQUEST_OK).
 * - RAMCFG_REQUEST_ERROR and the output not changed for the other combinations, for a NULL pointer and for an invalid
 *   SRAM.
 */
void Ut_RamCfg_Reg_Get_Reporting_EnableBits_ReturnsModeOrError( void )
{
    const uint32_t            modeBits[]  = { 0u, UT_RAMCFG_IER_IRQ, UT_RAMCFG_IER_NMI };
    const ramCfg_ReportMode_t modeList[]  = { RAMCFG_REPORT_NONE, RAMCFG_REPORT_IRQ, RAMCFG_REPORT_NMI };
    const uint32_t            otherBits[] = { UT_RAMCFG_IER_SEIE, UT_RAMCFG_IER_DEIE, UT_RAMCFG_IER_ECCNMI,
                                              UT_RAMCFG_IER_DEIE | UT_RAMCFG_IER_ECCNMI };
    ramCfg_ReportMode_t       reportMode  = RAMCFG_REPORT_NONE;

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

        for( uint32_t modeIdx = 0u; ( sizeof( modeBits ) / sizeof( modeBits[ 0u ] ) ) > modeIdx; modeIdx ++ )
        {
            reportMode = RAMCFG_REPORT_CNT;
            inst->IER  = modeBits[ modeIdx ];

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Get_Reporting( sramId, &reportMode ) );
            TEST_ASSERT_EQUAL( modeList[ modeIdx ], reportMode );
        }

        for( uint32_t otherIdx = 0u; ( sizeof( otherBits ) / sizeof( otherBits[ 0u ] ) ) > otherIdx; otherIdx ++ )
        {
            reportMode = (ramCfg_ReportMode_t)UT_RAMCFG_SENTINEL;
            inst->IER  = otherBits[ otherIdx ];

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Get_Reporting( sramId, &reportMode ) );
            TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, reportMode );
        }
    }

    reportMode = (ramCfg_ReportMode_t)UT_RAMCFG_SENTINEL;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Get_Reporting( RAMCFG_SRAM_2, NULL ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Get_Reporting( RAMCFG_SRAM_CNT, &reportMode ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Get_Reporting( RAMCFG_SRAM_UNUSED, &reportMode ) );
    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, reportMode );
}

/* ---------------------------- Write protection ---------------------------- */

/**
 * \brief   RamCfg_Reg_Set_WriteProtection() sets exactly the bits of every range of pages of SRAM2.
 *
 * \details For every first page and every count of pages up to the last page of SRAM2 of the line: clears the write
 *          protection registers, protects the range and compares all write protection registers with the bits of the
 *          pages of the range (page n is the bit n modulo 32 of the register n / 32, the registers are read at the
 *          addresses of the reference manual: 0x18, 0x1C, 0x20 or 0x30, 0x34).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, the registers hold the bits of the pages of the range and nothing else (also across the borders
 *   of the registers and for the pages 56 - 62 and 88 - 94 whose CMSIS bit definitions are one position too high).
 */
void Ut_RamCfg_Reg_Set_WriteProtection_EveryRange_SetsExactlyBitsOfRange( void )
{
    for( uint32_t firstPage = 0u; UT_RAMCFG_SRAM2_PAGES > firstPage; firstPage ++ )
    {
        for( uint32_t pageCnt = 1u; ( UT_RAMCFG_SRAM2_PAGES - firstPage ) >= pageCnt; pageCnt ++ )
        {
            UtRamCfg_Clear_Wpr( RAMCFG_SRAM_2 );

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                               RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                               (ramCfg_Page_t)firstPage,
                                                               (ramCfg_PageCnt_t)pageCnt ) );

            UtRamCfg_Check_Wpr( RAMCFG_SRAM_2, firstPage, pageCnt );
        }
    }
}


/**
 * \brief   RamCfg_Reg_Set_WriteProtection() refuses an empty range, a range beyond the pages and an invalid SRAM
 *          without any write.
 *
 * \details Requests a range without a page, the first page beyond the pages, a range which ends beyond the pages, a
 *          count beyond the pages, a range which would overflow the 16-bit page type and the SRAMs beyond the list (the
 *          register block is filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, no register written.
 */
void Ut_RamCfg_Reg_Set_WriteProtection_InvalidRange_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, RAMCFG_PAGE_CNT_UNUSED ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_2, UT_RAMCFG_SRAM2_PAGES, 1u ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                       UT_RAMCFG_SRAM2_PAGES - 1u,
                                                       UT_RAMCFG_RANGE_TWO_PAGES ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, UT_RAMCFG_SRAM2_PAGES + 1u ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_2, UINT16_MAX, UT_RAMCFG_RANGE_TWO_PAGES ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_CNT, 0u, 1u ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_UNUSED, 0u, 1u ) );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Reg_Set_WriteProtection() detects protection bits which do not stay set.
 *
 * \details The HW model keeps the write protection bits at zero. Protects a range inside one register and a range
 *          across the border of two registers (lines with more than 32 pages).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR (read-back mismatch).
 */
void Ut_RamCfg_Reg_Set_WriteProtection_ReadBackMismatch_ReturnsError( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_WPR_NOT_KEPT );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_2, 0u, 1u ) );

#if ( UT_RAMCFG_SRAM2_PAGES > UT_RAMCFG_PAGES_PER_WPR )
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Set_WriteProtection( RAMCFG_SRAM_2,
                                                       UT_RAMCFG_PAGES_PER_WPR - 1u,
                                                       UT_RAMCFG_RANGE_TWO_PAGES ) );
#endif
}


/**
 * \brief   RamCfg_Reg_Get_WriteProtection() returns whether every page of every range is protected.
 *
 * \details Protects all pages of SRAM2 except the pages whose number modulo 7 is 3 (the registers are written
 *          directly), then asks for every first page and every count of pages up to the last page. The expected state
 *          is computed page by page.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK, RAMCFG_FUNCTION_ACTIVE for a range without a hole and RAMCFG_FUNCTION_INACTIVE for a range which
 *   contains an unprotected page.
 */
void Ut_RamCfg_Reg_Get_WriteProtection_EveryRange_ReturnsStateOfRange( void )
{
    UtRamCfg_Clear_Wpr( RAMCFG_SRAM_2 );

    for( uint32_t pageIdx = 0u; UT_RAMCFG_SRAM2_PAGES > pageIdx; pageIdx ++ )
    {
        if( UT_RAMCFG_HOLE_PAGE != ( pageIdx % UT_RAMCFG_HOLE_PERIOD ) )
        {
            volatile uint32_t * const wprReg =
                UtRamCfg_Get_RegPtr( RAMCFG_SRAM_2, utRamCfg_WprOffsetList[ pageIdx / UT_RAMCFG_PAGES_PER_WPR ] );

            *wprReg |= ( UT_RAMCFG_BIT_ONE << ( pageIdx % UT_RAMCFG_PAGES_PER_WPR ) );
        }
        else
        {
            /* The page is not protected */
        }
    }

    for( uint32_t firstPage = 0u; UT_RAMCFG_SRAM2_PAGES > firstPage; firstPage ++ )
    {
        for( uint32_t pageCnt = 1u; ( UT_RAMCFG_SRAM2_PAGES - firstPage ) >= pageCnt; pageCnt ++ )
        {
            ramCfg_FunctionState_t expected   = RAMCFG_FUNCTION_ACTIVE;
            ramCfg_FunctionState_t protection = (ramCfg_FunctionState_t)UT_RAMCFG_SENTINEL;

            for( uint32_t pageIdx = firstPage; ( firstPage + pageCnt ) > pageIdx; pageIdx ++ )
            {
                if( UT_RAMCFG_HOLE_PAGE == ( pageIdx % UT_RAMCFG_HOLE_PERIOD ) )
                {
                    expected = RAMCFG_FUNCTION_INACTIVE;
                }
                else
                {
                    /* The page is protected */
                }
            }

            TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK,
                               RamCfg_Reg_Get_WriteProtection( RAMCFG_SRAM_2,
                                                               (ramCfg_Page_t)firstPage,
                                                               (ramCfg_PageCnt_t)pageCnt,
                                                               &protection ) );
            TEST_ASSERT_EQUAL( expected, protection );
        }
    }
}


/**
 * \brief   RamCfg_Reg_Get_WriteProtection() refuses invalid arguments and does not change the output.
 *
 * \details Calls the function with a NULL pointer, an empty range, ranges beyond the pages and the SRAMs beyond the
 *          list (the output holds a sentinel).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR in every case, the output is not changed.
 */
void Ut_RamCfg_Reg_Get_WriteProtection_InvalidArguments_ReturnsErrorOutputUnchanged( void )
{
    ramCfg_FunctionState_t protection = (ramCfg_FunctionState_t)UT_RAMCFG_SENTINEL;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, 1u, NULL ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Get_WriteProtection( RAMCFG_SRAM_2, 0u, RAMCFG_PAGE_CNT_UNUSED, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Get_WriteProtection( RAMCFG_SRAM_2, UT_RAMCFG_SRAM2_PAGES, 1u, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Get_WriteProtection( RAMCFG_SRAM_2,
                                                       UT_RAMCFG_SRAM2_PAGES - 1u,
                                                       UT_RAMCFG_RANGE_TWO_PAGES,
                                                       &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Get_WriteProtection( RAMCFG_SRAM_2,
                                                       UINT16_MAX,
                                                       UT_RAMCFG_RANGE_TWO_PAGES,
                                                       &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Get_WriteProtection( RAMCFG_SRAM_CNT, 0u, 1u, &protection ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR,
                       RamCfg_Reg_Get_WriteProtection( RAMCFG_SRAM_UNUSED, 0u, 1u, &protection ) );

    TEST_ASSERT_EQUAL( UT_RAMCFG_SENTINEL, protection );
}

/* ---------------------------------- Erase --------------------------------- */

/**
 * \brief   RamCfg_Reg_Set_EraseStart() writes the two keys, then sets the erase bit.
 *
 * \details The HW model accepts the erase bit only after the keys 0xCA and 0x53 were written one after another and
 *          holds the erase. For every SRAM a request of the erase without the keys is tried first (the model ignores
 *          it), then the function is called.
 *
 * \par Expected results
 * - The erase bit set without the keys does not stay set and starts nothing. RAMCFG_REQUEST_OK for the function, the
 *   erase started once, the erase bit and the busy flag set, the key register holds the second key, the first word of
 *   the SRAM is zero.
 */
void Ut_RamCfg_Reg_Set_EraseStart_Sram_WritesKeysThenEraseBit( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_ERASE_HOLD );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        RAMCFG_TypeDef * const      inst      = UtRamCfg_Get_Inst( sramId );
        volatile uint32_t * const   firstWord = (volatile uint32_t *)(uintptr_t)UtRamCfg_Get_Expect( sramId )->Base;

        *firstWord = UT_RAMCFG_DATA_PATTERN;
        inst->CR  |= UT_RAMCFG_CR_SRAMER;

        TEST_ASSERT_EQUAL_HEX32( 0u, inst->CR & UT_RAMCFG_CR_SRAMER );
        TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_EraseStartCnt[ sramId ] );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_DATA_PATTERN, *firstWord );

        TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Set_EraseStart( sramId ) );

        TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_ONCE, utRamCfg_EraseStartCnt[ sramId ] );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_SRAMER, inst->CR & UT_RAMCFG_CR_SRAMER );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ISR_SRAMBUSY, inst->ISR & UT_RAMCFG_ISR_SRAMBUSY );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ERASE_KEY_2, inst->ERKEYR );
        TEST_ASSERT_EQUAL_HEX32( 0u, *firstWord );
    }
}


/**
 * \brief   RamCfg_Reg_Set_EraseStart() refuses an invalid SRAM without any write.
 *
 * \details Requests the SRAMs beyond the list (the register block is filled with a pattern).
 *
 * \par Expected results
 * - RAMCFG_REQUEST_ERROR, no register written (neither a key nor the erase bit).
 */
void Ut_RamCfg_Reg_Set_EraseStart_InvalidSram_ReturnsErrorWithoutWrite( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_EraseStart( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Set_EraseStart( RAMCFG_SRAM_UNUSED ) );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   RamCfg_Reg_Get_EraseBusy() reports a running erase by the busy flag or by the erase bit.
 *
 * \details Reads the state of every SRAM with nothing set, with the busy flag, with the erase bit and with both, then
 *          for the SRAMs beyond the list (the register block is filled with a pattern which has the flag and the bit
 *          set).
 *
 * \par Expected results
 * - RAMCFG_FLAG_ACTIVE while the busy flag or the erase bit is set, otherwise RAMCFG_FLAG_INACTIVE (also for an invalid
 *   SRAM).
 */
void Ut_RamCfg_Reg_Get_EraseBusy_BusyFlagOrEraseBit_ReturnsFlag( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

        inst->CR  = 0u;
        inst->ISR = 0u;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_EraseBusy( sramId ) );

        inst->ISR = UT_RAMCFG_ISR_SRAMBUSY;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_ACTIVE, RamCfg_Reg_Get_EraseBusy( sramId ) );

        inst->CR  = UT_RAMCFG_CR_SRAMER;
        inst->ISR = 0u;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_ACTIVE, RamCfg_Reg_Get_EraseBusy( sramId ) );

        inst->ISR = UT_RAMCFG_ISR_SRAMBUSY;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_ACTIVE, RamCfg_Reg_Get_EraseBusy( sramId ) );

        inst->CR  = UT_RAMCFG_CR_ALE | UT_RAMCFG_CR_ECCE;
        inst->ISR = UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_EraseBusy( sramId ) );
    }

    UtRamCfg_Fill_Block( UT_RAMCFG_VALUE_ALL_ONES );

    TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_EraseBusy( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_EraseBusy( RAMCFG_SRAM_UNUSED ) );
}


/**
 * \brief   RamCfg_Reg_Wait_EraseEnd() returns at once when no erase runs and reports a timeout for a running one.
 *
 * \details Waits for the end with nothing set, with the busy flag set and with the erase bit set, then for the SRAMs
 *          beyond the list.
 *
 * \par Expected results
 * - RAMCFG_REQUEST_OK without a running erase, RAMCFG_REQUEST_ERROR after the timeout while the busy flag or the erase
 *   bit stays set (the erase is not stopped), RAMCFG_REQUEST_ERROR for an invalid SRAM.
 */
void Ut_RamCfg_Reg_Wait_EraseEnd_BusyOrIdle_ReturnsOkOrTimeout( void )
{
    RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( RAMCFG_SRAM_2 );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Reg_Wait_EraseEnd( RAMCFG_SRAM_2 ) );

    inst->ISR = UT_RAMCFG_ISR_SRAMBUSY;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Wait_EraseEnd( RAMCFG_SRAM_2 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ISR_SRAMBUSY, inst->ISR );

    inst->ISR = 0u;
    inst->CR  = UT_RAMCFG_CR_SRAMER;
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Wait_EraseEnd( RAMCFG_SRAM_2 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_CR_SRAMER, inst->CR );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Wait_EraseEnd( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Reg_Wait_EraseEnd( RAMCFG_SRAM_UNUSED ) );
}

/* --------------------------------- Flags ---------------------------------- */

/**
 * \brief   RamCfg_Reg_Get_SingleFlag() and RamCfg_Reg_Get_DoubleFlag() return their own flag only.
 *
 * \details Reads both flags of every SRAM with none, one and both flags set in the status register, then for the SRAMs
 *          beyond the list (the register block is filled with a pattern).
 *
 * \par Expected results
 * - Every function is RAMCFG_FLAG_ACTIVE only while its own flag is set, RAMCFG_FLAG_INACTIVE for an invalid SRAM.
 */
void Ut_RamCfg_Reg_GetFlags_StatusRegister_ReturnsOwnFlag( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

        inst->ISR = 0u;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_SingleFlag( sramId ) );
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_DoubleFlag( sramId ) );

        inst->ISR = UT_RAMCFG_ISR_SEDC;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_ACTIVE, RamCfg_Reg_Get_SingleFlag( sramId ) );
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_DoubleFlag( sramId ) );

        inst->ISR = UT_RAMCFG_ISR_DED;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_SingleFlag( sramId ) );
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_ACTIVE, RamCfg_Reg_Get_DoubleFlag( sramId ) );

        inst->ISR = UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_ACTIVE, RamCfg_Reg_Get_SingleFlag( sramId ) );
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_ACTIVE, RamCfg_Reg_Get_DoubleFlag( sramId ) );

        inst->ISR = UT_RAMCFG_ISR_SRAMBUSY;
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_SingleFlag( sramId ) );
        TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_DoubleFlag( sramId ) );
    }

    UtRamCfg_Fill_Block( UT_RAMCFG_VALUE_ALL_ONES );

    TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_SingleFlag( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL( RAMCFG_FLAG_INACTIVE, RamCfg_Reg_Get_DoubleFlag( RAMCFG_SRAM_UNUSED ) );
}


/**
 * \brief   The functions which clear the flags write the clear register as a whole.
 *
 * \details The bits of the clear register read as the flags. Presets the clear register of every SRAM with both bits
 *          set (as a read of the flags would return them) and clears the single error flag, then again for the double
 *          error flag. Requests the SRAMs beyond the list (the register block is filled with a pattern).
 *
 * \par Expected results
 * - The clear register holds only the bit of the flag to clear (a read-modify-write would write both bits and clear the
 *   other flag as well), the clear registers of the other SRAMs and all other registers are not written, the SRAMs
 *   beyond the list are not written.
 */
void Ut_RamCfg_Reg_ClearFlags_PresetClearRegister_WritesOnlyBitOfFlag( void )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

        inst->ICR = UT_RAMCFG_ICR_CSEDC | UT_RAMCFG_ICR_CDED;
        RamCfg_Reg_Clear_SingleFlag( sramId );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ICR_CSEDC, inst->ICR );

        inst->ICR = UT_RAMCFG_ICR_CSEDC | UT_RAMCFG_ICR_CDED;
        RamCfg_Reg_Clear_DoubleFlag( sramId );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ICR_CDED, inst->ICR );

        inst->ICR = 0u;
        RamCfg_Reg_Clear_SingleFlag( sramId );
        UtRamCfg_Check_OnlyWord( &inst->ICR, UT_RAMCFG_ICR_CSEDC );

        inst->ICR = 0u;
    }

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    RamCfg_Reg_Clear_SingleFlag( RAMCFG_SRAM_CNT );
    RamCfg_Reg_Clear_DoubleFlag( RAMCFG_SRAM_CNT );
    RamCfg_Reg_Clear_SingleFlag( RAMCFG_SRAM_UNUSED );
    RamCfg_Reg_Clear_DoubleFlag( RAMCFG_SRAM_UNUSED );

    UtRamCfg_Check_BlockUnchanged( block );
}


/**
 * \brief   The functions which clear the flags clear one flag and keep the other (hardware behavior emulated by the HW
 *          model).
 *
 * \details The HW model clears a flag of the status register when 1 is written to its clear bit and mirrors the flags
 *          in the clear register. Raises both errors in every SRAM with ECC, clears the single error flag, then the
 *          double error flag.
 *
 * \par Expected results
 * - After the first call only the double error flag is set, after the second call no flag is set.
 */
void Ut_RamCfg_Reg_ClearFlags_BothFlagsSet_ClearsOneFlagAtATime( void )
{
    UtRamCfg_Start_Model( UT_RAMCFG_MODEL_NONE );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

        if( RAMCFG_FUNCTION_ACTIVE == expect->Ecc )
        {
            RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

            UtRamCfg_Raise_Single( sramId, expect->Base );
            UtRamCfg_Raise_Double( sramId, expect->Base );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED, inst->ISR );

            RamCfg_Reg_Clear_SingleFlag( sramId );
            TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_ISR_DED, inst->ISR );

            RamCfg_Reg_Clear_DoubleFlag( sramId );
            TEST_ASSERT_EQUAL_HEX32( 0u, inst->ISR );
        }
        else
        {
            /* The SRAM has no ECC */
        }
    }
}


/**
 * \brief   RamCfg_Reg_Get_SingleAddress() and RamCfg_Reg_Get_DoubleAddress() return the whole address registers.
 *
 * \details Presets the address registers of every SRAM with distinct values (and the all-ones value) and reads them,
 *          then requests the SRAMs beyond the list (the register block is filled with a pattern).
 *
 * \par Expected results
 * - The function returns the content of its own register (the single and the double error address are not mixed up, all
 *   32 bits are returned), zero for an invalid SRAM.
 */
void Ut_RamCfg_Reg_GetAddresses_AddressRegisters_ReturnWholeRegister( void )
{
    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

        inst->SEAR = UT_RAMCFG_VALUE_SEAR;
        inst->DEAR = UT_RAMCFG_VALUE_DEAR;

        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_SEAR, RamCfg_Reg_Get_SingleAddress( sramId ) );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_DEAR, RamCfg_Reg_Get_DoubleAddress( sramId ) );

        inst->SEAR = UT_RAMCFG_VALUE_ALL_ONES;
        inst->DEAR = UT_RAMCFG_VALUE_ALL_ONES;

        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_ALL_ONES, RamCfg_Reg_Get_SingleAddress( sramId ) );
        TEST_ASSERT_EQUAL_HEX32( UT_RAMCFG_VALUE_ALL_ONES, RamCfg_Reg_Get_DoubleAddress( sramId ) );
    }

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );

    TEST_ASSERT_EQUAL_HEX32( 0u, RamCfg_Reg_Get_SingleAddress( RAMCFG_SRAM_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, RamCfg_Reg_Get_DoubleAddress( RAMCFG_SRAM_UNUSED ) );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/* ------------------------------- NVIC stubs ------------------------------- */

/** Stub of Nvic_Set_PeriphIrq_Handler - stores the registered RAMCFG interrupt handler */
static nvic_RequestState_t UtRamCfg_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId,
                                                        const nvic_IsrCallback_t irqHandler,
                                                        int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_RAMCFG, irqId );

    utRamCfg_HandlerCnt ++;

    if( NVIC_REQUEST_OK == utRamCfg_NvicHandlerRet )
    {
        utRamCfg_Isr = irqHandler;
    }
    else
    {
        /* The handler is refused */
    }

    return ( utRamCfg_NvicHandlerRet );
}


/** Stub of Nvic_Set_PeriphIrq_Prio - stores the priority of the RAMCFG interrupt */
static nvic_RequestState_t UtRamCfg_NvicSetPrioStub( nvic_PeriphIrqList_t irqId, nvic_IrqPrio_t irqPrio, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_RAMCFG, irqId );

    utRamCfg_PrioCnt ++;

    if( NVIC_REQUEST_OK == utRamCfg_NvicPrioRet )
    {
        utRamCfg_Prio = irqPrio;
    }
    else
    {
        /* The priority is refused */
    }

    return ( utRamCfg_NvicPrioRet );
}


/** Stub of Nvic_Get_PeriphIrq_Prio - returns the stored priority of the RAMCFG interrupt */
static nvic_RequestState_t UtRamCfg_NvicGetPrioStub( nvic_PeriphIrqList_t irqId, nvic_IrqPrio_t * irqPrio, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_RAMCFG, irqId );

    utRamCfg_GetPrioCnt ++;

    if( NVIC_REQUEST_OK == utRamCfg_NvicGetPrioRet )
    {
        *irqPrio = utRamCfg_Prio;
    }
    else
    {
        /* The priority is not returned */
    }

    return ( utRamCfg_NvicGetPrioRet );
}


/** Stub of Nvic_Set_PeriphIrq_Active - activates the RAMCFG interrupt */
static nvic_RequestState_t UtRamCfg_NvicActiveStub( nvic_PeriphIrqList_t irqId, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_RAMCFG, irqId );

    utRamCfg_ActiveCnt ++;

    if( NVIC_REQUEST_OK == utRamCfg_NvicActiveRet )
    {
        utRamCfg_VectorActive = UT_RAMCFG_FLAG_SET;
    }
    else
    {
        /* The activation is refused */
    }

    return ( utRamCfg_NvicActiveRet );
}


/** Stub of Nvic_Set_PeriphIrq_Inactive - deactivates the RAMCFG interrupt */
static nvic_RequestState_t UtRamCfg_NvicInactiveStub( nvic_PeriphIrqList_t irqId, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_RAMCFG, irqId );

    utRamCfg_InactiveCnt ++;

    if( NVIC_REQUEST_OK == utRamCfg_NvicInactiveRet )
    {
        utRamCfg_VectorActive = UT_RAMCFG_FLAG_CLEAR;
    }
    else
    {
        /* The deactivation is refused */
    }

    return ( utRamCfg_NvicInactiveRet );
}


/** Stub of Nvic_Set_CoreIrq_Handler - stores the NMI handler and remembers whether it is the previous one */
static nvic_RequestState_t UtRamCfg_NvicSetCoreStub( nvic_CoreIrqList_t irqId,
                                                     const nvic_IsrCallback_t irqHandler,
                                                     int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_CORE_IRQ_NMI, irqId );

    utRamCfg_CoreSetCnt ++;

    if( NVIC_REQUEST_OK == utRamCfg_NvicSetCoreRet )
    {
        utRamCfg_NmiIsr = irqHandler;

        if( utRamCfg_PrevNmi != irqHandler )
        {
            utRamCfg_NmiInstalled = UT_RAMCFG_FLAG_SET;
        }
        else
        {
            utRamCfg_NmiInstalled = UT_RAMCFG_FLAG_CLEAR;
        }
    }
    else
    {
        /* The NMI handler is refused */
    }

    return ( utRamCfg_NvicSetCoreRet );
}


/** Stub of Nvic_Get_CoreIrq_Handler - returns the NMI handler which was present before the module */
static nvic_RequestState_t UtRamCfg_NvicGetCoreStub( nvic_CoreIrqList_t irqId,
                                                     nvic_IsrCallback_t * irqHandler,
                                                     int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_CORE_IRQ_NMI, irqId );

    utRamCfg_CoreGetCnt ++;

    if( NVIC_REQUEST_OK == utRamCfg_NvicGetCoreRet )
    {
        *irqHandler = utRamCfg_PrevNmi;
    }
    else
    {
        /* The NMI handler is not returned */
    }

    return ( utRamCfg_NvicGetCoreRet );
}

/* -------------------------------- RCC stub -------------------------------- */

/** Stub of Rcc_Set_PeriphActive - records the activated clocks */
static rcc_RequestState_t UtRamCfg_RccActiveStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;

    if( UT_RAMCFG_RCC_LIST_SIZE > utRamCfg_RccCnt )
    {
        utRamCfg_RccList[ utRamCfg_RccCnt ] = periphId;
    }
    else
    {
        /* The list is full, the count still grows */
    }

    utRamCfg_RccCnt ++;

    return ( utRamCfg_RccRet );
}

/* ------------------------------- Callbacks -------------------------------- */

/** NMI handler present before the module (a function with a body) */
static void UtRamCfg_PrevNmiHandler( void )
{
    return;
}


/** Single error callback - records the call and the state of the flag at the call */
static void UtRamCfg_SingleCallback( ramCfg_SramId_t sramId, ramCfg_Address_t address )
{
    utRamCfg_SingleCnt ++;
    utRamCfg_SingleSram       = sramId;
    utRamCfg_SingleAddr       = address;
    utRamCfg_SingleFlagAtCall = ( UtRamCfg_Get_Inst( sramId )->ISR & UT_RAMCFG_ISR_SEDC );
}


/** Double error callback - records the call and the state of the flag at the call */
static void UtRamCfg_DoubleCallback( ramCfg_SramId_t sramId, ramCfg_Address_t address )
{
    utRamCfg_DoubleCnt ++;
    utRamCfg_DoubleSram       = sramId;
    utRamCfg_DoubleAddr       = address;
    utRamCfg_DoubleFlagAtCall = ( UtRamCfg_Get_Inst( sramId )->ISR & UT_RAMCFG_ISR_DED );
}


/** Erase complete callback - records the call */
static void UtRamCfg_EraseCallback( ramCfg_SramId_t sramId )
{
    utRamCfg_EraseCnt ++;
    utRamCfg_EraseSram = sramId;
}

/* ------------------------- Facts and registers ---------------------------- */

/** Returns the expected features of an SRAM (restated from the reference manuals) */
static const utRamCfg_Sram_t * UtRamCfg_Get_Expect( ramCfg_SramId_t sramId )
{
    TEST_ASSERT_TRUE( RAMCFG_SRAM_CNT > sramId );

    return ( &utRamCfg_SramList[ sramId ] );
}


/** Returns the register structure of an SRAM at the address of the reference manual (block address + offset of the
 *  memory) */
static RAMCFG_TypeDef * UtRamCfg_Get_Inst( ramCfg_SramId_t sramId )
{
    const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

    return ( (RAMCFG_TypeDef *)(uintptr_t)( UT_RAMCFG_BLOCK_ADDR + expect->RegOffset ) );
}


/** Returns the address of a register of an SRAM (block address + offset of the memory + offset of the register) */
static volatile uint32_t * UtRamCfg_Get_RegPtr( ramCfg_SramId_t sramId, uint32_t regOffset )
{
    const utRamCfg_Sram_t * const expect = UtRamCfg_Get_Expect( sramId );

    return ( (volatile uint32_t *)(uintptr_t)( UT_RAMCFG_BLOCK_ADDR + expect->RegOffset + regOffset ) );
}

/* --------------------------- Configuration helpers ------------------------ */

/** Returns the default configuration (the function is asserted to succeed) */
static ramCfg_Config_t UtRamCfg_Get_Config( void )
{
    ramCfg_Config_t config;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Get_DefaultConfig( &config ) );

    return ( config );
}


/** Initializes the module with a configuration (the initialization is asserted to succeed) */
static void UtRamCfg_Init_Config( const ramCfg_Config_t * const config )
{
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_OK, RamCfg_Init( config ) );
}


/** Initializes the module with the default configuration */
static void UtRamCfg_Init_Default( void )
{
    const ramCfg_Config_t config = UtRamCfg_Get_Config();

    UtRamCfg_Init_Config( &config );
}


/** Initializes the module with the default configuration except the settings of one SRAM */
static void UtRamCfg_Init_Sram( ramCfg_SramId_t sramId,
                                ramCfg_CheckMode_t checkMode,
                                ramCfg_ReportMode_t reportMode,
                                ramCfg_FunctionState_t scrubbing )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ sramId ].CheckMode  = checkMode;
    config.Sram[ sramId ].ReportMode = reportMode;
    config.Sram[ sramId ].Scrubbing  = scrubbing;

    UtRamCfg_Init_Config( &config );
}


/** Initializes the module with the default configuration, the reporting mode of one SRAM and all callbacks */
static void UtRamCfg_Init_Callbacks( ramCfg_SramId_t sramId, ramCfg_ReportMode_t reportMode )
{
    ramCfg_Config_t config = UtRamCfg_Get_Config();

    config.Sram[ sramId ].ReportMode = reportMode;
    config.SingleErrorCallback       = UtRamCfg_SingleCallback;
    config.DoubleErrorCallback       = UtRamCfg_DoubleCallback;
    config.EraseCompleteCallback     = UtRamCfg_EraseCallback;

    UtRamCfg_Init_Config( &config );
}

/* ----------------------------- Register block ----------------------------- */

/** Fills the whole register block with a pattern (a distinct value in every word) */
static void UtRamCfg_Fill_Block( uint32_t seed )
{
    volatile uint32_t * const block = (volatile uint32_t *)(uintptr_t)UT_RAMCFG_BLOCK_ADDR;

    for( uint32_t wordIdx = 0u; UT_RAMCFG_BLOCK_WORDS > wordIdx; wordIdx ++ )
    {
        block[ wordIdx ] = ( seed + ( wordIdx * UT_RAMCFG_FILL_STEP ) );
    }
}


/** Copies the register block */
static void UtRamCfg_Snapshot( uint32_t * const block )
{
    const volatile uint32_t * const regs = (const volatile uint32_t *)(uintptr_t)UT_RAMCFG_BLOCK_ADDR;

    for( uint32_t wordIdx = 0u; UT_RAMCFG_BLOCK_WORDS > wordIdx; wordIdx ++ )
    {
        block[ wordIdx ] = regs[ wordIdx ];
    }
}


/** Checks that the register block holds the copy (no register was written) */
static void UtRamCfg_Check_BlockUnchanged( const uint32_t * const block )
{
    const volatile uint32_t * const regs = (const volatile uint32_t *)(uintptr_t)UT_RAMCFG_BLOCK_ADDR;

    for( uint32_t wordIdx = 0u; UT_RAMCFG_BLOCK_WORDS > wordIdx; wordIdx ++ )
    {
        TEST_ASSERT_EQUAL_HEX32_MESSAGE( block[ wordIdx ], regs[ wordIdx ], "RAMCFG register block was changed" );
    }
}


/** Checks that one register of the block holds the value and every other word of the block is zero */
static void UtRamCfg_Check_OnlyWord( const volatile uint32_t * const word, uint32_t value )
{
    const volatile uint32_t * const regs = (const volatile uint32_t *)(uintptr_t)UT_RAMCFG_BLOCK_ADDR;

    for( uint32_t wordIdx = 0u; UT_RAMCFG_BLOCK_WORDS > wordIdx; wordIdx ++ )
    {
        if( word == &regs[ wordIdx ] )
        {
            TEST_ASSERT_EQUAL_HEX32( value, regs[ wordIdx ] );
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX32_MESSAGE( 0u, regs[ wordIdx ], "another register of the block was written" );
        }
    }
}


/** Checks that the module is not initialized (a getter which needs the initialization returns an error) */
static void UtRamCfg_Check_NotInitialized( void )
{
    ramCfg_CheckMode_t checkMode = (ramCfg_CheckMode_t)UT_RAMCFG_SENTINEL;

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Get_CheckMode( RAMCFG_SRAM_2, &checkMode ) );
}


/** Checks that the RCC and NVIC modules were not called */
static void UtRamCfg_Check_NoOtherModuleCalls( void )
{
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_RccCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_HandlerCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_PrioCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_GetPrioCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_ActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_CoreSetCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_RAMCFG_CALLS_NONE, utRamCfg_CoreGetCnt );
}


/** Checks that an invalid configuration is rejected before any register write or call of another module */
static void UtRamCfg_Check_InitRejected( const ramCfg_Config_t * const config )
{
    uint32_t block[ UT_RAMCFG_BLOCK_WORDS ];

    UtRamCfg_Fill_Block( UT_RAMCFG_FILL_SEED );
    UtRamCfg_Snapshot( block );

    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Init( config ) );

    UtRamCfg_Check_BlockUnchanged( block );
    UtRamCfg_Check_NoOtherModuleCalls();
    UtRamCfg_Check_NotInitialized();
}


/** Checks that an initialization fails and leaves the module not initialized */
static void UtRamCfg_Check_InitFailed( const ramCfg_Config_t * const config )
{
    TEST_ASSERT_EQUAL( RAMCFG_REQUEST_ERROR, RamCfg_Init( config ) );

    UtRamCfg_Check_NotInitialized();
}

/* -------------------------------- HW model -------------------------------- */

/** Starts the HW model with the given behavior (the unlock sequences and the count of the stores need the synchronous
 *  model) */
static void UtRamCfg_Start_Model( uint32_t modelFlags )
{
    utRamCfg_ModelFlags = modelFlags;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtRamCfg_HwModel ) );

    const regMem_ModelMode_t modelMode = RegMem_Get_ModelMode();

    if( REGMEM_MODEL_MODE_SYNC != modelMode )
    {
        TEST_IGNORE_MESSAGE( "The HW model of RAMCFG (unlock sequences) needs the synchronous mode of RegMem" );
    }
    else
    {
        /* The model reacts to every store */
    }
}


/** Raises a single error in an SRAM: address register and flag (the HW model mirrors the flag to the clear register) */
static void UtRamCfg_Raise_Single( ramCfg_SramId_t sramId, uint32_t address )
{
    RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

    inst->SEAR = address;
    inst->ISR |= UT_RAMCFG_ISR_SEDC;
}


/** Raises a double error in an SRAM: address register and flag (the HW model mirrors the flag to the clear register) */
static void UtRamCfg_Raise_Double( ramCfg_SramId_t sramId, uint32_t address )
{
    RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

    inst->DEAR = address;
    inst->ISR |= UT_RAMCFG_ISR_DED;
}


/** Raises a single error, runs the interrupt handler and returns the count of the stores the handler made */
static uint32_t UtRamCfg_Run_SingleIsr( ramCfg_SramId_t sramId, uint32_t address )
{
    UtRamCfg_Raise_Single( sramId, address );

    TEST_ASSERT_NOT_NULL( utRamCfg_Isr );

    const uint32_t callsBefore = utRamCfg_ModelCalls;

    utRamCfg_Isr();

    return ( utRamCfg_ModelCalls - callsBefore );
}


/** Ends a held erase: clears the erase bit and the busy flag as the hardware does at the end of the erase */
static void UtRamCfg_Finish_Erase( ramCfg_SramId_t sramId )
{
    RAMCFG_TypeDef * const inst = UtRamCfg_Get_Inst( sramId );

    inst->CR  &= ~UT_RAMCFG_CR_SRAMER;
    inst->ISR &= ~UT_RAMCFG_ISR_SRAMBUSY;
}

/* ---------------------------- Write protection ---------------------------- */

/** Returns the bits of a write protection register which belong to a range of pages (page n is the bit n modulo 32 of
 *  the register n / 32) */
static uint32_t UtRamCfg_Get_PageMask( uint32_t wprIdx, uint32_t firstPage, uint32_t pageCnt )
{
    uint32_t pageMask = 0u;

    for( uint32_t pageIdx = firstPage; ( firstPage + pageCnt ) > pageIdx; pageIdx ++ )
    {
        if( wprIdx == ( pageIdx / UT_RAMCFG_PAGES_PER_WPR ) )
        {
            pageMask |= ( UT_RAMCFG_BIT_ONE << ( pageIdx % UT_RAMCFG_PAGES_PER_WPR ) );
        }
        else
        {
            /* The page belongs to another register */
        }
    }

    return ( pageMask );
}


/** Reads a write protection register of an SRAM at the address of the reference manual */
static uint32_t UtRamCfg_Get_WprReg( ramCfg_SramId_t sramId, uint32_t wprIdx )
{
    TEST_ASSERT_TRUE( UT_RAMCFG_WPR_CNT > wprIdx );

    return ( *UtRamCfg_Get_RegPtr( sramId, utRamCfg_WprOffsetList[ wprIdx ] ) );
}


/** Checks that the write protection registers of an SRAM protect exactly the pages of the range */
static void UtRamCfg_Check_Wpr( ramCfg_SramId_t sramId, uint32_t firstPage, uint32_t pageCnt )
{
    for( uint32_t wprIdx = 0u; UT_RAMCFG_WPR_CNT > wprIdx; wprIdx ++ )
    {
        TEST_ASSERT_EQUAL_HEX32_MESSAGE( UtRamCfg_Get_PageMask( wprIdx, firstPage, pageCnt ),
                                         UtRamCfg_Get_WprReg( sramId, wprIdx ),
                                         "write protection register" );
    }
}


/** Clears all write protection registers of an SRAM (the hardware clears them only by a reset of the peripheral) */
static void UtRamCfg_Clear_Wpr( ramCfg_SramId_t sramId )
{
    for( uint32_t wprIdx = 0u; UT_RAMCFG_WPR_CNT > wprIdx; wprIdx ++ )
    {
        *UtRamCfg_Get_RegPtr( sramId, utRamCfg_WprOffsetList[ wprIdx ] ) = 0u;
    }
}

/* ------------------------------ HW model body ----------------------------- */

/**
 * HW model of the SRAM controller (evaluated after every store to the emulated registers):
 * - unlock sequences: the ECC enable bit is cleared and the erase bit is set only after the two keys were written to
 *   the key register one after another (a wrong key locks again), otherwise the write is ignored,
 * - erase: accepted start zeroes the first word of the SRAM and sets the busy flag, the erase ends at once (or is
 *   held),
 * - flags: a write of 1 to the clear register clears the flag, the clear register reads as the flags,
 * - registers which do not keep the written value (selected by the behavior flags).
 */
static void UtRamCfg_HwModel( void )
{
    (void)__atomic_add_fetch( &utRamCfg_ModelCalls, 1u, __ATOMIC_SEQ_CST );

    for( ramCfg_SramId_t sramId = RAMCFG_SRAM_1; RAMCFG_SRAM_CNT > sramId; sramId ++ )
    {
        RAMCFG_TypeDef * const inst =
            (RAMCFG_TypeDef *)(uintptr_t)( UT_RAMCFG_BLOCK_ADDR + utRamCfg_SramList[ sramId ].RegOffset );

        UtRamCfg_Model_Ecc( sramId, inst );
        UtRamCfg_Model_Erase( sramId, inst );
        UtRamCfg_Model_Flags( sramId, inst );
        UtRamCfg_Model_NotKept( sramId, inst );
    }
}


/**
 * Unlock sequence of a key register: a new value of the register is a written key. The state follows
 * locked - first key - unlocked, a wrong key locks again. Returns UT_RAMCFG_FLAG_SET when the sequence was
 * completed by this evaluation.
 */
static uint32_t UtRamCfg_Model_Key( volatile uint32_t * const key,
                                    volatile uint32_t * const prev,
                                    volatile uint32_t * const state,
                                    uint32_t locked,
                                    uint32_t key1,
                                    uint32_t key2 )
{
    uint32_t       unlockedNow = UT_RAMCFG_FLAG_CLEAR;
    const uint32_t keyValue    = *key;

    if( keyValue != *prev )
    {
        *prev = keyValue;

        if( ( UT_RAMCFG_FLAG_CLEAR == locked   ) &&
            ( key1                 == keyValue )    )
        {
            *state = UT_RAMCFG_KEY_FIRST;
        }
        else if( ( UT_RAMCFG_FLAG_CLEAR == locked   ) &&
                 ( UT_RAMCFG_KEY_FIRST  == *state   ) &&
                 ( key2                 == keyValue )    )
        {
            *state      = UT_RAMCFG_KEY_UNLOCKED;
            unlockedNow = UT_RAMCFG_FLAG_SET;
        }
        else
        {
            *state = UT_RAMCFG_KEY_LOCKED;
        }
    }
    else
    {
        /* No new key was written */
    }

    return ( unlockedNow );
}


/** Model of the ECC enable bit: cleared only after the unlock sequence, can be made stuck or locked */
static void UtRamCfg_Model_Ecc( ramCfg_SramId_t sramId, RAMCFG_TypeDef * const inst )
{
    const uint32_t modelFlags = utRamCfg_ModelFlags;
    uint32_t       locked     = UT_RAMCFG_FLAG_CLEAR;
    uint32_t       eccBit     = UT_RAMCFG_FLAG_CLEAR;

    if( UT_RAMCFG_MODEL_NONE != ( modelFlags & UT_RAMCFG_MODEL_ECC_LOCKED ) )
    {
        locked = UT_RAMCFG_FLAG_SET;
    }
    else
    {
        /* The unlock sequence is accepted */
    }

    const uint32_t unlockedNow = UtRamCfg_Model_Key( &inst->ECCKEY,
                                                     &utRamCfg_EccKeyPrev[ sramId ],
                                                     &utRamCfg_EccKeyState[ sramId ],
                                                     locked,
                                                     UT_RAMCFG_ECC_KEY_1,
                                                     UT_RAMCFG_ECC_KEY_2 );

    if( UT_RAMCFG_FLAG_SET == unlockedNow )
    {
        utRamCfg_EccUnlockCnt[ sramId ] ++;
    }
    else
    {
        /* The sequence is not complete */
    }

    eccBit = ( inst->CR & UT_RAMCFG_CR_ECCE );

    if( ( UT_RAMCFG_MODEL_NONE != ( modelFlags & UT_RAMCFG_MODEL_ECC_STUCK ) ) &&
        ( UT_RAMCFG_FLAG_CLEAR != eccBit                                    )    )
    {
        /* The bit cannot be set */
        (void)__atomic_and_fetch( &inst->CR, (uint32_t)~UT_RAMCFG_CR_ECCE, __ATOMIC_SEQ_CST );
        eccBit = UT_RAMCFG_FLAG_CLEAR;
    }
    else if( ( UT_RAMCFG_FLAG_CLEAR == eccBit                           ) &&
             ( UT_RAMCFG_FLAG_CLEAR != utRamCfg_EccePrev[ sramId ]      )    )
    {
        /* The bit was cleared */
        if( UT_RAMCFG_KEY_UNLOCKED == utRamCfg_EccKeyState[ sramId ] )
        {
            utRamCfg_EccKeyState[ sramId ] = UT_RAMCFG_KEY_LOCKED;
        }
        else
        {
            /* Not unlocked: the write is ignored */
            (void)__atomic_or_fetch( &inst->CR, UT_RAMCFG_CR_ECCE, __ATOMIC_SEQ_CST );
            eccBit = UT_RAMCFG_CR_ECCE;
        }
    }
    else
    {
        /* The bit did not change */
    }

    utRamCfg_EccePrev[ sramId ] = eccBit;
}


/** Model of the erase: the start needs the unlock sequence, the erase ends at once unless the model holds it */
static void UtRamCfg_Model_Erase( ramCfg_SramId_t sramId, RAMCFG_TypeDef * const inst )
{
    const uint32_t modelFlags = utRamCfg_ModelFlags;

    (void)UtRamCfg_Model_Key( &inst->ERKEYR,
                              &utRamCfg_EraseKeyPrev[ sramId ],
                              &utRamCfg_EraseKeyState[ sramId ],
                              UT_RAMCFG_FLAG_CLEAR,
                              UT_RAMCFG_ERASE_KEY_1,
                              UT_RAMCFG_ERASE_KEY_2 );

    uint32_t eraseBit = ( inst->CR & UT_RAMCFG_CR_SRAMER );

    if( ( UT_RAMCFG_FLAG_CLEAR != eraseBit                           ) &&
        ( UT_RAMCFG_FLAG_CLEAR == utRamCfg_EraseBitPrev[ sramId ]    )    )
    {
        /* The software requested the erase */
        if( UT_RAMCFG_KEY_UNLOCKED == utRamCfg_EraseKeyState[ sramId ] )
        {
            volatile uint32_t * const firstWord = (volatile uint32_t *)(uintptr_t)utRamCfg_SramList[ sramId ].Base;

            utRamCfg_EraseKeyState[ sramId ] = UT_RAMCFG_KEY_LOCKED;
            utRamCfg_EraseStartCnt[ sramId ] ++;

            *firstWord = 0u;
            (void)__atomic_or_fetch( &inst->ISR, UT_RAMCFG_ISR_SRAMBUSY, __ATOMIC_SEQ_CST );

            if( UT_RAMCFG_MODEL_NONE == ( modelFlags & UT_RAMCFG_MODEL_ERASE_HOLD ) )
            {
                /* The erase is finished at once */
                (void)__atomic_and_fetch( &inst->CR, (uint32_t)~UT_RAMCFG_CR_SRAMER, __ATOMIC_SEQ_CST );
                (void)__atomic_and_fetch( &inst->ISR, (uint32_t)~UT_RAMCFG_ISR_SRAMBUSY, __ATOMIC_SEQ_CST );
                eraseBit = UT_RAMCFG_FLAG_CLEAR;
            }
            else
            {
                /* The erase runs until the test releases it */
            }
        }
        else
        {
            /* Not unlocked: the write is ignored */
            (void)__atomic_and_fetch( &inst->CR, (uint32_t)~UT_RAMCFG_CR_SRAMER, __ATOMIC_SEQ_CST );
            eraseBit = UT_RAMCFG_FLAG_CLEAR;
        }
    }
    else
    {
        /* No request of the erase */
    }

    utRamCfg_EraseBitPrev[ sramId ] = eraseBit;
}


/**
 * Model of the flags: a write of 1 to the clear register clears the flag of the status register. The clear register
 * reads as the flags (RM: "reading this flag returns the value"), the model keeps the flags in it together with a
 * reserved marker bit, so a store of the software (a value without the marker) is told from the mirror. The model also
 * records the state of the address latch and of the NMI handler when the error reporting is enabled first.
 */
static void UtRamCfg_Model_Flags( ramCfg_SramId_t sramId, RAMCFG_TypeDef * const inst )
{
    const uint32_t icrValue = inst->ICR;

    if( icrValue != utRamCfg_IcrMirror[ sramId ] )
    {
        const uint32_t clearBits = ( icrValue & ( UT_RAMCFG_ICR_CSEDC | UT_RAMCFG_ICR_CDED ) );

        (void)__atomic_and_fetch( &inst->ISR, (uint32_t)~clearBits, __ATOMIC_SEQ_CST );

        if( UT_RAMCFG_FLAG_CLEAR != ( clearBits & UT_RAMCFG_ICR_CSEDC ) )
        {
            utRamCfg_ClearAtCall = utRamCfg_ModelCalls;
        }
        else
        {
            /* The flag of the single error was not cleared */
        }
    }
    else
    {
        /* The clear register was not written */
    }

    const uint32_t mirrorValue = ( ( inst->ISR & ( UT_RAMCFG_ISR_SEDC | UT_RAMCFG_ISR_DED ) ) | UT_RAMCFG_ICR_MARK );

    __atomic_store_n( &inst->ICR, mirrorValue, __ATOMIC_SEQ_CST );
    utRamCfg_IcrMirror[ sramId ] = mirrorValue;

    const uint32_t ierValue = inst->IER;

    if( ( UT_RAMCFG_FLAG_CLEAR != ( ierValue & ( UT_RAMCFG_IER_IRQ | UT_RAMCFG_IER_ECCNMI ) ) ) &&
        ( UT_RAMCFG_FLAG_CLEAR == utRamCfg_IerSeen[ sramId ]                                  )    )
    {
        utRamCfg_IerSeen[ sramId ]        = UT_RAMCFG_FLAG_SET;
        utRamCfg_AleAtIerEnable[ sramId ] = UT_RAMCFG_FLAG_CLEAR;

        if( UT_RAMCFG_FLAG_CLEAR != ( inst->CR & UT_RAMCFG_CR_ALE ) )
        {
            utRamCfg_AleAtIerEnable[ sramId ] = UT_RAMCFG_FLAG_SET;
        }
        else
        {
            /* The address latch was not set before the interrupt */
        }
    }
    else
    {
        /* The error reporting is not enabled or was seen already */
    }

    if( ( UT_RAMCFG_FLAG_CLEAR != ( ierValue & UT_RAMCFG_IER_ECCNMI ) ) &&
        ( UT_RAMCFG_FLAG_CLEAR == utRamCfg_NmiEnableSeen              )    )
    {
        utRamCfg_NmiEnableSeen        = UT_RAMCFG_FLAG_SET;
        utRamCfg_NmiInstalledAtEnable = utRamCfg_NmiInstalled;
    }
    else
    {
        /* The NMI redirection is not enabled or was seen already */
    }
}


/** Model of the registers which do not keep the written value (selected by the behavior flags of the model) */
static void UtRamCfg_Model_NotKept( ramCfg_SramId_t sramId, RAMCFG_TypeDef * const inst )
{
    const uint32_t modelFlags = utRamCfg_ModelFlags;

    if( UT_RAMCFG_MODEL_NONE != ( modelFlags & UT_RAMCFG_MODEL_IER_NOT_KEPT ) )
    {
        (void)__atomic_and_fetch( &inst->IER, (uint32_t)~UT_RAMCFG_IER_IRQ, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* The error interrupt enables are kept */
    }

    uint32_t ierBits = ( inst->IER & UT_RAMCFG_IER_IRQ );

    if( ( UT_RAMCFG_MODEL_NONE != ( modelFlags & UT_RAMCFG_MODEL_IER_STUCK ) ) &&
        ( UT_RAMCFG_FLAG_CLEAR != utRamCfg_IerPrev[ sramId ]                 ) &&
        ( UT_RAMCFG_FLAG_CLEAR == ierBits                                    )    )
    {
        /* The enables cannot be cleared */
        ierBits = utRamCfg_IerPrev[ sramId ];
        (void)__atomic_or_fetch( &inst->IER, ierBits, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* The enables follow the written value */
    }

    utRamCfg_IerPrev[ sramId ] = ierBits;

    if( UT_RAMCFG_MODEL_NONE != ( modelFlags & UT_RAMCFG_MODEL_ALE_NOT_KEPT ) )
    {
        (void)__atomic_and_fetch( &inst->CR, (uint32_t)~UT_RAMCFG_CR_ALE, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* The address latch bit is kept */
    }

    if( ( UT_RAMCFG_MODEL_NONE != ( modelFlags & UT_RAMCFG_MODEL_WPR_NOT_KEPT ) ) &&
        ( RAMCFG_SRAM_2        == sramId                                        )    )
    {
        for( uint32_t wprIdx = 0u; UT_RAMCFG_WPR_CNT > wprIdx; wprIdx ++ )
        {
            volatile uint32_t * const wprReg =
                (volatile uint32_t *)(uintptr_t)( (uintptr_t)inst + utRamCfg_WprOffsetList[ wprIdx ] );

            __atomic_store_n( wprReg, 0u, __ATOMIC_SEQ_CST );
        }
    }
    else
    {
        /* The write protection bits are kept */
    }
}
