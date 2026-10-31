/**
 * \author Mr.Nobody
 * \file RamCfg_Reg.c
 * \ingroup RamCfg
 * \brief RamCfg module Reg component functionality.
 *
 * Register access of the module: generic accessors (bounds checked SRAM and register
 * identification) and the register protocols of the SRAM controller. The registers are
 * written only here, the masks come from the CMSIS device header. The address of a register
 * is the base address of the RAMCFG instance of the SRAM (RAMCFG_SRAM1, ... RAMCFG_BKPRAM)
 * plus the offset of the register in the CMSIS type RAMCFG_TypeDef.
 *
 * Properties of the registers, which the protocols respect (RM0481, RM0492, RM0517 - RAMCFG chapter):
 * - RAMCFG_CR.ECCE is cleared only after the unlock sequence in RAMCFG_ECCKEYR (0xAE, 0x75), a wrong
 *   key activates the protection again. The bit is set without a key.
 * - RAMCFG_CR.SRAMER is set only after the unlock sequence in RAMCFG_ERKEYR (0xCA, 0x53), it is cleared
 *   by the hardware at the end of the erase. RAMCFG_ISR.SRAMBUSY is set while the erase runs (also for the
 *   erase caused by a reset, a tamper event or a regression of the product state).
 * - RAMCFG_IER.ECCNMI is set only by software and cleared only by a reset of the RAMCFG peripheral.
 * - RAMCFG_WPRx.PyWP are set only by software and cleared only by a reset of the RAMCFG peripheral.
 * - The flags RAMCFG_ISR.SEDC and RAMCFG_ISR.DED are cleared by writing 1 to RAMCFG_ICR (CSEDC, CDED),
 *   the bits of RAMCFG_ICR read as the flags, the register is therefore never written by a
 *   read-modify-write (it would clear the flags which are set).
 * - The addresses RAMCFG_SEAR and RAMCFG_DEAR are updated only while the address latch (RAMCFG_CR.ALE) and
 *   the interrupt of the error (SEIE, DEIE or ECCNMI) are enabled (device errata ES0561 2.2.20).
 * - The registers RAMCFG_IER, RAMCFG_SEAR, RAMCFG_DEAR, RAMCFG_ICR, RAMCFG_ECCKEYR and the bits ECCE and ALE
 *   exist only for the SRAMs with the ECC, RAMCFG_WPRx for SRAM2 only. The module does not access them
 *   for the other SRAMs (the caller selects the SRAM by the features of the SRAM).
 * - The bits P56WP ... P62WP of RAMCFG_WPR2 and P88WP ... P94WP of RAMCFG_WPR3 of the CMSIS device headers
 *   are defined one bit higher than the reference manual says (the position of P62WP is the one of P63WP).
 *   The page masks are therefore built from the page index (bit = page index modulo 32), not from the CMSIS
 *   masks of the single pages.
 *
 */
/* ============================== INCLUDES ================================== */
#include "RamCfg_Reg.h"                     /* Self include                   */
#include "RamCfg_Types.h"                   /* Module types definitions       */
#include "Stm32.h"                          /* RAMCFG registers definition    */
#include "stddef.h"                         /* offsetof                       */
#include "limits.h"                         /* CHAR_BIT                       */
/* ============================== TYPEDEFS ================================== */

/** \brief Structure type used to store the RAMCFG instance of an SRAM.
 * This array is used to reduce size of configuration array. With this style
 * the instances are referenced through enumeration of the SRAMs.
 *
 */
typedef struct
{
    ramCfg_SramId_t SramId; /**< SRAM (position of the item in the table) */
    RAMCFG_TypeDef *Inst;   /**< RAMCFG registers of the SRAM             */
}   ramCfg_InstList_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static volatile uint32_t *      RamCfg_Get_RegAddr          ( ramCfg_SramId_t sramId, ramCfg_RegId_t regId );
static ramCfg_RequestState_t    RamCfg_Reg_Wait_RegVal      ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t regMask,
                                                              uint32_t regValue );
static ramCfg_RequestState_t    RamCfg_Reg_Set_Verified     ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t regMask,
                                                              uint32_t regValue );
static ramCfg_FunctionState_t   RamCfg_Reg_Get_BitState     ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t bitMask );
static ramCfg_FlagState_t       RamCfg_Reg_Get_FlagState    ( ramCfg_SramId_t sramId,
                                                              ramCfg_RegId_t regId,
                                                              uint32_t bitMask );
static uint32_t                 RamCfg_Reg_Calc_PageMask    ( uint32_t wprIdx,
                                                              ramCfg_Page_t firstPage,
                                                              ramCfg_PageCnt_t pageCnt );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Count of read-back iterations of a written register value */
#define RAMCFG_REG_READBACK_TIMEOUT_RAW     ( 0x1000u )

/** \brief Count of iterations of the wait for the end of the erase. The erase of the largest SRAM (384 Kbytes,
 *         98304 words) lasts one AHB clock cycle per word, an iteration of the loop takes several cycles. */
#define RAMCFG_REG_ERASE_TIMEOUT_RAW        ( 0x100000u )

/** \brief Register value without any bit set (masked register value of an inactive bit field) */
#define RAMCFG_REG_BITS_CLEAR               ( 0u )

/** \brief First key of the unlock sequence of the ECC enable bit */
#define RAMCFG_REG_ECC_KEY_1                ( 0xAEu )

/** \brief Second key of the unlock sequence of the ECC enable bit */
#define RAMCFG_REG_ECC_KEY_2                ( 0x75u )

/** \brief First key of the unlock sequence of the erase bit */
#define RAMCFG_REG_ERASE_KEY_1              ( 0xCAu )

/** \brief Second key of the unlock sequence of the erase bit */
#define RAMCFG_REG_ERASE_KEY_2              ( 0x53u )

/** \brief Bits of the interrupt enable register which select the reporting of the errors */
#define RAMCFG_REG_IER_MASK                 ( RAMCFG_IER_SEIE | RAMCFG_IER_DEIE | RAMCFG_IER_ECCNMI )

/** \brief Bits of the interrupt enable register of the reporting by the interrupt (single and double error) */
#define RAMCFG_REG_IER_IRQ                  ( RAMCFG_IER_SEIE | RAMCFG_IER_DEIE )

/**
 * \brief Bits of the interrupt enable register of the reporting of the double error by the NMI (single error by
 *        the interrupt)
 */
#define RAMCFG_REG_IER_NMI                  ( RAMCFG_IER_SEIE | RAMCFG_IER_ECCNMI )

/** \brief Count of the bits of a write protection register (the pages of one register) */
#define RAMCFG_REG_WPR_BIT_CNT              ( RAMCFG_REG_PAGES_PER_WPR )

/** \brief Value of a single bit at the position zero */
#define RAMCFG_REG_BIT_ONE                  ( 1u )

/* =============================== MACROS =================================== */

/** Positions the key to the field ECCKEY of RAMCFG_ECCKEYR */
#define RAMCFG_REG_ECC_KEY( key )           ( ( (uint32_t)( key ) << RAMCFG_ECCKEYR_ECCKEY_Pos ) & \
                                              RAMCFG_ECCKEYR_ECCKEY )

/** Positions the key to the field ERASEKEY of RAMCFG_ERKEYR */
#define RAMCFG_REG_ERASE_KEY( key )         ( ( (uint32_t)( key ) << RAMCFG_ERKEYR_ERASEKEY_Pos ) & \
                                              RAMCFG_ERKEYR_ERASEKEY )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief RAMCFG instances configuration structure
 *
 * This structure is used to store the RAMCFG registers of the SRAMs.
 * It is used to reduce code size while referring to the RAMCFG registers.
 */
static const ramCfg_InstList_t          ramCfg_InstList[] =
{
    [ RAMCFG_SRAM_1      ] = { .SramId = RAMCFG_SRAM_1     , .Inst = RAMCFG_SRAM1  }, /**< SRAM1 registers        */
    [ RAMCFG_SRAM_2      ] = { .SramId = RAMCFG_SRAM_2     , .Inst = RAMCFG_SRAM2  }, /**< SRAM2 registers        */
#if defined(RAMCFG_SRAM3)
    [ RAMCFG_SRAM_3      ] = { .SramId = RAMCFG_SRAM_3     , .Inst = RAMCFG_SRAM3  }, /**< SRAM3 registers        */
#endif
#if defined(RAMCFG_SRAM4)
    [ RAMCFG_SRAM_4      ] = { .SramId = RAMCFG_SRAM_4     , .Inst = RAMCFG_SRAM4  }, /**< SRAM4 registers        */
#endif
#if defined(RAMCFG_SRAM5)
    [ RAMCFG_SRAM_5      ] = { .SramId = RAMCFG_SRAM_5     , .Inst = RAMCFG_SRAM5  }, /**< SRAM5 registers        */
#endif
    [ RAMCFG_SRAM_BACKUP ] = { .SramId = RAMCFG_SRAM_BACKUP, .Inst = RAMCFG_BKPRAM }, /**< Backup SRAM registers  */
};

_Static_assert( (sizeof(ramCfg_InstList) / sizeof(ramCfg_InstList_t)) == RAMCFG_SRAM_CNT,
                "RamCfg_Reg: ramCfg_InstList has incorrect size." );


/** \brief Offsets of the RAMCFG registers in the RAMCFG instance, indexed by \ref ramCfg_RegId_t */
static const uint32_t                   ramCfg_RegOffsetList[] =
{
    [ RAMCFG_REG_CR     ] = offsetof( RAMCFG_TypeDef, CR     ), /**< Control register                  */
    [ RAMCFG_REG_IER    ] = offsetof( RAMCFG_TypeDef, IER    ), /**< Interrupt enable register         */
    [ RAMCFG_REG_ISR    ] = offsetof( RAMCFG_TypeDef, ISR    ), /**< Interrupt status register         */
    [ RAMCFG_REG_SEAR   ] = offsetof( RAMCFG_TypeDef, SEAR   ), /**< ECC single error address register */
    [ RAMCFG_REG_DEAR   ] = offsetof( RAMCFG_TypeDef, DEAR   ), /**< ECC double error address register */
    [ RAMCFG_REG_ICR    ] = offsetof( RAMCFG_TypeDef, ICR    ), /**< Interrupt clear register          */
    [ RAMCFG_REG_WPR1   ] = offsetof( RAMCFG_TypeDef, WPR1   ), /**< Write protection register 1       */
#if ( RAMCFG_REG_WPR_CNT > 1u )
    [ RAMCFG_REG_WPR2   ] = offsetof( RAMCFG_TypeDef, WPR2   ), /**< Write protection register 2       */
#endif
#if ( RAMCFG_REG_WPR_CNT > 2u )
    [ RAMCFG_REG_WPR3   ] = offsetof( RAMCFG_TypeDef, WPR3   ), /**< Write protection register 3       */
#endif
#if ( RAMCFG_REG_WPR_CNT > 3u )
    [ RAMCFG_REG_WPR4   ] = offsetof( RAMCFG_TypeDef, WPR4   ), /**< Write protection register 4       */
#endif
    [ RAMCFG_REG_ECCKEY ] = offsetof( RAMCFG_TypeDef, ECCKEY ), /**< ECC key register                  */
    [ RAMCFG_REG_ERKEYR ] = offsetof( RAMCFG_TypeDef, ERKEYR ), /**< Erase key register                */
};

_Static_assert( (sizeof(ramCfg_RegOffsetList) / sizeof(uint32_t)) == RAMCFG_REG_CNT,
                "RamCfg_Reg: ramCfg_RegOffsetList has incorrect size." );

_Static_assert( ( RAMCFG_REG_WPR1 + RAMCFG_REG_WPR_CNT ) == RAMCFG_REG_ECCKEY,
                "RamCfg_Reg: write protection registers are not consecutive in ramCfg_RegId_t." );

_Static_assert( RAMCFG_REG_WP_PAGE_CNT <= ( RAMCFG_REG_WPR_CNT * RAMCFG_REG_PAGES_PER_WPR ),
                "RamCfg_Reg: pages of SRAM2 do not fit to the write protection registers." );

_Static_assert( RAMCFG_REG_WPR_BIT_CNT == ( CHAR_BIT * sizeof( uint32_t ) ),
                "RamCfg_Reg: a write protection register has one bit per page." );

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Sets the register bit with given mask.
 *
 * \param sramId  [in]: SRAM (RAMCFG instance)
 * \param regId   [in]: RAMCFG register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void RamCfg_Set_RegBit( ramCfg_SramId_t sramId, ramCfg_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t * const regAddr = RamCfg_Get_RegAddr( sramId, regId );

    if( RAMCFG_NULL_PTR != regAddr )
    {
        *regAddr |= bitMask;
    }
    else
    {
        /* Invalid SRAM or register ID, no action required */
    }
}


/**
 * \brief Resets the register bit with given mask.
 *
 * \param sramId  [in]: SRAM (RAMCFG instance)
 * \param regId   [in]: RAMCFG register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void RamCfg_Reset_RegBit( ramCfg_SramId_t sramId, ramCfg_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t * const regAddr = RamCfg_Get_RegAddr( sramId, regId );

    if( RAMCFG_NULL_PTR != regAddr )
    {
        *regAddr &= ~bitMask;
    }
    else
    {
        /* Invalid SRAM or register ID, no action required */
    }
}


/**
 * \brief Gets the register bit value with given mask.
 *
 * \param sramId  [in]: SRAM (RAMCFG instance)
 * \param regId   [in]: RAMCFG register ID
 * \param bitMask [in]: Mask to apply to the register
 * \return Register bit value with applied mask, no bit set for invalid SRAM or register ID
 */
uint32_t RamCfg_Get_RegBit( ramCfg_SramId_t sramId, ramCfg_RegId_t regId, uint32_t bitMask )
{
    uint32_t                      retValue = RAMCFG_REG_BITS_CLEAR;
    volatile uint32_t * const     regAddr  = RamCfg_Get_RegAddr( sramId, regId );

    if( RAMCFG_NULL_PTR != regAddr )
    {
        retValue = ( *regAddr & bitMask );
    }
    else
    {
        /* Invalid SRAM or register ID, no bit is set */
        retValue = RAMCFG_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with given mask and value.
 *
 * \param sramId   [in]: SRAM (RAMCFG instance)
 * \param regId    [in]: RAMCFG register ID
 * \param regMask  [in]: Mask to apply to the register
 * \param regValue [in]: Value to set in the register
 */
void RamCfg_Set_RegVal( ramCfg_SramId_t sramId, ramCfg_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    volatile uint32_t * const regAddr = RamCfg_Get_RegAddr( sramId, regId );

    if( RAMCFG_NULL_PTR != regAddr )
    {
        /* Set the register value with mask */
        *regAddr = ( ( *regAddr & ~regMask ) | ( regValue & regMask ) );
    }
    else
    {
        /* Invalid SRAM or register ID, no action required */
    }
}


/**
 * \brief Gets the register value with given mask.
 *
 * \param sramId  [in]: SRAM (RAMCFG instance)
 * \param regId   [in]: RAMCFG register ID.
 * \param regMask [in]: Mask to apply to the register
 * \return Register value with applied mask, no bit set for invalid SRAM or register ID
 */
uint32_t RamCfg_Get_RegVal( ramCfg_SramId_t sramId, ramCfg_RegId_t regId, uint32_t regMask )
{
    uint32_t                      retValue = RAMCFG_REG_BITS_CLEAR;
    volatile uint32_t * const     regAddr  = RamCfg_Get_RegAddr( sramId, regId );

    if( RAMCFG_NULL_PTR != regAddr )
    {
        /* Get the register value with mask */
        retValue = ( *regAddr & regMask );
    }
    else
    {
        /* Invalid SRAM or register ID, no bit is set */
        retValue = RAMCFG_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Writes the value to the register without reading it (write-only registers: keys, flags to clear).
 *
 * \param sramId   [in]: SRAM (RAMCFG instance)
 * \param regId    [in]: RAMCFG register ID
 * \param regValue [in]: Value written to the whole register
 */
void RamCfg_Set_RegRaw( ramCfg_SramId_t sramId, ramCfg_RegId_t regId, uint32_t regValue )
{
    volatile uint32_t * const regAddr = RamCfg_Get_RegAddr( sramId, regId );

    if( RAMCFG_NULL_PTR != regAddr )
    {
        *regAddr = regValue;
    }
    else
    {
        /* Invalid SRAM or register ID, no action required */
    }
}


/**
 * \brief Enables or disables the ECC of the SRAM and verifies it by read-back.
 *
 * The ECC is enabled by setting ECCE. It is disabled by the unlock sequence in the key register
 * (two keys, one after another) followed by clearing of ECCE. The content of the SRAM is not
 * touched: a memory which was written while the ECC was disabled has no valid code words.
 *
 * \param sramId   [in]: SRAM with ECC
 * \param eccState [in]: \ref RAMCFG_FUNCTION_ACTIVE enables the ECC, \ref RAMCFG_FUNCTION_INACTIVE disables it
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if ECCE read back as written within the timeout. Returns
 *         \ref RAMCFG_REQUEST_ERROR for an invalid SRAM or state (nothing is written) or when the
 *         read-back does not match (the bit is write protected, the key was not accepted).
 */
ramCfg_RequestState_t RamCfg_Reg_Set_Ecc( ramCfg_SramId_t sramId, ramCfg_FunctionState_t eccState )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_SRAM_CNT > sramId )
    {
        if( RAMCFG_FUNCTION_ACTIVE == eccState )
        {
            retState = RamCfg_Reg_Set_Verified( sramId, RAMCFG_REG_CR, RAMCFG_CR_ECCE, RAMCFG_CR_ECCE );
        }
        else if( RAMCFG_FUNCTION_INACTIVE == eccState )
        {
            RamCfg_Set_RegRaw( sramId, RAMCFG_REG_ECCKEY, RAMCFG_REG_ECC_KEY( RAMCFG_REG_ECC_KEY_1 ) );
            RamCfg_Set_RegRaw( sramId, RAMCFG_REG_ECCKEY, RAMCFG_REG_ECC_KEY( RAMCFG_REG_ECC_KEY_2 ) );

            retState = RamCfg_Reg_Set_Verified( sramId, RAMCFG_REG_CR, RAMCFG_CR_ECCE, RAMCFG_REG_BITS_CLEAR );
        }
        else
        {
            /* Invalid state, nothing is written */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid SRAM, nothing is written */
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the ECC enable bit of the SRAM.
 *
 * \param sramId [in]: SRAM with ECC
 *
 * \return \ref RAMCFG_FUNCTION_ACTIVE if the ECC is enabled, otherwise \ref RAMCFG_FUNCTION_INACTIVE
 *         (also for an invalid SRAM).
 */
ramCfg_FunctionState_t RamCfg_Reg_Get_Ecc( ramCfg_SramId_t sramId )
{
    return ( RamCfg_Reg_Get_BitState( sramId, RAMCFG_REG_CR, RAMCFG_CR_ECCE ) );
}


/**
 * \brief Selects the reporting of the errors (interrupt enables and address latch) and verifies it by read-back.
 *
 * The address latch is written first, so the address of an error is stored from the moment the interrupt is
 * enabled. The NMI redirection can be set only by software and is cleared only by a reset of the RAMCFG
 * peripheral: when it is set, any mode other than \ref RAMCFG_REPORT_NMI is refused and nothing is written.
 * In the NMI mode the single error is reported by the interrupt (SEIE) and the double error by the NMI, the
 * interrupt of the double error (DEIE) has no meaning there and is cleared.
 *
 * \param sramId     [in]: SRAM with ECC
 * \param reportMode [in]: Reporting mode
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the bits read back as written within the timeout. Returns
 *         \ref RAMCFG_REQUEST_ERROR for an invalid SRAM or mode, for a mode which cannot be set because
 *         the NMI redirection is set (nothing is written) and when the read-back does not match.
 */
ramCfg_RequestState_t RamCfg_Reg_Set_Reporting( ramCfg_SramId_t sramId, ramCfg_ReportMode_t reportMode )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;
    uint32_t              ierValue = RAMCFG_REG_BITS_CLEAR;
    uint32_t              aleValue = RAMCFG_REG_BITS_CLEAR;
    const uint32_t        nmiState = RamCfg_Get_RegBit( sramId, RAMCFG_REG_IER, RAMCFG_IER_ECCNMI );

    if( RAMCFG_SRAM_CNT > sramId )
    {
        retState = RAMCFG_REQUEST_OK;

        if( RAMCFG_REPORT_NONE == reportMode )
        {
            ierValue = RAMCFG_REG_BITS_CLEAR;
            aleValue = RAMCFG_REG_BITS_CLEAR;
        }
        else if( RAMCFG_REPORT_IRQ == reportMode )
        {
            ierValue = RAMCFG_REG_IER_IRQ;
            aleValue = RAMCFG_CR_ALE;
        }
        else if( RAMCFG_REPORT_NMI == reportMode )
        {
            ierValue = RAMCFG_REG_IER_NMI;
            aleValue = RAMCFG_CR_ALE;
        }
        else
        {
            /* Invalid mode, nothing is written */
            retState = RAMCFG_REQUEST_ERROR;
        }

        if( ( RAMCFG_REQUEST_OK        == retState  ) &&
            ( RAMCFG_REG_BITS_CLEAR    != nmiState  ) &&
            ( RAMCFG_REG_IER_NMI       != ierValue  )    )
        {
            /* The NMI redirection cannot be cleared by software, nothing is written */
            retState = RAMCFG_REQUEST_ERROR;
        }
        else
        {
            /* The mode can be written or an earlier check failed */
        }

        if( RAMCFG_REQUEST_OK == retState )
        {
            retState = RamCfg_Reg_Set_Verified( sramId, RAMCFG_REG_CR, RAMCFG_CR_ALE, aleValue );
        }
        else
        {
            /* Previous step failed */
        }

        if( RAMCFG_REQUEST_OK == retState )
        {
            retState = RamCfg_Reg_Set_Verified( sramId, RAMCFG_REG_IER, RAMCFG_REG_IER_MASK, ierValue );
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        /* Invalid SRAM, nothing is written */
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables the interrupts of the errors and the address latch (the NMI redirection is not touched).
 *
 * \param sramId [in]: SRAM with ECC
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the bits read back as cleared within the timeout. Returns
 *         \ref RAMCFG_REQUEST_ERROR for an invalid SRAM or when the read-back does not match.
 */
ramCfg_RequestState_t RamCfg_Reg_Set_ReportingOff( ramCfg_SramId_t sramId )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    retState = RamCfg_Reg_Set_Verified( sramId, RAMCFG_REG_IER, RAMCFG_REG_IER_IRQ, RAMCFG_REG_BITS_CLEAR );

    if( RAMCFG_REQUEST_OK == retState )
    {
        retState = RamCfg_Reg_Set_Verified( sramId, RAMCFG_REG_CR, RAMCFG_CR_ALE, RAMCFG_REG_BITS_CLEAR );
    }
    else
    {
        /* Previous step failed */
    }

    return ( retState );
}


/**
 * \brief Reads the reporting mode of the errors from the interrupt enable bits.
 *
 * The mode is \ref RAMCFG_REPORT_NONE without any enable bit, \ref RAMCFG_REPORT_IRQ with SEIE and DEIE,
 * \ref RAMCFG_REPORT_NMI with SEIE and the NMI redirection. Any other combination is not a mode of the module.
 *
 * \param sramId     [in]: SRAM with ECC
 * \param reportMode [out]: Pointer to store the mode. Must not be NULL, it is not changed on an error.
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the bits are a mode of the module. Returns \ref RAMCFG_REQUEST_ERROR
 *         for an invalid SRAM or pointer and for another combination of the bits.
 */
ramCfg_RequestState_t RamCfg_Reg_Get_Reporting( ramCfg_SramId_t sramId, ramCfg_ReportMode_t * const reportMode )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( ( RAMCFG_NULL_PTR != reportMode ) &&
        ( RAMCFG_SRAM_CNT  > sramId     )    )
    {
        const uint32_t ierValue = RamCfg_Get_RegVal( sramId, RAMCFG_REG_IER, RAMCFG_REG_IER_MASK );

        if( RAMCFG_REG_BITS_CLEAR == ierValue )
        {
            *reportMode = RAMCFG_REPORT_NONE;
            retState    = RAMCFG_REQUEST_OK;
        }
        else if( RAMCFG_REG_IER_IRQ == ierValue )
        {
            *reportMode = RAMCFG_REPORT_IRQ;
            retState    = RAMCFG_REQUEST_OK;
        }
        else if( RAMCFG_IER_ECCNMI == ( ierValue & RAMCFG_IER_ECCNMI ) )
        {
            /* The interrupt of the double error has no meaning with the NMI redirection */
            if( RAMCFG_IER_SEIE == ( ierValue & RAMCFG_IER_SEIE ) )
            {
                *reportMode = RAMCFG_REPORT_NMI;
                retState    = RAMCFG_REQUEST_OK;
            }
            else
            {
                /* NMI redirection without the interrupt of the single error is not a mode of the module */
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Another combination of the enable bits is not a mode of the module */
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
 * \brief Sets the write protection of a range of pages and verifies it by read-back.
 *
 * The protection bits can be set only by software and are cleared only by a reset of the RAMCFG
 * peripheral. The pages are protected one register after another (lowest first), the pages which are already
 * protected stay protected. A mismatch of the read-back leaves the pages of the registers written before
 * protected (the protection cannot be undone).
 *
 * \param sramId    [in]: SRAM with the write protection (SRAM2)
 * \param firstPage [in]: First page of the range
 * \param pageCnt   [in]: Count of pages of the range, at least one
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the protection bits of all pages of the range read back set within
 *         the timeout. Returns \ref RAMCFG_REQUEST_ERROR for an invalid SRAM, for an empty range or a range beyond
 *         the pages of SRAM2 (nothing is written) and when the read-back does not match.
 */
ramCfg_RequestState_t RamCfg_Reg_Set_WriteProtection( ramCfg_SramId_t sramId,
                                                      ramCfg_Page_t firstPage,
                                                      ramCfg_PageCnt_t pageCnt )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( ( RAMCFG_SRAM_CNT                  >  sramId                               ) &&
        ( RAMCFG_PAGE_CNT_UNUSED           <  pageCnt                              ) &&
        ( RAMCFG_REG_WP_PAGE_CNT           >= ( (uint32_t)firstPage + pageCnt )    )    )
    {
        const uint32_t firstWpr = ( (uint32_t)firstPage / RAMCFG_REG_PAGES_PER_WPR );
        const uint32_t lastWpr  = ( ( (uint32_t)firstPage + pageCnt - 1u ) / RAMCFG_REG_PAGES_PER_WPR );

        retState = RAMCFG_REQUEST_OK;

        for( uint32_t wprIdx = firstWpr;
             ( lastWpr             >= wprIdx ) &&
             ( RAMCFG_REQUEST_OK   == retState );
             wprIdx ++ )
        {
            const uint32_t     pageMask = RamCfg_Reg_Calc_PageMask( wprIdx, firstPage, pageCnt );
            const ramCfg_RegId_t regId  = (ramCfg_RegId_t)( (uint32_t)RAMCFG_REG_WPR1 + wprIdx );

            retState = RamCfg_Reg_Set_Verified( sramId, regId, pageMask, pageMask );
        }
    }
    else
    {
        /* Invalid SRAM or page range, nothing is written */
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the write protection of a range of pages.
 *
 * \param sramId     [in]: SRAM with the write protection (SRAM2)
 * \param firstPage  [in]: First page of the range
 * \param pageCnt    [in]: Count of pages of the range, at least one
 * \param protection [out]: Pointer to store the state, \ref RAMCFG_FUNCTION_ACTIVE if every page of the range is
 *                          protected, otherwise \ref RAMCFG_FUNCTION_INACTIVE. Must not be NULL, it is not changed
 *                          on an error.
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the state was read. Returns \ref RAMCFG_REQUEST_ERROR for an invalid
 *         SRAM, pointer or page range.
 */
ramCfg_RequestState_t RamCfg_Reg_Get_WriteProtection( ramCfg_SramId_t sramId,
                                                      ramCfg_Page_t firstPage,
                                                      ramCfg_PageCnt_t pageCnt,
                                                      ramCfg_FunctionState_t * const protection )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( ( RAMCFG_NULL_PTR         != protection                           ) &&
        ( RAMCFG_SRAM_CNT         >  sramId                               ) &&
        ( RAMCFG_PAGE_CNT_UNUSED  <  pageCnt                              ) &&
        ( RAMCFG_REG_WP_PAGE_CNT  >= ( (uint32_t)firstPage + pageCnt )    )    )
    {
        const uint32_t firstWpr = ( (uint32_t)firstPage / RAMCFG_REG_PAGES_PER_WPR );
        const uint32_t lastWpr  = ( ( (uint32_t)firstPage + pageCnt - 1u ) / RAMCFG_REG_PAGES_PER_WPR );
        ramCfg_FunctionState_t state = RAMCFG_FUNCTION_ACTIVE;

        for( uint32_t wprIdx = firstWpr; lastWpr >= wprIdx; wprIdx ++ )
        {
            const uint32_t       pageMask = RamCfg_Reg_Calc_PageMask( wprIdx, firstPage, pageCnt );
            const ramCfg_RegId_t regId    = (ramCfg_RegId_t)( (uint32_t)RAMCFG_REG_WPR1 + wprIdx );
            const uint32_t       setBits  = RamCfg_Get_RegVal( sramId, regId, pageMask );

            if( pageMask != setBits )
            {
                state = RAMCFG_FUNCTION_INACTIVE;
            }
            else
            {
                /* All pages of the register are protected */
            }
        }

        *protection = state;
        retState    = RAMCFG_REQUEST_OK;
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the software erase of the SRAM (unlock sequence in the erase key register, then SRAMER).
 *
 * The start cannot be verified by a read-back: the keys cannot be read and the erase of a small SRAM can be
 * finished before the first read. The caller waits for the end by \ref RamCfg_Reg_Wait_EraseEnd or polls
 * \ref RamCfg_Reg_Get_EraseBusy. The content of the SRAM is zero after the erase.
 *
 * \param sramId [in]: SRAM to erase
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the sequence was written. Returns \ref RAMCFG_REQUEST_ERROR for an
 *         invalid SRAM (nothing is written).
 */
ramCfg_RequestState_t RamCfg_Reg_Set_EraseStart( ramCfg_SramId_t sramId )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_SRAM_CNT > sramId )
    {
        RamCfg_Set_RegRaw( sramId, RAMCFG_REG_ERKEYR, RAMCFG_REG_ERASE_KEY( RAMCFG_REG_ERASE_KEY_1 ) );
        RamCfg_Set_RegRaw( sramId, RAMCFG_REG_ERKEYR, RAMCFG_REG_ERASE_KEY( RAMCFG_REG_ERASE_KEY_2 ) );
        RamCfg_Set_RegBit( sramId, RAMCFG_REG_CR, RAMCFG_CR_SRAMER );

        retState = RAMCFG_REQUEST_OK;
    }
    else
    {
        /* Invalid SRAM, nothing is written */
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns whether an erase of the SRAM is running (bit SRAMBUSY or the erase bit SRAMER).
 *
 * \param sramId [in]: SRAM
 *
 * \return \ref RAMCFG_FLAG_ACTIVE while the erase runs (started by the software, a reset, a tamper event
 *         or a regression of the product state), otherwise \ref RAMCFG_FLAG_INACTIVE.
 */
ramCfg_FlagState_t RamCfg_Reg_Get_EraseBusy( ramCfg_SramId_t sramId )
{
    ramCfg_FlagState_t       retState = RAMCFG_FLAG_INACTIVE;
    const ramCfg_FlagState_t busyFlag = RamCfg_Reg_Get_FlagState( sramId, RAMCFG_REG_ISR, RAMCFG_ISR_SRAMBUSY );
    const ramCfg_FlagState_t eraseBit = RamCfg_Reg_Get_FlagState( sramId, RAMCFG_REG_CR, RAMCFG_CR_SRAMER );

    if( ( RAMCFG_FLAG_ACTIVE == busyFlag ) ||
        ( RAMCFG_FLAG_ACTIVE == eraseBit )    )
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
 * \brief Waits until the erase of the SRAM is finished.
 *
 * \param sramId [in]: SRAM
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if no erase is running (also if none was started). Returns
 *         \ref RAMCFG_REQUEST_ERROR when the erase is still running after the timeout (the erase is
 *         not stopped) and for an invalid SRAM.
 */
ramCfg_RequestState_t RamCfg_Reg_Wait_EraseEnd( ramCfg_SramId_t sramId )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( RAMCFG_SRAM_CNT > sramId )
    {
        for( uint32_t iterationCnt = 0u; RAMCFG_REG_ERASE_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const ramCfg_FlagState_t busyFlag = RamCfg_Reg_Get_EraseBusy( sramId );

            if( RAMCFG_FLAG_INACTIVE == busyFlag )
            {
                retState = RAMCFG_REQUEST_OK;
                break;
            }
            else
            {
                /* Erase is still running */
                retState = RAMCFG_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the flag of the corrected single error (bit SEDC).
 *
 * \param sramId [in]: SRAM with ECC
 *
 * \return \ref RAMCFG_FLAG_ACTIVE if a single error was detected and corrected and the flag was not cleared,
 *         otherwise \ref RAMCFG_FLAG_INACTIVE.
 */
ramCfg_FlagState_t RamCfg_Reg_Get_SingleFlag( ramCfg_SramId_t sramId )
{
    return ( RamCfg_Reg_Get_FlagState( sramId, RAMCFG_REG_ISR, RAMCFG_ISR_SEDC ) );
}


/**
 * \brief Returns the flag of the double error (bit DED).
 *
 * \param sramId [in]: SRAM with ECC
 *
 * \return \ref RAMCFG_FLAG_ACTIVE if a double error was detected and the flag was not cleared,
 *         otherwise \ref RAMCFG_FLAG_INACTIVE.
 */
ramCfg_FlagState_t RamCfg_Reg_Get_DoubleFlag( ramCfg_SramId_t sramId )
{
    return ( RamCfg_Reg_Get_FlagState( sramId, RAMCFG_REG_ISR, RAMCFG_ISR_DED ) );
}


/**
 * \brief Clears the flag of the single error by writing 1 to the clear bit CSEDC.
 *
 * \note  The clear register is written as a whole, not by a read-modify-write (the bits read as the flags and
 *        a read-modify-write would clear the flag of the double error as well).
 *
 * \param sramId [in]: SRAM with ECC
 */
void RamCfg_Reg_Clear_SingleFlag( ramCfg_SramId_t sramId )
{
    RamCfg_Set_RegRaw( sramId, RAMCFG_REG_ICR, RAMCFG_ICR_CSEDC );
}


/**
 * \brief Clears the flag of the double error by writing 1 to the clear bit CDED.
 *
 * \note  The clear register is written as a whole, not by a read-modify-write (the bits read as the flags and
 *        a read-modify-write would clear the flag of the single error as well).
 *
 * \param sramId [in]: SRAM with ECC
 */
void RamCfg_Reg_Clear_DoubleFlag( ramCfg_SramId_t sramId )
{
    RamCfg_Set_RegRaw( sramId, RAMCFG_REG_ICR, RAMCFG_ICR_CDED );
}


/**
 * \brief Returns the address of the last single error stored by the hardware.
 *
 * \param sramId [in]: SRAM with ECC
 *
 * \return Content of the register (the address is stored only while the address latch and the interrupt of the
 *         single error are enabled), zero for an invalid SRAM.
 */
ramCfg_Address_t RamCfg_Reg_Get_SingleAddress( ramCfg_SramId_t sramId )
{
    return ( RamCfg_Get_RegVal( sramId, RAMCFG_REG_SEAR, RAMCFG_SEAR_ESEA ) );
}


/**
 * \brief Returns the address of the last double error stored by the hardware.
 *
 * \param sramId [in]: SRAM with ECC
 *
 * \return Content of the register (the address is stored only while the address latch and the interrupt or the NMI
 *         of the double error are enabled), zero for an invalid SRAM.
 */
ramCfg_Address_t RamCfg_Reg_Get_DoubleAddress( ramCfg_SramId_t sramId )
{
    return ( RamCfg_Get_RegVal( sramId, RAMCFG_REG_DEAR, RAMCFG_DEAR_EDEA ) );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Returns the address of a register of an SRAM.
 *
 * \param sramId [in]: SRAM (RAMCFG instance)
 * \param regId  [in]: RAMCFG register ID
 *
 * \return Address of the register, \ref RAMCFG_NULL_PTR for an invalid SRAM or register ID.
 */
static volatile uint32_t * RamCfg_Get_RegAddr( ramCfg_SramId_t sramId, ramCfg_RegId_t regId )
{
    volatile uint32_t * retAddr = RAMCFG_NULL_PTR;

    if( ( RAMCFG_SRAM_CNT > sramId ) &&
        ( RAMCFG_REG_CNT  > regId  )    )
    {
        retAddr = (volatile uint32_t *)( (uintptr_t)ramCfg_InstList[ sramId ].Inst + ramCfg_RegOffsetList[ regId ] );
    }
    else
    {
        retAddr = RAMCFG_NULL_PTR;
    }

    return ( retAddr );
}


/**
 * \brief Waits until the register (bit field) reads back the required value.
 *
 * \param sramId   [in]: SRAM (RAMCFG instance)
 * \param regId    [in]: RAMCFG register ID
 * \param regMask  [in]: Mask of the checked bits
 * \param regValue [in]: Required value of the masked bits
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the value was read back within the
 *         timeout. Otherwise returns \ref RAMCFG_REQUEST_ERROR.
 */
static ramCfg_RequestState_t RamCfg_Reg_Wait_RegVal( ramCfg_SramId_t sramId,
                                                     ramCfg_RegId_t regId,
                                                     uint32_t regMask,
                                                     uint32_t regValue )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    for( uint32_t iterationCnt = 0u; RAMCFG_REG_READBACK_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t readValue = RamCfg_Get_RegVal( sramId, regId, regMask );

        if( ( regValue & regMask ) == readValue )
        {
            retState = RAMCFG_REQUEST_OK;
            break;
        }
        else
        {
            /* Register has not been updated yet */
            retState = RAMCFG_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Writes the bit field of the register and verifies it by read-back.
 *
 * \param sramId   [in]: SRAM (RAMCFG instance)
 * \param regId    [in]: RAMCFG register ID
 * \param regMask  [in]: Mask of the written bits
 * \param regValue [in]: Value of the masked bits
 *
 * \return Returns \ref RAMCFG_REQUEST_OK if the value read back within the timeout. Returns
 *         \ref RAMCFG_REQUEST_ERROR for an invalid SRAM or register (nothing is written) and when the
 *         read-back does not match.
 */
static ramCfg_RequestState_t RamCfg_Reg_Set_Verified( ramCfg_SramId_t sramId,
                                                      ramCfg_RegId_t regId,
                                                      uint32_t regMask,
                                                      uint32_t regValue )
{
    ramCfg_RequestState_t retState = RAMCFG_REQUEST_ERROR;

    if( ( RAMCFG_SRAM_CNT > sramId ) &&
        ( RAMCFG_REG_CNT  > regId  )    )
    {
        RamCfg_Set_RegVal( sramId, regId, regMask, regValue );

        retState = RamCfg_Reg_Wait_RegVal( sramId, regId, regMask, regValue );
    }
    else
    {
        /* Invalid SRAM or register, nothing is written */
        retState = RAMCFG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the state of a register bit field as module function state.
 *
 * \param sramId  [in]: SRAM (RAMCFG instance)
 * \param regId   [in]: RAMCFG register ID
 * \param bitMask [in]: Mask of the checked bits
 *
 * \return \ref RAMCFG_FUNCTION_ACTIVE if any bit of the mask is set, otherwise
 *         \ref RAMCFG_FUNCTION_INACTIVE (also for an invalid SRAM or register ID).
 */
static ramCfg_FunctionState_t RamCfg_Reg_Get_BitState( ramCfg_SramId_t sramId, ramCfg_RegId_t regId, uint32_t bitMask )
{
    ramCfg_FunctionState_t retState = RAMCFG_FUNCTION_INACTIVE;
    const uint32_t         bitValue = RamCfg_Get_RegBit( sramId, regId, bitMask );

    if( RAMCFG_REG_BITS_CLEAR != bitValue )
    {
        retState = RAMCFG_FUNCTION_ACTIVE;
    }
    else
    {
        retState = RAMCFG_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Returns the state of a register bit field as module flag state.
 *
 * \param sramId  [in]: SRAM (RAMCFG instance)
 * \param regId   [in]: RAMCFG register ID
 * \param bitMask [in]: Mask of the checked bits
 *
 * \return \ref RAMCFG_FLAG_ACTIVE if any bit of the mask is set, otherwise
 *         \ref RAMCFG_FLAG_INACTIVE (also for an invalid SRAM or register ID).
 */
static ramCfg_FlagState_t RamCfg_Reg_Get_FlagState( ramCfg_SramId_t sramId, ramCfg_RegId_t regId, uint32_t bitMask )
{
    ramCfg_FlagState_t retState = RAMCFG_FLAG_INACTIVE;
    const uint32_t     bitValue = RamCfg_Get_RegBit( sramId, regId, bitMask );

    if( RAMCFG_REG_BITS_CLEAR != bitValue )
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
 * \brief Calculates the bits of a write protection register which belong to a range of pages.
 *
 * Bit n of the register wprIdx belongs to the page wprIdx * 32 + n.
 *
 * \param wprIdx    [in]: Index of the write protection register (0 - RAMCFG_WPR1)
 * \param firstPage [in]: First page of the range
 * \param pageCnt   [in]: Count of pages of the range
 *
 * \return Mask of the bits of the pages which are in the range and in the register.
 */
static uint32_t RamCfg_Reg_Calc_PageMask( uint32_t wprIdx, ramCfg_Page_t firstPage, ramCfg_PageCnt_t pageCnt )
{
    uint32_t       pageMask = RAMCFG_REG_BITS_CLEAR;
    const uint32_t endPage  = ( (uint32_t)firstPage + pageCnt );

    for( uint32_t bitIdx = 0u; RAMCFG_REG_WPR_BIT_CNT > bitIdx; bitIdx ++ )
    {
        const uint32_t pageIdx = ( ( wprIdx * RAMCFG_REG_PAGES_PER_WPR ) + bitIdx );

        if( ( (uint32_t)firstPage <= pageIdx ) &&
            ( endPage             >  pageIdx )    )
        {
            pageMask |= ( RAMCFG_REG_BIT_ONE << bitIdx );
        }
        else
        {
            /* The page is not in the range */
        }
    }

    return ( pageMask );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
