# RAMCFG Peripheral Driver

This module provides an abstraction layer for the **configuration of the internal SRAMs** (`RAMCFG`) of STM32 MCUs: the error
correcting code (ECC) or the parity check with the error reporting, the **write protection** of the memory in pages, the **software
erase** of an SRAM and the wait states of the memory. It reports the single and the double errors, the failing address and
keeps the statistics. It is a low-level MCAL module: it owns the SRAM controller hardware, the application and the other
modules never touch the registers.

The public interface (`RamCfg_Port.h`, `RamCfg_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: STM32H5 implemented (version 0.1.0).** The interface below is implemented for STM32H5 in `RamCfg_Port.h` /
> `RamCfg_Types.h` (branch `Dev/STM32H5`), the registers are accessed only by the private `RamCfg_Reg.c/h`. Verified: unit tests
> (Unity + CMock + RegMem, 132 test cases) on one preset of each of the 12 STM32H5 lines (H503, H523, H533, H543, H553, H562, H563,
> H573, H5E4, H5E5, H5F4, H5F5) and a compilation of the module for all of them. The integration tests (`Tests/IntegrationTests`,
> 17 test cases) are only compiled (`STM32H503xB_IntegrationTest`, `STM32H563xI_IntegrationTest`), they have not been run on a board
> yet; two of them end with `TEST_IGNORE_MESSAGE` because they need the Pwr module (not available yet). Other families: stories and
> branches follow.
>
> Notes of the STM32H5 implementation (the interface above is refined for the hardware, the points below are the STM32H5 specifics and
> the deviations from the general text of this README):
> - **SRAMs of the line.** `ramCfg_SramId_t` contains exactly the memories of the device line: STM32H503 has SRAM1 (16 KB), SRAM2
>   (16 KB) and the backup SRAM (2 KB); STM32H523 and STM32H533 have SRAM1 of 128 KB, SRAM2 of 80 KB, SRAM3 of 64 KB and the backup SRAM
>   of 2 KB; STM32H543 and STM32H553 the same with SRAM3 of 96 KB; STM32H562, STM32H563 and STM32H573 have SRAM1 of 256 KB, SRAM2 of
>   64 KB, SRAM3 of 320 KB and the backup SRAM of 4 KB; STM32H5E4, STM32H5E5, STM32H5F4 and STM32H5F5 have SRAM1 of 256 KB, SRAM2 of
>   128 KB, SRAM3, SRAM4 and SRAM5 of 384 KB each and the backup SRAM of 4 KB. The items `RAMCFG_SRAM_3`, `RAMCFG_SRAM_4` and
>   `RAMCFG_SRAM_5` exist where the CMSIS device header defines the `RAMCFG` instance. The base address and the size of a memory come
>   from the device header, the SRAM identification is the index of the configuration, the statistics and the runtime data.
> - **ECC.** The ECC (single error correction, double error detection) has SRAM2, SRAM3 (not on STM32H523 and STM32H533, where the
>   option bit `SRAM3_ECC` does not exist) and the backup SRAM. SRAM1, SRAM4 and SRAM5 have no ECC: the module refuses the ECC, the
>   error reporting and the scrubbing of them (`RAMCFG_REQUEST_ERROR`, no register is written) and accepts `RAMCFG_CHECK_NONE` and
>   `RAMCFG_REPORT_NONE`. The reference manual of STM32H503 (RM0492) names SRAM1 as a memory with the ECC, but the registers of SRAM1
>   have no ECC bits (`ECCE`, `ALE`, `SEDC`, `DED` are reserved, `IER`, `SEAR`, `DEAR`, `ICR`, `ECCKEYR` exist for the memories
>   2, 3 and 5 only): SRAM1 of STM32H503 is handled as a memory without the `RAMCFG` control of the ECC.
> - **Check mode.** `RAMCFG_CHECK_UNCHANGED` (value 0, the default of the configuration) is an item added to the list of this README: the
>   ECC state set by the option bytes is kept and no register is written. `RAMCFG_CHECK_PARITY` stays in the list for the common
>   interface and is refused (the SRAMs of STM32H5 have the ECC only). `RAMCFG_CHECK_ECC` sets `RAMCFG_CR.ECCE`, `RAMCFG_CHECK_NONE`
>   clears it after the unlock sequence in `RAMCFG_ECCKEYR` (0xAE, 0x75, a wrong key activates the protection again); both are verified
>   by a read-back. The module does **not** initialize the memory when it enables the ECC (the erase would destroy the data): a
>   memory which was written with the ECC disabled has no valid code words, the application erases it (`RamCfg_Set_Erase`) before the
>   first enabling.
> - **Error reporting.** `ramCfg_ReportMode_t` has `RAMCFG_REPORT_NONE`, `RAMCFG_REPORT_IRQ` (single and double error by the `RAMCFG`
>   interrupt) and `RAMCFG_REPORT_NMI` (single error by the interrupt, double error by the NMI). The default mode of the configuration is
>   `RAMCFG_REPORT_IRQ` for the SRAMs with ECC and `RAMCFG_REPORT_NONE` for the others, so that the callbacks of the usage example below
>   work without a further setting. The hardware stores the failing address only while the address latch (`RAMCFG_CR.ALE`) and the
>   interrupt of the error are enabled (device errata sheet of STM32H503, ES0561 2.2.20, a documentation erratum): the module sets the
>   latch first and the interrupt enables after it, so the statistics need a reporting mode other than none. The NMI mode takes over
>   the NMI vector through the Nvic module before the redirection `RAMCFG_IER.ECCNMI` is set (the previous handler is saved and restored
>   by `RamCfg_Deinit`, the module does not call it); the redirection is set by the software only and is cleared only by a reset of the
>   peripheral, so an SRAM which is in the NMI mode refuses every other mode. The integration test framework uses the NMI as its fault
>   handler: the integration tests never use the NMI mode.
> - **Errors.** The interrupt handler counts the error (the counters wrap around at 2^32), stores the address (`RAMCFG_SEAR` or `RAMCFG_DEAR`
>   as the hardware reports it), scrubs the word when the scrubbing is active, acknowledges the flag and calls the callback (in this
>   order, the flag is already cleared in the callback). The flags are cleared by writing 1 to `RAMCFG_ICR`, which reads as the flags:
>   the register is always written as a whole and never by a read-modify-write (it would clear the other flag too). The scrubbing reads
>   the failing word (the hardware returns the corrected value) and writes it back; the address of the hardware is accepted as an address
>   of the memory map (either alias) or as an offset in the SRAM, an address outside of the SRAM is not scrubbed. The format of the
>   address register for the scrubbing has not been verified on a board. The statistics are kept per SRAM with ECC, the errors of an SRAM
>   which is not reported are not recorded.
> - **Write protection.** Only SRAM2 has it, in pages of 1 KB: 16 pages on STM32H503, 80 on STM32H523 ... STM32H553, 64 on STM32H562,
>   STM32H563 and STM32H573, 128 on STM32H5E4 ... STM32H5F5 (one 32-bit register per 32 pages, `RAMCFG_WPR1` ... `RAMCFG_WPR4`). The
>   protection bits are set by the software only and cleared only by a reset of the peripheral: `RamCfg_Set_WriteProtection` adds pages,
>   a count of pages 0 (which would remove it) is refused like a range beyond the pages, a range which holds the main or the process
>   stack of the caller is refused (a write to a protected page would fault). `RamCfg_Get_WriteProtection` is added to the interface: it
>   returns `RAMCFG_FUNCTION_ACTIVE` when **every** page of the range is protected. The page masks are built from the page index (page n is
>   the bit n modulo 32 of the register n / 32): the CMSIS device headers define the bits `P56WP` ... `P62WP` of `RAMCFG_WPR2` and
>   `P88WP` ... `P94WP` of `RAMCFG_WPR3` one position too high (and STM32H503 defines an extra `P16WP`), and the offset of
>   `RAMCFG_WPR3` is 0x20 on STM32H523 ... STM32H553 but 0x30 (with `RAMCFG_WPR4` at 0x34) on STM32H5E4 ... STM32H5F5.
> - **Erase.** `RamCfg_Set_Erase` activates the clock of the memory through the Rcc module (SRAM1, SRAM2, SRAM3, backup SRAM; SRAM4 and
>   SRAM5 have the enable bits `SRAM4EN`, `SRAM5EN` in `RCC_AHB2ENR`, but the Rcc module has no clock identification for them yet and
>   the module does not enable them), writes the unlock keys 0xCA and 0x53 to `RAMCFG_ERKEYR` and sets `RAMCFG_CR.SRAMER`. With an erase
>   complete callback in the configuration the function returns at once and `RamCfg_Task` calls the callback once at the end; without
>   it the function waits for the end (a bounded loop on `SRAMBUSY` and `SRAMER`) and returns `RAMCFG_REQUEST_ERROR` on a timeout (the
>   erase keeps running and `RamCfg_Get_EraseState` reports `RAMCFG_ERASE_BUSY`). The start cannot be verified by a read-back (the keys
>   cannot be read, the erase of a small SRAM may be finished before the first read). The erase is refused when the main or the
>   process stack pointer (the secure alias of the address is folded to the non-secure one) lies in the SRAM and while an erase of the
>   SRAM runs; an erase started by the hardware (reset, tamper event, regression of the product state) is reported as busy. The erase
>   destroys the data of the SRAM, including the runtime data of the module and of the application if they are in it (only the stack of
>   the caller is checked). The backup SRAM clock bit `BKPRAMEN` has the reset value disabled in RM0481 and enabled in RM0517, so the
>   module activates it before an erase; the access of the CPU to the backup SRAM also needs the write access to the backup domain
>   (`PWR_DBPCR.DBP`), which belongs to the Pwr module (not implemented yet): this module does not touch the registers of the Pwr module, so the integration test cases which need that write access (the pattern check of the erase of the backup SRAM) end with `TEST_IGNORE_MESSAGE` until the Pwr module is available. SRAM2 is erased when the backup domain is reset (device errata
>   sheets ES0565 2.2.14 and ES0621 2.2.6), the option bits `SRAMx_RST` erase immediately and not through `RAMCFG` (ES0565 2.2.13, ES0621 2.2.4).
> - **Init and Deinit.** `RamCfg_Init` checks the whole configuration before any register is written (an invalid request returns an error
>   and changes nothing), deactivates the interrupt, activates the clock of the controller, applies check mode, reporting, scrubbing and
>   write protection of every SRAM with a read-back of every write, then sets the NMI vector and the interrupt (handler, priority,
>   activation) according to the reporting modes. A failure after the first change deinitializes the module and returns an error.
>   `RamCfg_Deinit` disables the error interrupts and the address latch, deactivates the interrupt, restores the NMI vector, removes
>   the callbacks and clears the statistics; what the hardware cannot undo stays (the ECC state, the write protection, the NMI
>   redirection), the clock of the controller is not disabled. The write protection and the NMI redirection are therefore additive over
>   repeated initializations.
> - **Getters.** The getters have no side effects (they never enable a clock or touch a register) and return `RAMCFG_REQUEST_ERROR` before
>   `RamCfg_Init`; the outputs are not changed on an error. The statistics getters read the data of the module, not the registers.
> - **Wait states.** The SRAMs of STM32H5 have no setting of the wait states: `RamCfg_Set_WaitStates` and `RamCfg_Get_WaitStates` keep their
>   signature and always return `RAMCFG_REQUEST_ERROR`.
> - **Interface details derived from the tables of this README.** The code block of the short interface lists only part of the functions,
>   the interface is the union of the table and the block, with `RamCfg_Get_CheckMode`, `RamCfg_Set_ReportMode`, `RamCfg_Get_ReportMode`
>   (an SRAM without ECC reports `RAMCFG_REPORT_NONE`), `RamCfg_Set_ErrorStatisticsClear`, `RamCfg_Get_WriteProtection`,
>   `RamCfg_Set_WaitStates` / `Get` and `RamCfg_Set_IrqPriority` / `Get` added. `RamCfg_Set_Scrubbing` is a setting of the module (no
>   register is written) and has no getter. The configuration members are `CheckMode`, `ReportMode`, `Scrubbing`,
>   `ProtectionFirstPage`, `ProtectionPageCnt` per SRAM, the three callbacks and `IrqPriority`. The erase state is `RAMCFG_ERASE_IDLE` until an
>   erase was requested, `RAMCFG_ERASE_BUSY` while the hardware reports a running erase and `RAMCFG_ERASE_DONE` afterwards.
> - **Reference manual inconsistencies** (handled as described): RM0517 lists `RAMCFG_MxSEAR` for the memories 2 and 3 only, but `DEAR`,
>   `ICR`, `IER` and `ECCKEYR` for 2, 3 and 5 (backup SRAM): taken as a typo, the backup SRAM single error address is used. The register
>   lists of RM0492 mention the memory 3, which does not exist on STM32H503. Table 43 of RM0481 (the erase conditions of the memories)
>   is garbled in the PDF; the software erase is assumed for every SRAM including the backup SRAM, as the description of
>   `RAMCFG_ERKEYR` (x = 1 to 5) says. Only the first 256 KB (STM32H56x, STM32H573) or 64 KB (STM32H543, STM32H553) of SRAM3 are covered by
>   the ECC according to RM0481, the rest is not covered or holds the ECC bits (not modelled by the module).
> - **Not verified on a board.** The behavior of the scrubbing (format of the failing address), the erase with write-protected pages, the NMI
>   path and the ECC initialization procedure are covered by the unit tests with the hardware model of the registers only (or not at
>   all); the erase of the backup SRAM with its clock is exercised by the integration tests, which have not been run. The pattern check of
>   the erase of the backup SRAM (CPU write access, `PWR_DBPCR.DBP` of the Pwr module) is not implemented: the two integration test cases
>   concerned end with `TEST_IGNORE_MESSAGE` until the Pwr module is available.

---

## Intent and purpose

- A memory that holds the stack, the heap and the data of a safety or a security relevant application can be flipped by radiation, by a
  glitch or by an attack. The ECC **corrects a single-bit error and detects a double-bit error**; the application wants to know about both: the
  single error is a warning (and the corrected value is written back), the double error is a fault that needs a reaction (a reset, a safe
  state). The module configures the checking, collects the events (the count, the address, the SRAM) and calls the callback.
- **Erasing the SRAM by the software** is needed before the memory is handed to a less trusted code, after a key was held in it, or
  after a tamper event; the module erases a whole SRAM (or the part the hardware allows), waits for the end and reports it.
- **Write protection** in the granularity of a page (1 KB in the newest families) protects a code or a table in the SRAM against an
  accidental overwrite; a write to a protected page is an error that the module reports.
- The module hides the differences between the families: the names and the number of the SRAMs, the ECC / parity availability, the
  protection granularity, the erase and the interrupt / NMI routing differ, the interface stays the same.

What the module is **not**: it does not decide what the application does after an uncorrectable error (it calls the
callback), it does not configure the memory protection unit (the attributes of the regions) and it does not handle the erase
of the SRAMs at the reset (an option byte of the Flash module).

---

## Concept

An SRAM is identified by `ramCfg_SramId_t` (`RAMCFG_SRAM_1`, `RAMCFG_SRAM_2`, ..., `RAMCFG_SRAM_BACKUP` - the SRAMs of the device). For every SRAM the
module keeps:

| Item | Meaning |
|---|---|
| **Check** (`ramCfg_CheckMode_t`) | `RAMCFG_CHECK_NONE`, `RAMCFG_CHECK_PARITY`, `RAMCFG_CHECK_ECC` - the checking the SRAM supports. |
| **Error reporting** | The single error and the double error are reported by the interrupt, the double error optionally by the NMI (`RAMCFG_REPORT_NMI`). |
| **Write protection** | A range of the pages (from the first page, the number of pages) that rejects the writes; locked until the reset. |
| **Statistics** | The number of the corrected single errors, of the double errors, the address of the last failing word. |
| **Wait states** | The access wait states of the SRAM, where the hardware offers the choice (a function of the clock and the voltage scale). |

Rules:

1. `RamCfg_Init` configures the SRAMs from the configuration (the check mode, the reporting, the protection) and starts the
   monitoring; an SRAM that does not support the requested check returns an error.
2. A **single error** is corrected by the hardware; the module counts it, remembers the address and calls the
   `SingleErrorCallback`. Optionally it writes the corrected word back (the **scrubbing** of the cell) so that the error does not persist.
3. A **double error** is not correctable: the module records the address and calls the `DoubleErrorCallback`
   (or the NMI handler of the application); the data at the address are lost.
4. `RamCfg_Set_Erase( sramId )` erases the SRAM and returns when the erase is finished (or with the complete callback);
   the content is zero after it. The erase of the SRAM used by the running code (the stack of the caller) is rejected.
5. The write protection is set per page and cannot be removed until the reset; an attempt returns an error.
6. All parameters are checked before a register is written; the TrustZone / privilege rules of the family are applied to the caller.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Controller | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `RAMCFG`: SRAM1, SRAM2, SRAM3 (SRAM4, SRAM5 on STM32H5E4 ... STM32H5F5), backup SRAM; ECC with single error correction and double error detection, the interrupt / NMI, the failing address, write protection in 1 KB pages, software erase | implemented (0.1.0) |
| STM32U5 | `Dev/STM32U5` | `RAMCFG`: SRAM1 ... SRAM5, backup SRAM, ECC on the selected SRAMs, write protection, erase, wait states | planned |
| STM32U3 | `Dev/STM32U3` | `RAMCFG` | planned |
| STM32G4, STM32L4 | `Dev/STM32G4`, `Dev/STM32L4` | parity check, write protection and erase of the SRAM2 / CCM SRAM in the `SYSCFG` | planned |
| STM32H7 | `Dev/STM32H7` | `RAMECC` monitors of the SRAMs (single / double error, failing address) | planned |
| STM32F4, STM32F7 | `Dev/STM32F4`, `Dev/STM32F7` | no ECC or parity on the SRAMs - the module is a stub that returns `RAMCFG_REQUEST_ERROR` | planned |

A function without a meaning in a family stays in the interface, returns `RAMCFG_REQUEST_ERROR` and suppresses the unused
parameters (for example the ECC on an SRAM with the parity only, the write protection of an SRAM without it, the erase of
a memory that the hardware cannot erase).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `RamCfg_Get_ModuleVersion`, `RamCfg_Init`, `RamCfg_Deinit`, `RamCfg_Task`, `RamCfg_Get_DefaultConfig` |
| Checking | `RamCfg_Set_CheckMode` / `Get`, `RamCfg_Set_ReportMode` / `Get`, `RamCfg_Set_Scrubbing` |
| Errors | `RamCfg_Get_ErrorStatistics`, `RamCfg_Set_ErrorStatisticsClear`, `RamCfg_Get_FailingAddress` |
| Protection and erase | `RamCfg_Set_WriteProtection`, `RamCfg_Get_WriteProtection`, `RamCfg_Set_Erase`, `RamCfg_Get_EraseState` |
| Wait states | `RamCfg_Set_WaitStates` / `Get` |
| Interrupts | `RamCfg_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |

```c
ramCfg_ModuleVersion_t RamCfg_Get_ModuleVersion     ( void );

ramCfg_RequestState_t  RamCfg_Init                  ( const ramCfg_Config_t * const ramCfgConfig );
ramCfg_RequestState_t  RamCfg_Deinit                ( void );
void                   RamCfg_Task                  ( void );
ramCfg_RequestState_t  RamCfg_Get_DefaultConfig     ( ramCfg_Config_t * const ramCfgConfig );

ramCfg_RequestState_t  RamCfg_Set_CheckMode         ( ramCfg_SramId_t sramId, ramCfg_CheckMode_t checkMode );
ramCfg_RequestState_t  RamCfg_Set_Scrubbing         ( ramCfg_SramId_t sramId, ramCfg_FunctionState_t scrubbing );

ramCfg_RequestState_t  RamCfg_Get_ErrorStatistics   ( ramCfg_SramId_t sramId, ramCfg_ErrorStatistics_t * const statistics );
ramCfg_RequestState_t  RamCfg_Get_FailingAddress    ( ramCfg_SramId_t sramId, ramCfg_Address_t * const failingAddress );

ramCfg_RequestState_t  RamCfg_Set_WriteProtection   ( ramCfg_SramId_t sramId, ramCfg_Page_t firstPage, ramCfg_PageCnt_t pageCnt );
ramCfg_RequestState_t  RamCfg_Set_Erase             ( ramCfg_SramId_t sramId );
ramCfg_RequestState_t  RamCfg_Get_EraseState        ( ramCfg_SramId_t sramId, ramCfg_EraseState_t * const eraseState );
```

Every function returns `RAMCFG_REQUEST_OK` / `RAMCFG_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `ramCfg_Config_t` | Per SRAM: check mode, reporting, scrubbing, write protection; callbacks (`SingleErrorCallback`, `DoubleErrorCallback`, `EraseCompleteCallback`), interrupt priority |
| `ramCfg_ErrorStatistics_t` | The number of the corrected single errors, of the double errors, the SRAM and the address of the last one |
| `ramCfg_EraseState_t` | `RAMCFG_ERASE_IDLE`, `RAMCFG_ERASE_BUSY`, `RAMCFG_ERASE_DONE` |

---

## Usage

### ECC on the SRAM with a log of the errors

```c
#include "RamCfg_Port.h"

static void App_SingleError( ramCfg_SramId_t sramId, ramCfg_Address_t address )
{
    /* a warning: the word was corrected (and scrubbed), count it in the diagnostic log */
}

static void App_DoubleError( ramCfg_SramId_t sramId, ramCfg_Address_t address )
{
    /* a fault: the data at the address are lost - go to the safe state */
}

ramCfg_Config_t ramCfgConfig;

(void)RamCfg_Get_DefaultConfig( &ramCfgConfig );
ramCfgConfig.Sram[ RAMCFG_SRAM_2 ].CheckMode = RAMCFG_CHECK_ECC;
ramCfgConfig.Sram[ RAMCFG_SRAM_2 ].Scrubbing = RAMCFG_FUNCTION_ACTIVE;
ramCfgConfig.SingleErrorCallback             = App_SingleError;
ramCfgConfig.DoubleErrorCallback             = App_DoubleError;

(void)RamCfg_Init( &ramCfgConfig );
```

### Erase of a secret

```c
(void)RamCfg_Set_Erase( RAMCFG_SRAM_3 );              /* the complete callback reports the end */
```

---

## Behaviour and limits

- **ECC initialization.** A memory with the ECC contains an undefined code word after the power-up; reading a word that was never written
  can report an error. The module (or the start-up code) initializes the memory by a write (erase) **before** the ECC is enabled
  for the first time; the family README describes it.
- **Latency.** The ECC check adds a wait state on some families; the wait-states setting follows from the clock and the voltage scale.
- **Interrupt and NMI.** The double error can be routed to the NMI, which preempts everything; the handler must be minimal.
- **Erase time.** The erase of a large SRAM takes microseconds to milliseconds; the SRAM is not usable during it.
- **Not part of the first version:** the erase of the SRAM at the reset (the option byte of the Flash module), the periodic software
  scrubbing of the whole memory, the error injection for the test (the hardware offers it on some families and the unit tests use the
  RegMem model instead).

---

## Dependencies

- **Rcc** - the clock of the SRAM controller
- **Nvic** - the ECC interrupt and the NMI
- **Tamp** - the erase of the SRAM on a tamper event is configured in the tamper module
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC and NVIC configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the acceptance and rejection of the check modes per SRAM, the single / double error flow with
  the statistics and the callbacks, the scrubbing, the write protection rules (range, lock), the erase sequence with the timeout, the
  rejected erase of the SRAM in use.
- **Integration tests** (target, probe-rs): the erase of an SRAM and the compare of the content, the write protection fault, the
  error injection of the family (where it exists) and the reported address.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `RAMCFG_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `RamCfg_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `RamCfg_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`ramCfg_FunctionState_t`, `ramCfg_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `RAMCFG_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `RamCfg_Port.h`, `RamCfg_Types.h` | Public interface (identical in all families) |
| `RamCfg*.c`, `RamCfg.h` | Implementation of the family |
| `CMakeLists.txt` | Library `RamCfg_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE RamCfg_Lib )
```

1. Link `RamCfg_Lib` to your CMake library.
2. Include `RamCfg_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
