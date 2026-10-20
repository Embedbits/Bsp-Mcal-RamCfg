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

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

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
| STM32H5 | `Dev/STM32H5` | `RAMCFG`: SRAM1, SRAM2, SRAM3, backup SRAM; ECC with single error correction and double error detection, the interrupt / NMI, the failing address, write protection in 1 KB pages, software erase | planned (first) |
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
