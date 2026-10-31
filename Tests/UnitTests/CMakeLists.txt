#======================== Unit tests configuration ============================#
# Unit tests of RamCfg module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(RAMCFG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${RAMCFG_DIR}/.." ABSOLUTE)

# The tests of the unlock sequences, the erase, the flags and the read-back verification use the HW model
# of RegMem (synchronous mode: the model reacts to every store, in the thread mode the tests which need it
# are ignored) - executed serially where the model runs in a thread
UnitTesting_Add_Test(
    NAME            RamCfg
    TEST_SOURCE     Test_RamCfg.c
    UUT_SOURCES     ${RAMCFG_DIR}/RamCfg.c
                    ${RAMCFG_DIR}/RamCfg_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
    INCLUDE_DIRS    ${RAMCFG_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
