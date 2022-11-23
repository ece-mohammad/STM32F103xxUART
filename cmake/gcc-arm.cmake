###################################################
# 
# This file defines common compiler and linker options
# for GCC aRM compiler 
# 
###################################################

include_guard(DIRECTORY)

# Common compiler options for all applications
set(COMPILER_FLAGS
    ${GCC_CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    --std=c99
)

# Common linker options for all applications
set(LINKER_FLAGS
    -T${LINKER_SCRIPT_SRC}
    ${GCC_CPU_PARAMETERS}
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
    -Wl,--print-memory-usage
)