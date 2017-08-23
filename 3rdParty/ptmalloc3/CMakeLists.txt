
set(TARGET_NAME ptmalloc3)

add_library(${TARGET_NAME} STATIC
    malloc-2.8.3.h
    ptmalloc3.c
    malloc.c
)

target_compile_options(${TARGET_NAME} PRIVATE
    -Wall -Wstrict-prototypes
    $<$<CONFIG:Release>:-g -O2>
)
target_compile_definitions(${TARGET_NAME} PRIVATE
    _GNU_SOURCE=1
    # USE_TSD_DATA_HACK
    THREAD_STATS=1
)
target_include_directories(${TARGET_NAME} PRIVATE
    .
    sysdeps/pthread
    sysdeps/generic
)
set_property(SOURCE ptmalloc3.c
    PROPERTY COMPILE_DEFINITIONS MSPACES=1
)
set_property(SOURCE malloc.c
    PROPERTY COMPILE_DEFINITIONS
        ONLY_MSPACES
        USE_LOCKS=0
)
