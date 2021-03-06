cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(N-Body_Barnes_Hut C CXX)

foreach(p CMP0054)
    if (POLICY ${p})
        cmake_policy(SET ${p} NEW)
    endif()
endforeach()

if (DEFINED CMAKE_BUILD_TYPE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release")
endif()

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(CMakeDependentOption)

option(OPTION_WITH_RENDERING
    "Rendering support based on OpenGL, GLFW, glbinding and globjects." ON)
cmake_dependent_option(OPTION_WITH_PNG_EXPORT
    "Write each frame to a PNG file next to the executable" OFF
    "OPTION_WITH_RENDERING" OFF)

option(OPTION_PROFILING
    "Setup compile flags for profiling with GPROF (gcc only)." OFF)

option(OPTION_WITH_BENCHMARK "Add benchmarking target." ON)
set(OPTION_ALLOCATOR "std" CACHE STRING
    "Memory allocator: std (c library's default), tcmalloc (gperftools), ptmalloc3, or PGASUS (allocator only).")
set_property(CACHE OPTION_ALLOCATOR PROPERTY STRINGS "std" "tcmalloc" "ptmalloc3" "PGASUS")

set(OPTION_TIMING_EXCLUDE_FRAMES "10" CACHE STRING
    "Exclude first N frames from time measurement (total time is also printed).")
option(OPTION_MEASURE_OPERATION_TIMINGS "Output timings per operation" OFF)

option(OPTION_CENTER_OF_MASS_IS_BODY "Use Body class for Node::m_centerOfMass. \
This introduces some overhead, which can be used to measure the impact of occasional \
dynamic memory allocations." OFF)
set(CENTER_OF_MASS_IS_BODY ${OPTION_CENTER_OF_MASS_IS_BODY})
option(OPTION_LESS_DYNAMIC_ALLOCS "Reduce potential dynamic allocations in Node reset etc." ON)
set(LESS_DYNAMIC_ALLOCS ${OPTION_LESS_DYNAMIC_ALLOCS})
option(OPTION_PERSISTENT_NODE_VECTOR "Prevent dynamic allocations in node vector." ON)
set(PERSISTENT_NODE_VECTOR ${OPTION_PERSISTENT_NODE_VECTOR})

find_package(OpenMP REQUIRED)
find_package(glm QUIET)
if (NOT glm_FOUND)
    message("Using legacy glm find script...")
    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/legacy")
    find_package(glm REQUIRED)
endif()
if (OPTION_WITH_RENDERING)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(glbinding REQUIRED)
    find_package(globjects REQUIRED)
endif()
if (OPTION_WITH_PNG_EXPORT)
    find_package(PNG REQUIRED)
endif()

set(OPTION_BODY_INFLATE_BYTES 0 CACHE STRING "Increase the size of Body instances to fill the CPU caches...")
set(BODY_INFLATE_BYTES ${OPTION_BODY_INFLATE_BYTES})
set(OPTION_BODY_VALUE_STRIDE 0 CACHE STRING
    "Add padding between values to force each value to its own cache line.\
(POWER8: 128 bytes, Intel: 64 bytes)")
set(BODY_VALUE_STRIDE ${OPTION_BODY_VALUE_STRIDE})

if (OPTION_ALLOCATOR STREQUAL "tcmalloc")
    include(ExternalProject)

    ExternalProject_Add(
        gperftools
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/gperftools
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/gperftools_build
        DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/gperftools # hack: run autogen
        DOWNLOAD_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/gperftools/autogen.sh
        CONFIGURE_COMMAND
            ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/gperftools/configure
                --prefix=${CMAKE_CURRENT_BINARY_DIR}/3rdParty/gperftools
        BUILD_COMMAND
            make -j8
    )

elseif(OPTION_ALLOCATOR STREQUAL "ptmalloc3")
    add_subdirectory(3rdParty/ptmalloc3)

elseif(OPTION_ALLOCATOR STREQUAL "PGASUS")
    find_package(PGASUS REQUIRED)

endif()

include(cmake/PlatformDetection.cmake)


function(configure_target targetName linkLng)
    set_target_properties(${targetName} PROPERTIES LINKER_LANGUAGE ${linkLng})
    target_compile_options(${targetName} PRIVATE ${OpenMP_${linkLng}_FLAGS})
    if (NOT MSVC)
        target_link_libraries(${targetName} PUBLIC
            ${OpenMP_${linkLng}_LIB_NAMES}
            ${OpenMP_${linkLng}_FLAGS} # older systems/CMake versions require this flag here
        )
    endif()
    # target_compile_definitions(${targetName} PRIVATE GLM_FORCE_INLINE)

    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_compile_options(${targetName} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Werror-return-stack-address

            $<$<CONFIG:Debug>: -fstack-protector>

            -fcolor-diagnostics

            -ffast-math
        )
        if (OPTION_ALLOCATOR STREQUAL "tcmalloc")
            target_link_libraries(${targetName} PUBLIC
                -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free
                optimized "${CMAKE_CURRENT_BINARY_DIR}/3rdParty/gperftools/lib/libtcmalloc_minimal.so"
                debug "${CMAKE_CURRENT_BINARY_DIR}/3rdParty/gperftools/lib/libtcmalloc_minimal_debug.so"
            )
        elseif(OPTION_ALLOCATOR STREQUAL "ptmalloc3")
            target_link_libraries(${targetName} PUBLIC ptmalloc3)
        elseif(OPTION_ALLOCATOR STREQUAL "PGASUS")
            target_link_libraries(${targetName} PUBLIC PGASUS::hpinuma_msource)
        endif()
        if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.8)
            set(OPTION_CLANG_DEBUGGER lldb CACHE STRING
                "Debugger for that clang optimizes the DWARF debugging information")
            set_property(CACHE OPTION_CLANG_DEBUGGER PROPERTY STRINGS gdb lldb)
            target_compile_options(${targetName} PRIVATE
                $<$<CONFIG:Debug>:-g${OPTION_CLANG_DEBUGGER}>
            )
        endif()
        if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.0")
            target_compile_options(${targetName} PRIVATE
                $<$<CONFIG:Debug>:-fno-limit-debug-info>
            )
        endif()

        if (NOT PGASUS_PLATFORM_PPC64LE)
            target_compile_options(${targetName} PRIVATE
                -march=native -mtune=native
            )
        endif()

    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${targetName} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -ffast-math
        )
        target_compile_definitions(${targetName} PUBLIC
            _GLIBCXX_USE_CXX11_ABI
            $<$<CONFIG:Debug>:_GLIBCXX_DEBUG>
        )
        if (OPTION_ALLOCATOR STREQUAL "tcmalloc")
            target_link_libraries(${targetName} PUBLIC
                -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free
                optimized "${CMAKE_CURRENT_BINARY_DIR}/3rdParty/gperftools/lib/libtcmalloc_minimal.so"
                debug "${CMAKE_CURRENT_BINARY_DIR}/3rdParty/gperftools/lib/libtcmalloc_minimal_debug.so"
            )
        elseif(OPTION_ALLOCATOR STREQUAL "ptmalloc3")
            target_link_libraries(${targetName} PUBLIC ptmalloc3)
        elseif(OPTION_ALLOCATOR STREQUAL "PGASUS")
            target_link_libraries(${targetName} PUBLIC PGASUS::hpinuma_msource)
        endif()
        if (PGASUS_PLATFORM_PPC64LE)
            # See https://www.ibm.com/developerworks/community/wikis/home?lang=en#!/wiki/W51a7ffcf4dfd_4b40_9d82_446ebc23c550/page/IBM%20Advance%20Toolchain%20for%20PowerLinux%20Documentation?section=optimization
            # Ubuntu 16.04 packaged gcc does not support -march flags for POWER.
            # IBM Advance Toolchain 10.0 sets "-mcpu=power8 -mtune=power8" by default.
            # So nothing to set here for the required setup.
        else()  # generic case for other platforms, including x86
            target_compile_options(${targetName} PRIVATE
                -march=native -mtune=native -malign-double
            )
        endif()
        if (OPTION_PROFILING)
            target_compile_options(${targetName} PRIVATE -g -pg)
            target_link_libraries(${targetName} PRIVATE -g -pg)
        endif()
        if (linkLng STREQUAL "C")
            target_link_libraries(${targetName} PRIVATE -lm)
        endif()

    else()
        message(WARNING "Untested compiler: ${CMAKE_CXX_COMPILER_ID}")
        message(WARNING "Compiler not setup for gperftools/tcmalloc")
    endif()
endfunction()

set(CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/config.h")
configure_file("cmake/config.h.in" ${CONFIG_FILE})

set(sources
    ${CONFIG_FILE}
    src/core.h
    src/core.cpp
    src/main.cpp
)
if (OPTION_WITH_RENDERING)
    list(APPEND sources
        src/render.h
        src/render.cpp
    )
endif()
if (OPTION_WITH_PNG_EXPORT)
    list(APPEND sources
        src/bitmap.h
        src/bitmap.cpp
    )
endif()


add_executable(nbody ${sources})
configure_target(nbody CXX)

target_include_directories(nbody PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(nbody SYSTEM PUBLIC ${GLM_INCLUDE_DIR})

if (OPTION_WITH_RENDERING)
    target_link_libraries(nbody
        PUBLIC
            ${OPENGL_gl_LIBRARY}
            glfw
            globjects::globjects
    )
    target_include_directories(nbody SYSTEM PRIVATE ${OPENGL_INCLUDE_DIR})
    target_compile_definitions(nbody PRIVATE GLFW_INCLUDE_NONE)
endif()

if (OPTION_WITH_PNG_EXPORT)
    target_link_libraries(nbody PUBLIC ${PNG_LIBRARIES})
    target_include_directories(nbody SYSTEM PRIVATE ${PNG_INCLUDE_DIRS})
    target_compile_definitions(nbody PRIVATE ${PNG_DEFINITIONS})
endif()


if (OPTION_WITH_BENCHMARK)
    find_package(Boost COMPONENTS filesystem REQUIRED)
    add_executable(nbody_benchmark
        ${CONFIG_FILE}
        src/core.h
        src/core.cpp
        src/bench_main.cpp
    )
    configure_target(nbody_benchmark CXX)
    target_link_libraries(nbody_benchmark PUBLIC
        Boost::filesystem
    )
    target_compile_definitions(nbody_benchmark PRIVATE
        BOOST_FILESYSTEM_NO_DEPRECATED
    )
    target_include_directories(nbody_benchmark PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    target_include_directories(nbody_benchmark SYSTEM PUBLIC ${GLM_INCLUDE_DIR})
endif()


add_executable(nbody_c
    src/c/config.h
    src/c/core.h
    src/c/core.c
    src/c/main.c
)
configure_target(nbody_c C)
