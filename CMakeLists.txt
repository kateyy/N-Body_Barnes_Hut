cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(N-Body_Barnes_Hut CXX)

foreach(p CMP0054)
    if (POLICY ${p})
        cmake_policy(SET ${p} NEW)
    endif()
endforeach()

include(CMakeDependentOption)

option(OPTION_WITH_RENDERING
    "Rendering support based on OpenGL, GLFW, glbinding and globjects." ON)
cmake_dependent_option(OPTION_WITH_PNG_EXPORT
    "Write each frame to a PNG file next to the executable" OFF
    "OPTION_WITH_RENDERING" OFF)

find_package(OpenMP REQUIRED)
find_package(glm REQUIRED)
if (OPTION_WITH_RENDERING)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(glbinding REQUIRED)
    find_package(globjects REQUIRED)
endif()
if (OPTION_WITH_PNG_EXPORT)
    find_package(PNG REQUIRED)
endif()

set(PGASUS_PLATFORM_X86_64 0)
set(PGASUS_PLATFORM_PPC64LE 0)
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(PGASUS_PLATFORM_X86_64 1)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "ppc64le")
    set(PGASUS_PLATFORM_PPC64LE 1)
else()
    message(WARNING "Unknown platform: \"${CMAKE_SYSTEM_PROCESSOR}\"")
endif()

function(configure_target targetName)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_compile_options(${targetName} PRIVATE
            -Wpedantic
            -Wall
            -ffast-math
        )
        if (NOT CMAKE_CXX_COMPILER_ID VERSION_LESS 3.8)
            target_compile_options(${targetName} PRIVATE
                $<$<CONFIG:Debug>:-g${OPTION_CLANG_DEBUGGER}>
            )
        endif()
        if (NOT CMAKE_CXX_COMPILER_ID VERSION_LESS "4.0")
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
            -Wpedantic
            -Wall
            -ffast-math -malign-double
        )

        if (NOT PGASUS_PLATFORM_PPC64LE)
            target_compile_options(${targetName} PRIVATE
                -march=native -mtune=native
            )
        endif()

    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        target_compile_options(${targetName} PRIVATE
            /W4 /we4013 /wd4201 /wd4251 /wd4505
            /MP
            /Zc:forScope /Zc:inline /Zc:rvalueCast /Zc:wchar_t
            $<$<CONFIG:Debug>:      /F 33554432>
            $<$<CONFIG:Release>:    /F 8388608>
            $<$<CONFIG:Release>:
                /Oi /Ot /Oy /GL /fp:fast /arch:AVX2 /GS-
            >
            $<$<CONFIG:RelWithDebInfo>:
                /Oy- /fp:fast /arch:AVX2 /GS- /Zi
            >
        )
        target_compile_definitions(${targetName} PRIVATE _CRT_SECURE_NO_WARNINGS)
        set_target_properties(${targetName} PROPERTIES
            LINK_FLAGS_DEBUG            "/STACK:33554432"
            LINK_FLAGS_RELEASE          "/STACK:8388608 /OPT:REF /OPT:ICF /LTCG"
            LINK_FLAGS_RELWITHDEBINFO   "/STACK:8388608 /DEBUG /INCREMENTAL:NO"
        )
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

add_executable(nbody ${sources})
configure_target(nbody)

target_link_libraries(nbody
    PUBLIC
        OpenMP::OpenMP_CXX
        glm
)
target_include_directories(nbody PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_compile_definitions(nbody
    PUBLIC
        # $<$<CONFIG:Debug>:_GLIBCXX_DEBUG>
)

if (OPTION_WITH_RENDERING)
    target_link_libraries(nbody
        PUBLIC
            OpenMP::OpenMP_CXX
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
