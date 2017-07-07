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

find_package(OpenMP REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(glbinding REQUIRED)
find_package(globjects REQUIRED)
find_package(PNG REQUIRED)

if (NOT OPENGL_GLU_FOUND)
    message(FATAL_ERROR "OpenGL GLU library is required!")
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



set(sources
    src/config.h
    src/core.h
    src/core.cpp
    src/main.cpp
    src/render.h
    src/render.cpp
    # src/viewer.cpp
)

add_executable(nbody ${sources})
configure_target(nbody)

target_link_libraries(nbody
    PUBLIC
        OpenMP::OpenMP_CXX
        ${OPENGL_gl_LIBRARY}
        glfw
        glm
        globjects::globjects
        ${PNG_LIBRARIES}
)

# For libraries without imported targets: manual includes and definitions
target_include_directories(nbody SYSTEM
    PRIVATE
        ${OPENGL_INCLUDE_DIR}
        ${PNG_INCLUDE_DIRS}
)
target_compile_definitions(nbody
    PRIVATE
        ${PNG_DEFINITIONS}
        GLFW_INCLUDE_NONE
    PUBLIC
        # $<$<CONFIG:Debug>:_GLIBCXX_DEBUG>
)
