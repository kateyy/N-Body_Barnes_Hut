
set(PGASUS_PLATFORM_X86_64 0)
set(PGASUS_PLATFORM_PPC64LE 0)
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(PGASUS_PLATFORM_X86_64 1)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "ppc64le")
    set(PGASUS_PLATFORM_PPC64LE 1)
else()
    message(WARNING "Unknown platform: \"${CMAKE_SYSTEM_PROCESSOR}\"")
endif()

function(detect_cpu_model OUTPUT_MODEL_NAME)
    set(${OUTPUT_MODEL_NAME} "Unkown" PARENT_SCOPE)
    if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
        execute_process(
            COMMAND bash "-c" "cat /proc/cpuinfo | grep cpu | head -n1  | awk '{print $3}'"
            RESULT_VARIABLE returnValue
            OUTPUT_VARIABLE cpuModel)
        if (returnValue EQUAL 0)
            set(${OUTPUT_MODEL_NAME} ${cpuModel} PARENT_SCOPE)
        endif()
    else()
    endif()
endfunction()

if (NOT SYSTEM_CPU_MODEL)
    detect_cpu_model(SYSTEM_CPU_MODEL)
    set(SYSTEM_CPU_MODEL ${SYSTEM_CPU_MODEL} CACHE INTERNAL "")
endif()

message("Detected CPU model: ${SYSTEM_CPU_MODEL}")
