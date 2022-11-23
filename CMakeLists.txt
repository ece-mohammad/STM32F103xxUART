cmake_minimum_required(VERSION 3.18)

######################################################
# C99 standard
######################################################
set(CMAKE_C_STANDARD                99)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
# set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

######################################################
# Top level paths
######################################################

set(PROJECT_PATH                    ${CMAKE_CURRENT_SOURCE_DIR})
set(APPLICATIONS_DIR                ${PROJECT_PATH}/Application)
set(ROOT_BUILD_DIR                  build)

message("Build type: "              ${CMAKE_BUILD_TYPE})

######################################################
# project
######################################################
project(STM32F103X_Serial
    VERSION         1.0.0.0
    DESCRIPTION 
    HOMEPAGE_URL    https://github.com/ece-mohammad/STM32f1xxUART
    # LANGUAGES       C ASM
)

configure_file(VERSION_TEMPLATE VERSION)

######################################################
# project eabled languages
######################################################
enable_language(C ASM)


######################################################
# Global compiler and linker flags & options
######################################################

# Common compiler definitions for all aplications
set(COMPILER_DEFINITIONS
    "USE_FULL_ASSERT=1U"
    "HSE_VALUE=8000000"
    "HSE_STARTUP_TIMEOUT=100"
    "LSE_STARTUP_TIMEOUT=5000"
    "LSE_VALUE=32768"
    "HSI_VALUE=8000000"
    "LSI_VALUE=40000"
    "VDD_VALUE=3300"
    "PREFETCH_ENABLE=1"
    "STM32F103xB"
)

# include applications
include(${APPLICATIONS_DIR}/Applications.cmake)


# include CMakeFiles for each application
foreach(app ${APPLICATIONS})
    message(Application: ${app})
    
    include(${APPLICATIONS_DIR}/${app}/CMakeLists.txt OPTIONAL RESULT_VARIABLE found)
    
    if(NOT found)
        message(SEND_ERROR Couldn't find CMakeLists.txt for application: ${app})
    endif()

endforeach()
