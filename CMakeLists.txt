# CMake minimum required version
cmake_minimum_required(VERSION 3.12)

# Set board
# set(BOARD_NAME W6100_EVB_PICO)
# set(BOARD_NAME W6100_EVB_PICO2)
# set(BOARD_NAME W6300_EVB_PICO)
set(BOARD_NAME W6300_EVB_PICO2)

# Set ethernet chip
if(${BOARD_NAME} STREQUAL W6100_EVB_PICO)
    set(WIZNET_CHIP W6100)
    add_definitions(-D_WIZCHIP_=W6100)

elseif(${BOARD_NAME} STREQUAL W6100_EVB_PICO2)
    set(PICO_PLATFORM rp2350)
    set(WIZNET_CHIP W6100)
    add_definitions(-D_WIZCHIP_=W6100)

elseif(${BOARD_NAME} STREQUAL W6300_EVB_PICO)
    set(WIZNET_CHIP W6300)
    add_definitions(-D_WIZCHIP_=W6300)

elseif(${BOARD_NAME} STREQUAL W6300_EVB_PICO2)
    set(PICO_PLATFORM rp2350)
    set(WIZNET_CHIP W6300)
    add_definitions(-D_WIZCHIP_=W6300)

else() 
    message(FATAL_ERROR "BOARD_NAME is wrong = ${BOARD_NAME}")
endif()

message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")

if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
    message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
endif()

include(pico_sdk_import.cmake)
include(wiznet_pico_c_sdk_version.cmake)

# Pull in FreeRTOS (must be before project)
if(NOT DEFINED FREERTOS_KERNEL_PATH)
set(FREERTOS_KERNEL_PATH ${CMAKE_SOURCE_DIR}/libraries/FreeRTOS-Kernel)
    message(STATUS "FREERTOS_KERNEL_PATH = ${FREERTOS_KERNEL_PATH}")
endif()
include(FreeRTOS_Kernel_import.cmake)


# Set project name
set(PROJECT_NAME WIZNET-PICO-C)

# Set project informations
project(${PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Set the project root directory if it's not already defined, as may happen if
# the tests folder is included directly by a parent project, without including
# the top level CMakeLists.txt.
if(NOT DEFINED FREERTOS_DIR)
    set(FREERTOS_DIR ${CMAKE_SOURCE_DIR}/libraries/FreeRTOS-Kernel)
    message(STATUS "FREERTOS_DIR = ${FREERTOS_DIR}")
endif()


if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/io6Library)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED MBEDTLS_DIR)
    set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/libraries/mbedtls)
    message(STATUS "MBEDTLS_DIR = ${MBEDTLS_DIR}")
endif()

if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${CMAKE_SOURCE_DIR}/port)
    message(STATUS "PORT_DIR = ${PORT_DIR}")
endif()

# Turn off mbedtls test mode 
set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
add_definitions(-DMBEDTLS_CONFIG_FILE="${PORT_DIR}/mbedtls/inc/ssl_config.h")
add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)

# Hardware-specific examples in subdirectories:
add_subdirectory(examples)

# Add libraries in subdirectories
add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
add_subdirectory(${MBEDTLS_DIR})
add_subdirectory(${PORT_DIR})

if(${WIZNET_CHIP} STREQUAL W6300)
    # Set ethernet chip interface
    # Support 2 ethernet chip interfaces: SPI, PIO
    set(WIZNET_CHIP_INTERFACE PIO)

    if(${WIZNET_CHIP_INTERFACE} STREQUAL SPI)
        add_definitions(-D_WIZCHIP_IF_=SPI)
    elseif(${WIZNET_CHIP_INTERFACE} STREQUAL PIO)
        add_definitions(-D_WIZCHIP_IF_=PIO)
    else()
        message(FATAL_ERROR "WIZNET_CHIP_INTERFACE is wrong = ${WIZNET_CHIP_INTERFACE}")
    endif()

    message(STATUS "WIZNET_CHIP_INTERFACE = ${WIZNET_CHIP_INTERFACE}")

    # Set PIO interface
    # Support 4 PIO interfaces: SPI_SINGLE, SPI_DUAL, SPI_QUAD, SPI_OCTAL
    set(PIO_INTERFACE SPI_QUAD)

    if(${PIO_INTERFACE} STREQUAL SPI_SINGLE)
        add_definitions(-D_WIZCHIP_PIO_MODE_=QSPI_SINGLE_MODE)
    elseif(${PIO_INTERFACE} STREQUAL SPI_DUAL)
        add_definitions(-D_WIZCHIP_PIO_MODE_=QSPI_DUAL_MODE)
    elseif(${PIO_INTERFACE} STREQUAL SPI_QUAD)
        add_definitions(-D_WIZCHIP_PIO_MODE_=QSPI_QUAD_MODE)
    elseif(${PIO_INTERFACE} STREQUAL SPI_OCTAL)
        add_definitions(-D_WIZCHIP_PIO_MODE_=QSPI_OCTAL_MODE)
    else()
        message(FATAL_ERROR "PIO_INTERFACE is wrong = ${PIO_INTERFACE}")
    endif()

    message(STATUS "PIO_INTERFACE = ${PIO_INTERFACE}")
endif()

# Set compile options
add_compile_options(
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )


