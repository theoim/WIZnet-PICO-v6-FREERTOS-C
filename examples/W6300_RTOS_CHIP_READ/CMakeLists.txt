set(TARGET_NAME W6300_RTOS_CHIP_READ)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )

target_include_directories(${TARGET_NAME} PUBLIC
        ${PORT_DIR}/FreeRTOS-Kernel/inc
)

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_pio
        FreeRTOS-Kernel-Heap4
        ETHERNET_FILES
        IOLIBRARY_FILES
        PORT_COMMON_FILES
        TIMER_FILES        
        mbedcrypto
        mbedx509
        mbedtls
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
