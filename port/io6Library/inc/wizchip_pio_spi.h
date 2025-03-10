/**
 * Copyright (c) 2024 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _WIZCHIP_PIO_SPI_H_
#define _WIZCHIP_PIO_SPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdint.h>

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
#define PIO_SPI_INSTANCE_COUNT 1
#define PIO_SPI_PREFERRED_PIO 1
#define IRQ_SAMPLE_DELAY_NS 100

/*
SPI SCLK SPEED = 66.5MHz / (PIO_CLOCK_DIV_MAJOR + (PIO_CLOCK_DIV_MINOR / 256)) 
*/
#define PIO_CLOCK_DIV_MAJOR 13
#define PIO_CLOCK_DIV_MINOR 77

#define PIO_IRQ_PIN             15
#define PIO_SPI_SCK_PIN         17
#define PIO_SPI_DATA_IO0_PIN    18
#define PIO_SPI_DATA_IO1_PIN    19
#define PIO_SPI_DATA_IO2_PIN    20
#define PIO_SPI_DATA_IO3_PIN    21
#define PIO_SPI_CS_PIN          16
#define PIO_RESET_PIN           22

#define EN_WRITE_FLASH 0
#define EN_DISABLE_FLASH 1
#define PAGE_PROGRAM_FLASH 2

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
typedef struct wizchip_pio_spi_config
{
  uint16_t clock_div_major;
  uint8_t clock_div_minor;
  uint8_t clock_pin;
  uint8_t data_io0_pin;
  uint8_t data_io1_pin;
  uint8_t data_io2_pin;
  uint8_t data_io3_pin;
  uint8_t cs_pin;
  uint8_t reset_pin;
  uint8_t irq_pin;
} wizchip_pio_spi_config_t; 

//typedef struct wizchip_pio_spi_funcs **wizchip_pio_spi_handle_t;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

static void wizchip_pio_spi_deinit();
static void wizchip_pio_spi_gpio_config();
// static void wizchip_pio_spi_cs_set(bool value);
// static void wizchip_pio_spi_cs_set(wizchip_pio_spi_state_t *state, bool value);

static void wizchip_pio_spi_read_buffer(void);
static void wizchip_pio_spi_write_buffer(void);

int wizchip_pio_spi_init(const wizchip_pio_spi_config_t *config);
void wizchip_pio_spi_reset();
void wizchip_pio_spi_frame_start(void);
void wizchip_pio_spi_frame_end(void);
void wizchip_pio_spi_read_byte(uint8_t op_code, uint16_t AddrSel, uint8_t *rx, uint16_t rx_length);
void wizchip_pio_spi_write_byte(uint8_t op_code, uint16_t AddrSel, uint8_t *tx, uint16_t tx_length);
wizchip_pio_spi_config_t *wizchip_default_pio_spi_config(void);

void wizchip_pio_spi_read_flash(uint8_t op_code, uint16_t AddrSel, uint8_t *rx, uint16_t rx_length);


#ifdef __cplusplus
}
#endif

#endif /* _WIZCHIP_PIO_SPI_H_ */
