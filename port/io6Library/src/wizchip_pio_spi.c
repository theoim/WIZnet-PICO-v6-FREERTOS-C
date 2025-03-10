/**
 * Copyright (c) 2024 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "wizchip_conf.h"
#include "wizchip_pio_spi.h"
#include "wizchip_pio_spi.pio.h"
  
/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
#define PADS_DRIVE_STRENGTH PADS_BANK0_GPIO0_DRIVE_VALUE_12MA

#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
#define PIO_SPI_PROGRAM_NAME wizchip_pio_spi_single_write_read
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
#define PIO_SPI_PROGRAM_NAME wizchip_pio_spi_dual_write_read
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
#define PIO_SPI_PROGRAM_NAME wizchip_pio_spi_quad_write_read
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
#define PIO_SPI_PROGRAM_NAME wizchip_pio_spi_octal_write_read
#endif

#define PIO_SPI_PROGRAM_FUNC __CONCAT(PIO_SPI_PROGRAM_NAME, _program)
#define PIO_SPI_PROGRAM_GET_DEFAULT_CONFIG_FUNC __CONCAT(PIO_SPI_PROGRAM_NAME, _program_get_default_config)
#define PIO_SPI_OFFSET_WRITE_BITS __CONCAT(PIO_SPI_PROGRAM_NAME, _offset_write_bits)  
#define PIO_SPI_OFFSET_WRITE_BITS_END __CONCAT(PIO_SPI_PROGRAM_NAME, _offset_write_bits_end)  
#define PIO_SPI_OFFSET_READ_BITS_END __CONCAT(PIO_SPI_PROGRAM_NAME, _offset_read_bits_end)

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
typedef struct wizchip_pio_spi_state
{
  //wizchip_pio_spi_funcs_t *pio_spi_funcs;
  const wizchip_pio_spi_config_t *pio_spi_config;
  pio_hw_t *pio;
  uint8_t pio_func_sel;
  int8_t pio_offset;
  int8_t pio_sm;
  int8_t dma_out;
  int8_t dma_in;
  uint32_t freq;
} wizchip_pio_spi_state_t;

static wizchip_pio_spi_state_t wizchip_pio_spi_state[PIO_SPI_INSTANCE_COUNT];
static wizchip_pio_spi_state_t *active_state;

static dma_channel_config out_config;
static dma_channel_config in_config;

wizchip_pio_spi_config_t pio_spi_default_config = {
    .clock_div_major = PIO_CLOCK_DIV_MAJOR,
    .clock_div_minor = PIO_CLOCK_DIV_MINOR,
    .clock_pin = PIO_SPI_SCK_PIN,
    .data_io0_pin = PIO_SPI_DATA_IO0_PIN,
    .data_io1_pin = PIO_SPI_DATA_IO1_PIN,
    .data_io2_pin = PIO_SPI_DATA_IO2_PIN,
    .data_io3_pin = PIO_SPI_DATA_IO3_PIN,
    .cs_pin = PIO_SPI_CS_PIN,
    .reset_pin = PIO_RESET_PIN,
    .irq_pin = PIO_IRQ_PIN,
};

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
static void wizchip_pio_spi_deinit();
static void wizchip_pio_spi_gpio_config();
static void wizchip_pio_spi_cs_set(wizchip_pio_spi_state_t *state, bool value);

static void wizchip_pio_spi_read_buffer(void);
static void wizchip_pio_spi_write_buffer(void);

static uint16_t mk_cmd_buf(uint8_t *pdst, uint8_t opcode, uint16_t addr);

void wizchip_pio_spi_reset();
void wizchip_pio_spi_frame_start(void);
void wizchip_pio_spi_frame_end(void);
void wizchip_pio_spi_read_byte(uint8_t op_code, uint16_t AddrSel, uint8_t *rx, uint16_t rx_length);
void wizchip_pio_spi_write_byte(uint8_t op_code, uint16_t AddrSel, uint8_t *tx, uint16_t tx_length);
wizchip_pio_spi_config_t *wizchip_default_pio_spi_config(void);

int wizchip_pio_spi_init(const wizchip_pio_spi_config_t *config)
{
  uint8_t offset_ = 0;
  active_state = &wizchip_pio_spi_state[0];

  active_state->pio_spi_config = wizchip_default_pio_spi_config();
  wizchip_pio_spi_gpio_config();

  pio_hw_t *pios[2] = {pio0, pio1};
  uint pio_index = PIO_SPI_PREFERRED_PIO;

  // Check we can add the program
  if (!pio_can_add_program(pios[pio_index], &PIO_SPI_PROGRAM_FUNC))
  {
    pio_index ^= 1;
    if (!pio_can_add_program(pios[pio_index], &PIO_SPI_PROGRAM_FUNC))
    {
      return -1;
    }
  }

  active_state->pio = pios[pio_index];
  active_state->dma_in = -1;
  active_state->dma_out = -1;

  active_state->pio_func_sel = GPIO_FUNC_PIO0 + pio_index;
  active_state->pio_sm = (int8_t)pio_claim_unused_sm(active_state->pio, false);
  if (active_state->pio_sm < 0)
  {
    wizchip_pio_spi_deinit();
    return -1;
  }

  active_state->pio_offset = pio_add_program(active_state->pio, &PIO_SPI_PROGRAM_FUNC);
  pio_sm_config sm_config = PIO_SPI_PROGRAM_GET_DEFAULT_CONFIG_FUNC(active_state->pio_offset);

  sm_config_set_clkdiv_int_frac(&sm_config, active_state->pio_spi_config->clock_div_major, active_state->pio_spi_config->clock_div_minor);
  
  active_state->freq = (uint32_t)(clock_get_hz(clk_peri) / active_state->pio_spi_config->clock_div_major);
 
  hw_write_masked(&pads_bank0_hw->io[active_state->pio_spi_config->clock_pin],
                  (uint)PADS_DRIVE_STRENGTH << PADS_BANK0_GPIO0_DRIVE_LSB,
                  PADS_BANK0_GPIO0_DRIVE_BITS);
  hw_write_masked(&pads_bank0_hw->io[active_state->pio_spi_config->clock_pin],
                  (uint)1 << PADS_BANK0_GPIO0_SLEWFAST_LSB,
                  PADS_BANK0_GPIO0_SLEWFAST_BITS);

  printf("[SPI CLOCK SPEED : %.2lf MHz]\r\n", 66.5 / (PIO_CLOCK_DIV_MAJOR + ((double)PIO_CLOCK_DIV_MINOR / 256)));

#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  printf("\r\n[QSPI SINGLE MODE]\r\n");
  sm_config_set_out_pins(&sm_config, active_state->pio_spi_config->data_io0_pin, 1);
  sm_config_set_in_pins(&sm_config, active_state->pio_spi_config->data_io1_pin);
  sm_config_set_set_pins(&sm_config, active_state->pio_spi_config->data_io0_pin, 2);
  sm_config_set_sideset(&sm_config, 1, false, false);
  sm_config_set_sideset_pins(&sm_config, active_state->pio_spi_config->clock_pin);

  sm_config_set_in_shift(&sm_config, false, true, 8);
  sm_config_set_out_shift(&sm_config, false, true, 8);

  hw_set_bits(&active_state->pio->input_sync_bypass,
              (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin));
  pio_sm_set_config(active_state->pio, active_state->pio_sm, &sm_config);
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->clock_pin, 1, true);

  gpio_set_function(active_state->pio_spi_config->data_io0_pin, active_state->pio_func_sel);

  // Set data pin to pull down and schmitt
  gpio_set_pulls(active_state->pio_spi_config->data_io0_pin, false, true);
  gpio_set_pulls(active_state->pio_spi_config->data_io1_pin, false, true);
  gpio_set_input_hysteresis_enabled(active_state->pio_spi_config->data_io0_pin, true);
  gpio_set_input_hysteresis_enabled(active_state->pio_spi_config->data_io1_pin, true);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  printf("[QSPI DUAL MODE]\r\n\r\n");
  sm_config_set_out_pins(&sm_config, active_state->pio_spi_config->data_io0_pin, 2);
  sm_config_set_in_pins(&sm_config, active_state->pio_spi_config->data_io0_pin);
  sm_config_set_set_pins(&sm_config, active_state->pio_spi_config->data_io0_pin, 2);
  sm_config_set_sideset(&sm_config, 1, false, false);
  sm_config_set_sideset_pins(&sm_config, active_state->pio_spi_config->clock_pin);

  sm_config_set_in_shift(&sm_config, false, true, 8);
  sm_config_set_out_shift(&sm_config, false, true, 8);

  hw_set_bits(&active_state->pio->input_sync_bypass,
              (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin));
  pio_sm_set_config(active_state->pio, active_state->pio_sm, &sm_config);
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->clock_pin, 1, true);

  gpio_set_function(active_state->pio_spi_config->data_io0_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io1_pin, active_state->pio_func_sel);

  // Set data pin to pull down and schmitt
  gpio_set_pulls(active_state->pio_spi_config->data_io0_pin, false, true);
  gpio_set_pulls(active_state->pio_spi_config->data_io1_pin, false, true);
  gpio_set_input_hysteresis_enabled(active_state->pio_spi_config->data_io0_pin, true);
  gpio_set_input_hysteresis_enabled(active_state->pio_spi_config->data_io1_pin, true);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  printf("\r\n[QSPI QUAD MODE]\r\n");
  sm_config_set_out_pins(&sm_config, active_state->pio_spi_config->data_io0_pin, 4);
  sm_config_set_in_pins(&sm_config, active_state->pio_spi_config->data_io0_pin);
  sm_config_set_set_pins(&sm_config, active_state->pio_spi_config->data_io0_pin, 4);
  sm_config_set_sideset(&sm_config, 1, false, false);
  sm_config_set_sideset_pins(&sm_config, active_state->pio_spi_config->clock_pin);

  sm_config_set_in_shift(&sm_config, false, true, 8);
  sm_config_set_out_shift(&sm_config, false, true, 8);

  hw_set_bits(&active_state->pio->input_sync_bypass,
              (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin) | (1u << active_state->pio_spi_config->data_io2_pin) | (1u << active_state->pio_spi_config->data_io3_pin));
  pio_sm_set_config(active_state->pio, active_state->pio_sm, &sm_config);
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->clock_pin, 1, true);

  gpio_set_function(active_state->pio_spi_config->data_io0_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io1_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io2_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io3_pin, active_state->pio_func_sel);

  // Set data pin to pull down and schmitt
  gpio_set_pulls(active_state->pio_spi_config->data_io0_pin, false, true);
  gpio_set_pulls(active_state->pio_spi_config->data_io1_pin, false, true);
  gpio_set_pulls(active_state->pio_spi_config->data_io2_pin, false, true);
  gpio_set_pulls(active_state->pio_spi_config->data_io3_pin, false, true);
  gpio_set_input_hysteresis_enabled(active_state->pio_spi_config->data_io0_pin, true);
  gpio_set_input_hysteresis_enabled(active_state->pio_spi_config->data_io1_pin, true);
  gpio_set_input_hysteresis_enabled(active_state->pio_spi_config->data_io2_pin, true);
  gpio_set_input_hysteresis_enabled(active_state->pio_spi_config->data_io3_pin, true);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
  /* @todo: Implement to use. */
#endif

  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_set(pio_pins, 1));

  active_state->dma_out = (int8_t)dma_claim_unused_channel(false); // todo: Should be able to use one dma channel?
  active_state->dma_in = (int8_t)dma_claim_unused_channel(false);
  if (active_state->dma_out < 0 || active_state->dma_in < 0)
  {
    wizchip_pio_spi_deinit();
    return -1;
  }
  return 0;
}

static void wizchip_pio_spi_deinit()
{
  if (active_state)
  {
    if (active_state->pio_sm >= 0)
    {
      if (active_state->pio_offset != -1)
        pio_remove_program(active_state->pio, &PIO_SPI_PROGRAM_FUNC, active_state->pio_offset);
      pio_sm_unclaim(active_state->pio, active_state->pio_sm);
    }
    if (active_state->dma_out >= 0)
    {
      dma_channel_unclaim(active_state->dma_out);
      active_state->dma_out = -1;
    }
    if (active_state->dma_in >= 0)
    {
      dma_channel_unclaim(active_state->dma_in);
      active_state->dma_in = -1;
    }
  }
}

static void wizchip_pio_spi_gpio_config()
{
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  // Setup DO and DI
  gpio_init(active_state->pio_spi_config->data_io0_pin);
  gpio_init(active_state->pio_spi_config->data_io1_pin);
  gpio_set_dir(active_state->pio_spi_config->data_io0_pin, GPIO_OUT);
  gpio_set_dir(active_state->pio_spi_config->data_io1_pin, GPIO_OUT);
  gpio_put(active_state->pio_spi_config->data_io0_pin, false);
  gpio_put(active_state->pio_spi_config->data_io1_pin, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  // Setup DO and DI
  gpio_init(active_state->pio_spi_config->data_io0_pin);
  gpio_init(active_state->pio_spi_config->data_io1_pin);
  gpio_set_dir(active_state->pio_spi_config->data_io0_pin, GPIO_OUT);
  gpio_set_dir(active_state->pio_spi_config->data_io1_pin, GPIO_OUT);
  gpio_put(active_state->pio_spi_config->data_io0_pin, false);
  gpio_put(active_state->pio_spi_config->data_io1_pin, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  // Setup DO and DI
  gpio_init(active_state->pio_spi_config->data_io0_pin);
  gpio_init(active_state->pio_spi_config->data_io1_pin);
  gpio_init(active_state->pio_spi_config->data_io2_pin);
  gpio_init(active_state->pio_spi_config->data_io3_pin);
  gpio_set_dir(active_state->pio_spi_config->data_io0_pin, GPIO_OUT);
  gpio_set_dir(active_state->pio_spi_config->data_io1_pin, GPIO_OUT);
  gpio_set_dir(active_state->pio_spi_config->data_io2_pin, GPIO_OUT);
  gpio_set_dir(active_state->pio_spi_config->data_io3_pin, GPIO_OUT);
  gpio_put(active_state->pio_spi_config->data_io0_pin, false);
  gpio_put(active_state->pio_spi_config->data_io1_pin, false);
  gpio_put(active_state->pio_spi_config->data_io2_pin, false);
  gpio_put(active_state->pio_spi_config->data_io3_pin, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
#endif

  // Setup CS
  gpio_init(active_state->pio_spi_config->cs_pin);
  gpio_set_dir(active_state->pio_spi_config->cs_pin, GPIO_OUT);
  gpio_put(active_state->pio_spi_config->cs_pin, true);

  // Setup reset
  gpio_init(active_state->pio_spi_config->reset_pin);
  gpio_set_dir(active_state->pio_spi_config->reset_pin, GPIO_OUT);
  gpio_put(active_state->pio_spi_config->reset_pin, true);

  // Setup reset
  gpio_init(active_state->pio_spi_config->irq_pin);
  gpio_set_dir(active_state->pio_spi_config->irq_pin, GPIO_IN);
  gpio_set_pulls(active_state->pio_spi_config->irq_pin, false, false);
}

// static void wizchip_pio_spi_cs_set(bool value)
// {
//   gpio_put(active_state->pio_spi_config->cs_pin, value);
// }

void wizchip_pio_spi_reset()
{
  gpio_set_dir(active_state->pio_spi_config->reset_pin, GPIO_OUT);
  gpio_put(active_state->pio_spi_config->reset_pin, 0);
  sleep_ms(500);
  gpio_put(active_state->pio_spi_config->reset_pin, 1);
  sleep_ms(500);
}

static __noinline void ns_delay(uint32_t ns) {
    // cycles = ns * clk_sys_hz / 1,000,000,000
    uint32_t cycles = ns * (clock_get_hz(clk_sys) >> 16u) / (1000000000u >> 16u);
    busy_wait_at_least_cycles(cycles);
}

void wizchip_pio_spi_frame_start(void)
{
  assert(active_state);
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  gpio_set_function(active_state->pio_spi_config->data_io0_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io1_pin, active_state->pio_func_sel);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  gpio_set_function(active_state->pio_spi_config->data_io0_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io1_pin, active_state->pio_func_sel);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  gpio_set_function(active_state->pio_spi_config->data_io0_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io1_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io2_pin, active_state->pio_func_sel);
  gpio_set_function(active_state->pio_spi_config->data_io3_pin, active_state->pio_func_sel);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
  /* @todo: Implement to use. */
#endif
  gpio_set_function(active_state->pio_spi_config->clock_pin, active_state->pio_func_sel);
  gpio_pull_down(active_state->pio_spi_config->clock_pin);

  // Pull CS low
  // wizchip_pio_spi_cs_set(false);
  wizchip_pio_spi_cs_set(active_state, false);
}

void wizchip_pio_spi_frame_end(void)
{
  assert(active_state);
  // uint64_t cs_hold_time = 2000 * 1000 * (float)((float)1 / active_state->freq);
  // Pull CS high
  wizchip_pio_spi_cs_set(active_state, true);
  // sleep_us(cs_hold_time);
  #ifdef IRQ_SAMPLE_DELAY_NS
  ns_delay(IRQ_SAMPLE_DELAY_NS);
  #endif
}

static void wizchip_pio_spi_cs_set(wizchip_pio_spi_state_t *state, bool value)
{
  gpio_put(state->pio_spi_config->cs_pin, value);
}

static uint16_t mk_cmd_buf(uint8_t *pdst, uint8_t opcode, uint16_t addr)
{
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)

  pdst[0] = opcode;
  pdst[1] = (uint8_t)((addr >> 8) & 0xFF); 
  pdst[2] = (uint8_t)((addr >> 0) & 0xFF); 
  pdst[3] = 0;

  return 3 + 1;
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  pdst[0] = ((opcode >> 7 & 0x01) << 6) | ((opcode >> 6 & 0x01) << 4) | ((opcode >> 5 & 0x01) << 2) | ((opcode >> 4 & 0x01) << 0);
  pdst[1] = ((opcode >> 3 & 0x01) << 6) | ((opcode >> 2 & 0x01) << 4) | ((opcode >> 1 & 0x01) << 2) | ((opcode >> 0 & 0x01) << 0);
  pdst[2] = (uint8_t)((addr >> 8) & 0xFF); 
  pdst[3] = (uint8_t)((addr >> 0) & 0xFF); 

  pdst[4] = 0;

  return 4 + 1;
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  pdst[0] = ( (opcode >> 7 & 0x01) << 4 ) | ( (opcode >> 6 & 0x01) << 0 );
  pdst[1] = ( (opcode >> 5 & 0x01) << 4 ) | ( (opcode >> 4 & 0x01) << 0 );
  pdst[2] = ( (opcode >> 3 & 0x01) << 4 ) | ( (opcode >> 2 & 0x01) << 0 );
  pdst[3] = ( (opcode >> 1 & 0x01) << 4 ) | ( (opcode >> 0 & 0x01) << 0 );

  pdst[4] = ((uint8_t)(addr >> 8) & 0xFF); 
  pdst[5] = ((uint8_t)(addr >> 0) & 0xFF); 

  pdst[6] = 0;

  return 6 + 1;
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
  /* @todo: Implement to use. */
#endif
  return 0;
}

void wizchip_pio_spi_read_byte(uint8_t op_code, uint16_t AddrSel, uint8_t *rx, uint16_t rx_length) 
{
  uint8_t command_buf[8] = {0,};
  uint16_t command_len = mk_cmd_buf(command_buf, op_code, AddrSel);
  uint32_t loop_cnt = 0;

  //wizchip_pio_spi_frame_start();

  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
  pio_sm_set_wrap(active_state->pio, active_state->pio_sm, active_state->pio_offset, active_state->pio_offset + PIO_SPI_OFFSET_READ_BITS_END - 1);
  //pio_sm_set_wrap(active_state->pio, active_state->pio_sm, active_state->pio_offset + PIO_SPI_OFFSET_WRITE_BITS, active_state->pio_offset + PIO_SPI_OFFSET_READ_BITS_END - 1);
  pio_sm_clear_fifos(active_state->pio, active_state->pio_sm);

#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  loop_cnt = 8;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin), (1u << active_state->pio_spi_config->data_io0_pin));// | (1u << active_state->pio_spi_config->data_io1_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  loop_cnt = 4;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin),
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  loop_cnt = 2;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin) | (1u << active_state->pio_spi_config->data_io2_pin) | (1u << active_state->pio_spi_config->data_io3_pin),
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin) | (1u << active_state->pio_spi_config->data_io2_pin) | (1u << active_state->pio_spi_config->data_io3_pin));

#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
    /* @todo: Implement to use. */
#endif

  pio_sm_restart(active_state->pio, active_state->pio_sm);
  pio_sm_clkdiv_restart(active_state->pio, active_state->pio_sm);

  pio_sm_put(active_state->pio, active_state->pio_sm, command_len * loop_cnt - 1);  
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_x, 32));

  pio_sm_put(active_state->pio, active_state->pio_sm, rx_length - 1);
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_y, 32));

  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_jmp(active_state->pio_offset));

  dma_channel_abort(active_state->dma_out);
  dma_channel_abort(active_state->dma_in);

  wizchip_pio_spi_frame_start();

  dma_channel_config out_config = dma_channel_get_default_config(active_state->dma_out);
  channel_config_set_transfer_data_size(&out_config, DMA_SIZE_8);
  channel_config_set_bswap(&out_config, true);
  channel_config_set_dreq(&out_config, pio_get_dreq(active_state->pio, active_state->pio_sm, true));
  dma_channel_configure(active_state->dma_out, &out_config, &active_state->pio->txf[active_state->pio_sm], command_buf, command_len, true); 

  dma_channel_config in_config = dma_channel_get_default_config(active_state->dma_in);
  channel_config_set_transfer_data_size(&in_config, DMA_SIZE_8);
  channel_config_set_bswap(&in_config, true);
  channel_config_set_dreq(&in_config, pio_get_dreq(active_state->pio, active_state->pio_sm, false));
  channel_config_set_write_increment(&in_config, true);
  channel_config_set_read_increment(&in_config, false);
  dma_channel_configure(active_state->dma_in, &in_config, rx, &active_state->pio->rxf[active_state->pio_sm], rx_length, true);

#if 1
  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, true);

  __compiler_memory_barrier();

  dma_channel_wait_for_finish_blocking(active_state->dma_out);
  dma_channel_wait_for_finish_blocking(active_state->dma_in);

  __compiler_memory_barrier();

  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_mov(pio_pins, pio_null)); 
  
  wizchip_pio_spi_frame_end();
  
#endif

}

void wizchip_pio_spi_write_byte(uint8_t op_code, uint16_t AddrSel, uint8_t *tx, uint16_t tx_length)  
{
  uint8_t command_buf[8] = {0,}; //[8] = {0,};
  uint16_t command_len = mk_cmd_buf(command_buf, op_code, AddrSel);
  uint32_t loop_cnt = 0;
  tx_length = tx_length + command_len;

  //command_buf[7] = 0xAB;
  //command_buf[8] = 0x02;

  //tx_length = 9;

  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
  pio_sm_set_wrap(active_state->pio, active_state->pio_sm, active_state->pio_offset, active_state->pio_offset + PIO_SPI_OFFSET_WRITE_BITS_END - 1);
  pio_sm_clear_fifos(active_state->pio, active_state->pio_sm);

#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  loop_cnt = 8;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin), (1u << active_state->pio_spi_config->data_io0_pin) );
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  loop_cnt = 4;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin),
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  loop_cnt = 2;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin) | (1u << active_state->pio_spi_config->data_io2_pin) | (1u << active_state->pio_spi_config->data_io3_pin),
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin) | (1u << active_state->pio_spi_config->data_io2_pin) | (1u << active_state->pio_spi_config->data_io3_pin));

#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
    /* @todo: Implement to use. */
#endif

#if 0
  uint8_t i = 0;

  printf("cmd_buf[%u] : ", command_len);
  for(i = 0;  i < command_len; i++)
  {
    printf("0x%x ", command_buf[i]);
  }
  printf("\n");
#endif
  pio_sm_restart(active_state->pio, active_state->pio_sm);
  pio_sm_clkdiv_restart(active_state->pio, active_state->pio_sm);
  pio_sm_put(active_state->pio, active_state->pio_sm, tx_length * loop_cnt - 1);
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_x, 32));
  pio_sm_put(active_state->pio, active_state->pio_sm, 0);
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_y, 32));
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_jmp(active_state->pio_offset));
  dma_channel_abort(active_state->dma_out);

  wizchip_pio_spi_frame_start();

  out_config = dma_channel_get_default_config(active_state->dma_out);
  channel_config_set_transfer_data_size(&out_config, DMA_SIZE_8);
  channel_config_set_bswap(&out_config, true);
  channel_config_set_dreq(&out_config, pio_get_dreq(active_state->pio, active_state->pio_sm, true));

  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, true);

#if 0
  dma_channel_configure(active_state->dma_out, &out_config, &active_state->pio->txf[active_state->pio_sm], command_buf, tx_length, true);
  dma_channel_wait_for_finish_blocking(active_state->dma_out);
#endif
  dma_channel_configure(active_state->dma_out, &out_config, &active_state->pio->txf[active_state->pio_sm], command_buf, command_len, true);
  //dma_channel_wait_for_finish_blocking(active_state->dma_out);
  dma_channel_wait_for_finish_blocking(active_state->dma_out);
  dma_channel_configure(active_state->dma_out, &out_config, &active_state->pio->txf[active_state->pio_sm], tx, tx_length - command_len, true);
  

  const uint32_t fdebug_tx_stall = 1u << (PIO_FDEBUG_TXSTALL_LSB + active_state->pio_sm);
  active_state->pio->fdebug = fdebug_tx_stall;
  //pio_sm_set_enabled(active_state->pio, active_state->pio_sm, true);
  while (!(active_state->pio->fdebug & fdebug_tx_stall))
  {
    tight_loop_contents(); // todo timeout
  }
#if 1

  __compiler_memory_barrier();
  //pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->data_io0_pin, 1, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->data_io0_pin, 2, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->data_io0_pin, 4, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
    /* @todo: Implement to use. */
#endif

  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_mov(pio_pins, pio_null)); 

  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
  wizchip_pio_spi_frame_end();
#endif

 #if 0    // origin
  __compiler_memory_barrier();
  
  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);

#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->data_io0_pin, 1, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->data_io0_pin, 2, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->pio_spi_config->data_io0_pin, 4, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
    /* @todo: Implement to use. */
#endif

  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_mov(pio_pins, pio_null)); 
  wizchip_pio_spi_frame_end();
#endif
}

static void wizchip_pio_spi_read_buffer(void)
{
  ;
}

static void wizchip_pio_spi_write_buffer(void)
{
  ;
}

wizchip_pio_spi_config_t *wizchip_default_pio_spi_config(void)
{
  return &pio_spi_default_config;
}


static uint16_t mk_cmd_buf_flash(uint8_t *pdst, uint8_t opcode, uint16_t addr)
{
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  // pdst[0] = opcode;
  // pdst[1] = (uint8_t)((addr >> 8) & 0xFF); 
  // pdst[2] = (uint8_t)((addr >> 0) & 0xFF);
  // pdst[3] = (uint8_t)((addr >> 0) & 0xFF);
  
  pdst[0] = opcode;
  pdst[1] = 0; 
  pdst[2] = 0;
  pdst[3] = 0;
  
  return 4;
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  pdst[0] = ((opcode >> 7 & 0x01) << 6) | ((opcode >> 6 & 0x01) << 4) | ((opcode >> 5 & 0x01) << 2) | ((opcode >> 4 & 0x01) << 0);
  pdst[1] = ((opcode >> 3 & 0x01) << 6) | ((opcode >> 2 & 0x01) << 4) | ((opcode >> 1 & 0x01) << 2) | ((opcode >> 0 & 0x01) << 0);
  pdst[2] = (uint8_t)((addr >> 8) & 0xFF); 
  pdst[3] = (uint8_t)((addr >> 0) & 0xFF); 

  pdst[4] = 0;

  return 4 + 1;
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  pdst[0] = ( (opcode >> 7 & 0x01) << 4 ) | ( (opcode >> 6 & 0x01) << 0 );
  pdst[1] = ( (opcode >> 5 & 0x01) << 4 ) | ( (opcode >> 4 & 0x01) << 0 );
  pdst[2] = ( (opcode >> 3 & 0x01) << 4 ) | ( (opcode >> 2 & 0x01) << 0 );
  pdst[3] = ( (opcode >> 1 & 0x01) << 4 ) | ( (opcode >> 0 & 0x01) << 0 );

  pdst[4] = ((uint8_t)(addr >> 8) & 0xFF); 
  pdst[5] = ((uint8_t)(addr >> 0) & 0xFF); 

  pdst[6] = 0;

  return 6 + 1;
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
  /* @todo: Implement to use. */
#endif
  return 0;
}

void wizchip_pio_spi_read_flash(uint8_t op_code, uint16_t AddrSel, uint8_t *rx, uint16_t rx_length) 
{
  uint8_t command_buf[16] = {0,};
  uint16_t command_len = mk_cmd_buf_flash(command_buf, op_code, AddrSel);
  uint32_t loop_cnt = 0;

  //wizchip_pio_spi_frame_start();

  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
  pio_sm_set_wrap(active_state->pio, active_state->pio_sm, active_state->pio_offset, active_state->pio_offset + PIO_SPI_OFFSET_READ_BITS_END - 1);
  //pio_sm_set_wrap(active_state->pio, active_state->pio_sm, active_state->pio_offset + PIO_SPI_OFFSET_WRITE_BITS, active_state->pio_offset + PIO_SPI_OFFSET_READ_BITS_END - 1);
  pio_sm_clear_fifos(active_state->pio, active_state->pio_sm);

#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
  loop_cnt = 8;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin), (1u << active_state->pio_spi_config->data_io0_pin));// | (1u << active_state->pio_spi_config->data_io1_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
  loop_cnt = 4;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin),
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
  loop_cnt = 2;
  pio_sm_set_pindirs_with_mask(active_state->pio,
                                active_state->pio_sm,
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin) | (1u << active_state->pio_spi_config->data_io2_pin) | (1u << active_state->pio_spi_config->data_io3_pin),
                                (1u << active_state->pio_spi_config->data_io0_pin) | (1u << active_state->pio_spi_config->data_io1_pin) | (1u << active_state->pio_spi_config->data_io2_pin) | (1u << active_state->pio_spi_config->data_io3_pin));

#elif (_WIZCHIP_QSPI_MODE_ == QSPI_OCTAL_MODE)
    /* @todo: Implement to use. */
#endif

  pio_sm_restart(active_state->pio, active_state->pio_sm);
  pio_sm_clkdiv_restart(active_state->pio, active_state->pio_sm);

  pio_sm_put(active_state->pio, active_state->pio_sm, command_len * loop_cnt - 1);  
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_x, 32));

  pio_sm_put(active_state->pio, active_state->pio_sm, rx_length * loop_cnt - 1);
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_y, 32));

  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_jmp(active_state->pio_offset));

  dma_channel_abort(active_state->dma_out);
  dma_channel_abort(active_state->dma_in);

  wizchip_pio_spi_frame_start();

  dma_channel_config out_config = dma_channel_get_default_config(active_state->dma_out);
  channel_config_set_transfer_data_size(&out_config, DMA_SIZE_8);
  channel_config_set_bswap(&out_config, true);
  channel_config_set_dreq(&out_config, pio_get_dreq(active_state->pio, active_state->pio_sm, true));
  dma_channel_configure(active_state->dma_out, &out_config, &active_state->pio->txf[active_state->pio_sm], command_buf, command_len, true); 

  dma_channel_config in_config = dma_channel_get_default_config(active_state->dma_in);
  channel_config_set_transfer_data_size(&in_config, DMA_SIZE_8);
  channel_config_set_bswap(&in_config, true);
  channel_config_set_dreq(&in_config, pio_get_dreq(active_state->pio, active_state->pio_sm, false));
  channel_config_set_write_increment(&in_config, true);
  channel_config_set_read_increment(&in_config, false);
  dma_channel_configure(active_state->dma_in, &in_config, rx, &active_state->pio->rxf[active_state->pio_sm], rx_length, true);

#if 1
  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, true);

  __compiler_memory_barrier();

  dma_channel_wait_for_finish_blocking(active_state->dma_out);
  dma_channel_wait_for_finish_blocking(active_state->dma_in);

  __compiler_memory_barrier();

  pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
  pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_mov(pio_pins, pio_null)); 
  
  wizchip_pio_spi_frame_end();
  
#endif

}