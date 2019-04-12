#pragma once

#include "driver/spi_master.h"

#define PIN_NUM_MISO 19  // MISO
#define PIN_NUM_MOSI 18  // MOSI
#define PIN_NUM_CLK  5   // SCK

void spi_init_bus(void);
void spi_send(uint8_t *data, spi_device_handle_t *spi);
void spi_read(uint8_t *data, spi_device_handle_t *spi);
void spi_read_addressed(uint8_t addr, uint8_t *data, spi_device_handle_t *spi);
void spi_send_addressed(uint8_t addr, uint8_t *data, spi_device_handle_t *spi);

