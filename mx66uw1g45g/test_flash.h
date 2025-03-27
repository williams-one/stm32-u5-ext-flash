#pragma once

#include <stdint.h>
#include "mx66uw1g45g.h"

#define MEMORY_MAPPED_MODE
// #define INDIRECT_MODE_SPI
// #define INDIRECT_MODE_OPI_STR
// #define INDIRECT_MODE_OPI_DTR

void print_buffer(char const* label, uint8_t const* buffer, uint32_t size);

void read_flash_id();
void read_from_external_flash(uint32_t address, uint8_t* data, uint32_t size);
void erase_external_flash(uint32_t address);
void write_to_external_flash(uint32_t address, uint8_t* data, uint32_t size);
