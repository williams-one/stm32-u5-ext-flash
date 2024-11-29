#include "test_flash.h"

#include <stdio.h>

extern XSPI_HandleTypeDef hxspi1;

void print_buffer(char const* label, uint8_t const* buffer, uint32_t size) {
  printf("%s = [ ", label);
  for (uint32_t i = 0; i < size; ++i)
    printf("%d ", buffer[i]);
  printf("]\n");
}

#if defined(INDIRECT_MODE_STR)

void read_from_external_flash(uint32_t address, uint8_t* data, uint32_t size) {
  uint32_t offset = (uint32_t)address - 0xA0000000;

  int32_t status = MX66UW1G45G_ReadSTR(&hxspi1, MX66UW1G45G_SPI_MODE, MX66UW1G45G_3BYTES_SIZE, data, offset, size);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_ReadSTR failed: error %ld\n", status);
}

void erase_external_flash(uint32_t address) {
  uint32_t offset = (uint32_t)address - 0xA0000000;

  int32_t status = MX66UW1G45G_WriteEnable(&hxspi1, MX66UW1G45G_SPI_MODE, MX66UW1G45G_STR_TRANSFER);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_WriteEnable failed: error %ld\n", status);


  status = MX66UW1G45G_BlockErase(&hxspi1, MX66UW1G45G_SPI_MODE, MX66UW1G45G_STR_TRANSFER,
                                  MX66UW1G45G_4BYTES_SIZE, offset, MX66UW1G45G_ERASE_4K);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_BlockErase failed: error %ld\n", status);
}

void write_to_external_flash(uint32_t address, uint8_t* data, uint32_t size) {
  uint32_t offset = (uint32_t)address - 0xA0000000;

  int32_t status = MX66UW1G45G_WriteEnable(&hxspi1, MX66UW1G45G_SPI_MODE, MX66UW1G45G_STR_TRANSFER);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_WriteEnable failed: error %ld\n", status);

  status = MX66UW1G45G_PageProgram(&hxspi1, MX66UW1G45G_SPI_MODE, MX66UW1G45G_4BYTES_SIZE, data, offset, size);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_PageProgramSTR failed: error %ld\n", status);
}

#elif defined(INDIRECT_MODE_DTR)

void read_from_external_flash(uint32_t address, uint8_t* data, uint32_t size) {
  uint32_t offset = (uint32_t)address - 0xA0000000;

  int32_t status = MX66UW1G45G_ReadDTR(&hxspi1, data, offset, size);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_ReadDTR failed: error %ld\n", status);
}

void erase_external_flash(uint32_t address) {
  uint32_t offset = (uint32_t)address - 0xA0000000;

  int32_t status = MX66UW1G45G_WriteEnable(&hxspi1, MX66UW1G45G_OPI_MODE, MX66UW1G45G_DTR_TRANSFER);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_WriteEnable failed: error %ld\n", status);


  status = MX66UW1G45G_BlockErase(&hxspi1, MX66UW1G45G_OPI_MODE, MX66UW1G45G_DTR_TRANSFER,
                                  MX66UW1G45G_4BYTES_SIZE, offset, MX66UW1G45G_ERASE_4K);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_BlockErase failed: error %ld\n", status);
}

void write_to_external_flash(uint32_t address, uint8_t* data, uint32_t size) {
  uint32_t offset = (uint32_t)address - 0xA0000000;

  int32_t status = MX66UW1G45G_WriteEnable(&hxspi1, MX66UW1G45G_OPI_MODE, MX66UW1G45G_DTR_TRANSFER);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_WriteEnable failed: error %ld\n", status);

  status = MX66UW1G45G_PageProgramDTR(&hxspi1, data, offset, size);
  if (status != MX66UW1G45G_OK)
    printf("MX66UW1G45G_PageProgram failed: error %ld\n", status);
}

#endif
