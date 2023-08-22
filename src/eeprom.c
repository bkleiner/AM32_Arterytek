#include "eeprom.h"

#include "targets.h"

void eeprom_write(const uint8_t *data, const uint32_t length) {
  flash_unlock();
  flash_sector_erase(EEPROM_START_ADD);

  const uint32_t *ptr = (const uint32_t *)data;
  for (uint32_t i = 0; i < (length / 4); i++) {
    flash_word_program(EEPROM_START_ADD + i, ptr[i]);
    flash_flag_clear(FLASH_PROGRAM_ERROR | FLASH_EPP_ERROR | FLASH_OPERATE_DONE);
  }

  flash_lock();
}

void eeprom_read(uint8_t *data, const uint32_t length) {
  const uint8_t *ptr = (uint8_t *)(EEPROM_START_ADD);
  for (uint32_t i = 0; i < length; i++) {
    data[i] = ptr[i];
  }
}