/*
 * eeprom.c
 *
 *  Created on: Nov 17, 2024
 *      Author: cav
 */

#include "eeprom.h"



uint16_t eeprom_length() {
  return DATA_EEPROM_END - DATA_EEPROM_BASE + 1;
}

uint32_t eeprom_base_address(void) {
  return DATA_EEPROM_BASE;
}

uint8_t eeprom_read_byte(const uint32_t pos)
{
#if defined(DATA_EEPROM_BASE)
  __IO uint8_t data = 0;

  if (pos <= (DATA_EEPROM_END - DATA_EEPROM_BASE))
  {
    /* with actual EEPROM, pos is a relative address */
    data = *(__IO uint8_t *)(DATA_EEPROM_BASE + pos);
  }

  return (uint8_t)data;
#else

  return 0;
#endif /* _EEPROM_BASE */
}

void eeprom_write_byte(const uint32_t pos, const uint8_t value)
{
#if defined(DATA_EEPROM_BASE)

  /* with actual EEPROM, pos is a relative address */
  if (pos <= (DATA_EEPROM_END - DATA_EEPROM_BASE))
  {
    if (HAL_FLASHEx_DATAEEPROM_Unlock() == HAL_OK)
    {
      HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (pos + DATA_EEPROM_BASE), value);
      HAL_FLASHEx_DATAEEPROM_Lock();
    }
  }

#endif /* _EEPROM_BASE */
}


uint32_t eeprom_read_word(const uint32_t pos)
{
#if defined(DATA_EEPROM_BASE)
  __IO uint32_t data = 0;

  if (pos <= (DATA_EEPROM_END - DATA_EEPROM_BASE - 4))
  {
    /* with actual EEPROM, pos is a relative address */
    data = *(__IO uint32_t *)(DATA_EEPROM_BASE + pos);
  }

  return (uint32_t)data;
#else

  return 0;
#endif /* _EEPROM_BASE */
}

void eeprom_write_word(const uint32_t pos, const uint32_t value)
{
#if defined(DATA_EEPROM_BASE)

  /* with actual EEPROM, pos is a relative address */
  if (pos <= (DATA_EEPROM_END - DATA_EEPROM_BASE - 4))
  {
    if (HAL_FLASHEx_DATAEEPROM_Unlock() == HAL_OK)
    {
      // HAL_FLASHEx_DATAEEPROM_Erase(pos + DATA_EEPROM_BASE);
      HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, (pos + DATA_EEPROM_BASE), value);
      HAL_FLASHEx_DATAEEPROM_Lock();
    }
  }

#endif /* _EEPROM_BASE */
}
