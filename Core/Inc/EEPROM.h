#ifndef EEPROM_H
#define EEPROM_H

  void EE_read(uint16_t addr, void * pData, uint16_t len );
  void EE_write(uint16_t addr, void * pData, uint16_t len );
  
#endif