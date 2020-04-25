/*
  EEPROM like API that uses Arduino Zero's flash memory.
  Written by A. Christian

  Copyright (c) 2015-2016 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "FlashAsEEPROM_yn.h"

FlashStorage(eeprom_storage, EEPROM_EMULATION);

uint32_t read_unaligned_uint32(void *data)
{
  volatile union {
    uint32_t u32;
    uint8_t u8[4];
  } res;
  
  const uint8_t *d =(const uint8_t *)data;
  res.u8[0] = d[0];
  res.u8[1] = d[1];
  res.u8[2] = d[2];
  res.u8[3] = d[3];
  uint32_t temp=res.u32;
  return temp;
}

EEPROMClass::EEPROMClass(void) : _initialized(false), _dirty(false) {
  // Empty
}

EEPROMpacket EEPROMClass::read(int address)
{
  if (!_initialized) init();
  _eeprom = eeprom_storage.read();
  return _eeprom.res.packet[address];
}

void EEPROMClass::update(int address, uint8_t value)
{
  if (!_initialized) init();
  if (_eeprom.res.data[address] != value) {
    _dirty = true;
    _eeprom.res.data[address] = value;
  }
}

void EEPROMClass::write(int address, uint8_t value)
{
  update(address, value);
}

void EEPROMClass::convert(int num,EEPROMpacket *packet) {
if (!_initialized) init();
   _dirty = true;
  uint32_t size= (sizeof(EEPROMpacket) + 3) / 4;
  uint32_t blk_num=size;
//uint32_t *dst_addr = (uint32_t *)&(_eeprom.res.data[num*8]);
uint8_t *src_addr = (uint8_t *)packet;

  while (size) {
    
    uint32_t i;
    for (i = 0; i < 16 && size; i++) {
      
       uint32_t a= read_unaligned_uint32(src_addr);
       _eeprom.res.data[(num*blk_num)+i]=a;
      src_addr += 4;
      size--;
    }
  }
}

void EEPROMClass::init()
{
  _eeprom = eeprom_storage.read();
  if (!_eeprom.valid) {
    memset(_eeprom.res.data, 0xFF, EEPROM_EMULATION_SIZE);
  }
  _initialized = true;
}

bool EEPROMClass::isValid()
{
  if (!_initialized) init();
  return _eeprom.valid;
}

void EEPROMClass::commit()
{
  if (!_initialized) init();
  if (_dirty) {
    _eeprom.valid = true;
    eeprom_storage.write(_eeprom);
  }
}

EEPROMClass EEPROM;
