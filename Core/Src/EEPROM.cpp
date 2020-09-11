/*
 * EEPROM.cpp
 *
 *  Created on: 9 сент. 2020 г.
 *      Author: alekseev
 */

#include "EEPROM.hpp"

uint16_t EEPROM_t::varOffset=0;

extern I2C_HandleTypeDef hi2c1;


EEPROM_t EEPROM = EEPROM_t();
template <typename T>
inline bool data_put(uint16_t addr, T vol) {

#define PAGE_LEN 64

	uint8_t len = sizeof(T);

	uint32_t cnt = (
			((addr % PAGE_LEN + len) > PAGE_LEN) ?
					(PAGE_LEN - (addr % PAGE_LEN)) : len);
	uint32_t ByteWrited = 0;

	while (len) {

		struct
			__packed {
				uint16_t addr;
				T vol;
			} buf = { byteOrder::reverse(addr), vol };
			//uint8_t* pbuf =reinterpret_cast<uint8_t*>(&buf);
			//memcpy( &buf[EE_ADR_SIZE], EE_DATA+ByteWrited, LEN );

			if (HAL_OK
					!= HAL_I2C_Master_Transmit(&hi2c1, 0x50,
							reinterpret_cast<uint8_t*>(&buf),
							cnt + sizeof(addr), 100)) {
				if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
					Error_Handler()
					;
				}
				return false;
			}

			len -= cnt;
			addr += cnt;
			ByteWrited += cnt;
			cnt = (len / PAGE_LEN) ? PAGE_LEN : len;

			delay(3);
		}

		return true;
	}


template <typename T >
inline bool data_get(T *bf, uint16_t addr,	uint16_t len)  {

		byteOrder::reverse(addr);

		if (HAL_OK
				!= HAL_I2C_Master_Transmit(&hi2c1, 0x50,
						reinterpret_cast<uint8_t *>(&addr), sizeof(addr),
						100)) {
			if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
				Error_Handler()
				;

			}
			return false;
		} else if (HAL_OK
				!= HAL_I2C_Master_Receive(&hi2c1, 0x50,
						reinterpret_cast<uint8_t *>(bf), len, 100)) {
			if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
				Error_Handler()
				;
			}
			return false;

		}
		return true;
	}
template <>
bool data_get<uint8_t>(uint8_t *bf, uint16_t addr,	uint16_t len){
	return data_get(bf,addr,len);
}

