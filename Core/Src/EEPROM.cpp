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

template<typename T,int LENAREA>
T EEPROM_t::VarEE<T,LENAREA>::_get(bool force) const{
	if (value != 0 && !force)
		return value;
	T ret = static_cast<T>(0xFFFFFFFF);
	T buf[LENAREA / sizeof(T)];

	while (!data_get(buf, addr, LENAREA)) {
	};

	ret = buf[0];
	for (volatile uint16_t i = 1; i < LENAREA / sizeof(T); i++) {
		ret ^= buf[i];
	}

	return ret;
}
template<typename T,int LENAREA>
T EEPROM_t::VarEE<T,LENAREA>::get( ) const {
			T res = _get();

			if (copy1 != NULL) {
				T res1 = copy1->get();
				if (res == res1)
					return res;
				else if (copy2 != NULL) {
					T res2 = copy1->get();
					if (res == res2)
						return res;
					else if (res1 == res2 && res < res1)
						return res;
					else
						return (res1 < res2) ? res1 : res2;
				}

			}
			return res;

		}


template<typename T,int LENAREA>
T EEPROM_t::VarEE<T,LENAREA>::operator()()  {
	return get();
}

template class EEPROM_t::VarEE <uint8_t,32>;
