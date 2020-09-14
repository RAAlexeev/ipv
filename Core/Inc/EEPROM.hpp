/*
 * EEPROM.hpp
 *
 *  Created on: 14 дек. 2019 г.
 *      Author: alekseev
 */

#ifndef EEPROM_HPP_
#define EEPROM_HPP_

#include "main.h"
#include"myUtils.hpp"

extern I2C_HandleTypeDef hi2c1;
extern void delay(uint32_t ms);

template<typename T>
bool data_get(T *bf, uint16_t addr, uint16_t len = sizeof(T));
template<typename T>
bool data_put(uint16_t addr, T vol);

extern class EEPROM_t {

	static uint16_t varOffset;
public:
	template<typename T> class Jornal {
#define SIZE (256)
		uint16_t offset;
		uint16_t maxPos;
		uint16_t pos = 0;
		uint16_t getPos() {
			return offset + pos * sizeof(T);
		}
	public:
		Jornal() :
				offset(varOffset + 1), maxPos(SIZE / 8 - varOffset) {

			varOffset += SIZE / 8 - varOffset;
		}
		;
		bool put(T item) {
			if (getPos() < SIZE)
				pos++;
			else
				pos = 0;
			return data_put(getPos(), item);
		}
		bool get(uint16_t n, T *item) {
			if (n * sizeof(T) > SIZE)
				return false;
			return data_get(item, n * sizeof(T), sizeof(T));
		}
		bool getLast(T *item) {

			return data_get(item, getPos(), sizeof(T));
		}

	};

	template<typename T = uint16_t, int LENAREA = 32>
	class  VarEE {

		// T value = 0;

		const uint16_t addr;
		uint16_t _index;
		VarEE *copy1, *copy2;

		T _get(bool force = false)const;
		void _set(T _val) {

			if ((value == _val) && (value || get() == _val))
				return;
			do {

				T elem_i;
				T val = _val;
				if (++_index >= (LENAREA / sizeof(T)))
					_index = 0;

				if (data_get(&elem_i, addr + _index * sizeof(T))) {
					val ^= (_get() ^ elem_i);
					data_put(addr + _index * sizeof(T), val);
				}
				// tested

			} while (_get(true) != _val);
			value = _val;
		}

		T value = 0;
	public:
		VarEE(VarEE<T, LENAREA> *_copy1 = NULL,
				VarEE<T, LENAREA> *_copy2 = NULL) :
				addr(varOffset), copy1(_copy1), copy2(_copy2) {
			varOffset += LENAREA;
		}

		T set(T _val) {
			_set(_val);
			if (copy1 != NULL)
				copy1->_set(_val);
			if (copy2 != NULL)
				copy2->_set(_val);
			return _val;
		}

		T get() const ;


		T operator()();

	};

public:

	VarEE<uint8_t> porog11 = VarEE<uint8_t>();
	VarEE<uint8_t> porog12 = VarEE<uint8_t>();
	VarEE<uint8_t> porog21 = VarEE<uint8_t>();
	VarEE<uint8_t> porog22 = VarEE<uint8_t>();
	VarEE<uint8_t> range1 = VarEE<uint8_t>();
	VarEE<uint8_t> range2 = VarEE<uint8_t>();

	inline void reset(void) {
		porog11.set(0);
	}
	//	bool ifThirstRunInit();
	/*{
	 static uint16_t buf = 0;
	 if (buf == 0xAAAB)
	 return false;
	 while (!data_get(&buf, 0)) {
	 ;
	 };
	 if (buf == 0xAAAB)
	 return false;

	 reset();

	 buf = 0xAAAB;
	 while (!data_put(0, buf)) {
	 ;
	 }

	 return true;
	 }
	 */
} EEPROM;

#endif /* EEPROM_HPP_ */
