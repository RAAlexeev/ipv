/*
 * myUtils.hpp
 *
 *  Created on: 27 но€б. 2019 г.
 *      Author: alekseev
 */

#ifndef SRC_MYUTILS_HPP_
#define SRC_MYUTILS_HPP_
#include "stm32f4xx_hal.h"
#include "main.h"

namespace byteOrder {

   inline uint16_t  htons(uint16_t *data);
   inline uint32_t htonl(uint32_t *data);

	template<int size>	void byteSwap_(void* p);

	template<> void byteSwap_<1>(void* p);

	template<> void byteSwap_<2>(void* p);
	template<> void byteSwap_<4>(void* p);
	template<class T> T reverse(T &swapIt){
		byteSwap_<sizeof(T)>(&swapIt);
	    return swapIt;
	}

}

namespace myUtils{
	inline void ITM_SendStr(const  char *s ){
		while(*s){
		ITM_SendChar(*s++ );
		}
	}
}

namespace uartMBparam{
typedef union{	mbAddrUartParam_t p;

uint16_t raw;
}param_t ;
extern  param_t  param;
		uint8_t getAddr();
		uint32_t getParity();
		uint32_t getStopBit();
		uint8_t setAddr(uint8_t val);
		uint8_t setParity(uint8_t val );
		uint8_t setStopBit(uint8_t val);
}
#endif /* SRC_MYUTILS_HPP_ */
