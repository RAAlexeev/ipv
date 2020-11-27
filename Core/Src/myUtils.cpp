#include"myUtils.hpp"
#include "EEPROM.hpp"
namespace byteOrder {
   inline uint16_t  htons(uint16_t *data){
    	uint16_t buf;
		buf = *data<<8;
		buf |= *data>>8;
		return *data=buf;
		}
   inline uint32_t htonl(uint32_t *data){
	   uint32_t buf;
		buf = *data<<24;
		buf |= (*data&0xFF00)<<8;
		buf |= (*data&0xFF0000)>>8;
		buf |= *data>>24;
		return *data=buf;
		}

	template<int size>	void byteSwap_(void* p);

	template<> void byteSwap_<1>(void* p) { /* do nothing */ }

	template<> void byteSwap_<2>(void* p){
		htons(static_cast<uint16_t*> (p));
	}

	template<> void byteSwap_<4>(void* p){
		htonl(static_cast<uint32_t*> (p));
	}
}
namespace uartMBparam{
param_t param ;
	uint8_t getAddr(){
			param.raw=EEPROM.uartParam_mbAddr();
		  	  return param.p.addr;
		  }

	  uint32_t getParity(){
		  param.raw=EEPROM.uartParam_mbAddr();
		  switch( param.p.parity ){
		  	  case 1:return UART_PARITY_EVEN;
		  	  case 2:return UART_PARITY_ODD;
		  	  default:return UART_PARITY_NONE;
		  }
	  }
	uint32_t getStopBit(){

				param.raw=EEPROM.uartParam_mbAddr();
				  switch( param.p.parity ){
				  	  case 2:return UART_STOPBITS_2;
				  	  default:return UART_STOPBITS_1;
				  }


	}
	uint8_t setAddr(uint8_t addr){
		if(addr==0) return param.p.addr;
		if(addr > 247)addr=247;
		param.p.addr = addr;
		EEPROM.uartParam_mbAddr.set(param.raw);
		return addr;
	}
	uint8_t setParity(uint8_t parity){
		if(parity > 2) return param.p.parity;
		param.p.parity = parity;
		EEPROM.uartParam_mbAddr.set(param.raw);
		return parity;
	}

	uint8_t  setStopBit(uint8_t stopBit){
		if(stopBit > 2 || stopBit == 0) return param.p.stopBit;
		param.p.stopBit = stopBit;
		EEPROM.uartParam_mbAddr.set(param.raw);
		return stopBit;
	}
}
