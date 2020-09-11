#include"myUtils.hpp"

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
