/*
 * FlashLoader.cpp
 *
 *  Created on: 9 дек. 2020 г.
 *      Author: alekseev
 */
#include "main.h"
#define  FlashLoaderKey (0xAAAA)

uint32_t sFlashLoaderKey __attribute__((section(".noinit")));

bool isFlashLoaderKey(){
	return FlashLoaderKey==sFlashLoaderKey;
}

bool parseFlashLoaderReq(uint8_t * buf){

	return (buf[1]==0xFF||buf[1]==0xEF);
}

void rebootToFlashLoader( void ){
	sFlashLoaderKey=FlashLoaderKey;
	HAL_NVIC_SystemReset();
}

void flashLoaderStartIfNeed(){
	volatile uint32_t addr = 0x1FFF0000;
void(*bootLoader)(void) =  (void (*)(void))(*(( uint32_t *)(addr + 4)));


	if(isFlashLoaderKey()){
		__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
		sFlashLoaderKey=0;
		__set_MSP(*(uint32_t *)addr);
		bootLoader();
		while( 1 ){};
	}


}
