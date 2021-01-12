/*
 * leds.cpp
 *
 *  Created on: 13 окт. 2020 г.
 *      Author: alekseev
 */
#include "main.h"
 extern volatile uint32_t port_IO[3];
static  uint16_t ledIsOn=0;
static  uint16_t delayMenuShow = 40;


void led(uint8_t n, bool on,  bool resetDelayMenuShow){
	if (n >= 8)
		for(;;);
	n=7-n;
//	uint16_t p = 0;

	if(on){
	//	p |= 1<<n;
		ledIsOn|=1<<n;

	}else{
	//	p &= ~(1<<n);
		ledIsOn &= ~(1<<n);
	}
	if(resetDelayMenuShow){
		delayMenuShow = 40;
		port_IO[2] = ((( 0x200 | 0xFF ) << 16) |  ledIsOn ) | GPIO_BSRR_BS10;
	}

}

void scale(uint16_t percent, uint8_t mask,void delay(uint16_t ms) = NULL){
	static bool  blink;
	if(delayMenuShow){
		--delayMenuShow;
		if(blink)
			port_IO[2] = ((( 0x200 | 0xFF ) << 16) |  ledIsOn | mask) | GPIO_BSRR_BS10;
		else
			port_IO[2] = ((( 0x200 | 0xFF | mask ) << 16) |  ledIsOn ) | GPIO_BSRR_BS10;
		blink =!blink;
		return;
	};

	//uint32_t sPort_IO=port_IO[2];
/*	for(uint16_t i=0;i<0xF;i++){
		port_IO[2] = (GPIO_BSRR_BS10 << 16)  & ~GPIO_BSRR_BS10;
		if(delay)delay(i);//osDelay(1);
		port_IO[2] =sPort_IO;
	}*/
	uint32_t p;
	uint16_t diodes=percent*8/100;
//	if(bleenk){

	p = (uint8_t)(0xFF << (8-(diodes<=8?diodes:8)));
	//}
	//else
	//	p =( (port_IO[2]&(~ledIsOn)) | (ledIsOn<<16));

if(blink)
		 port_IO[2] = ( p|mask) | ((0x200 | ( 0xFF & (~p))) << 16) | GPIO_BSRR_BS10;
else
	port_IO[2] =  (( p & (~mask) ) | ( (( 0x200| mask)  | ( 0xFF & (~p))) << 16) ) | GPIO_BSRR_BS10;
blink =!blink;
/*	for(uint16_t i=0xF;i;i--){
		port_IO[2] = (GPIO_BSRR_BS10<<16)  & ~GPIO_BSRR_BS10;
		if(delay)delay(i);//osDelay(1);
		port_IO[2] =sPort_IO;
	}*/
}



