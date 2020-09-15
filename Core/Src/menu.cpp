/*
 * menu.cpp
 *
 *  Created on: 9 сент. 2020 г.
 *      Author: alekseev
 */
#include "menu.hpp"
#include "EEPROM.hpp"
#include "integrator.hpp"
#include "SC39-11driver.h"
Screen screen;

uint16_t varOffset = 0;
static float32_t  testVelocity (float32_t v,float32_t porog1, float32_t porog2){
	static uint32_t delay = 0XF;
					if(delay)delay--;
					else{ if(porog1 != 0 &&( v > porog1 )){
					 		  HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET);
					 	 }else if( v < (porog1 - v/10)){
					 		 	 HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_RESET);
					 		 	}

					 	 if(porog2 != 0 &&( v > porog2 )){
							 	 HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);
						 }else if( v < (porog2 - v/10)){
						 		 HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);
						 	 	}
					}return v;
};

MenuItem  Menu::items[]={
			 MenuItem(
					 [](){
					 	 return testVelocity( SignalChenal::getInstance(ADC1)->getVelocity(), (float32_t)EEPROM.porog11.get()/10, (float32_t)EEPROM.porog12.get()/10 );
			 }
			 ),
			 MenuItem(
					 [](){ return SignalChenal::getInstance(ADC1)->getAcceleration(); }
			 	 	 	 ),
			 MenuItem(

			 ),
			 MenuItem(
					 [](){
					 	 return testVelocity( SignalChenal::getInstance(ADC2)->getVelocity(), (float32_t)EEPROM.porog21.get()/10, (float32_t)EEPROM.porog22.get()/10 );
			 	 	 	 }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog11.get()/10; },
					 [](float32_t v){ EEPROM.porog11.set(round(v*10)); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog12()/10;},
			 	 	 [](float32_t v){ EEPROM.porog12.set(round(v*10)); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.range1()/10;},
	 	 	 	 	 [](float32_t v){ EEPROM.range1.set(round(v*10)); }
			 ),

			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog21()/10;},
			 	 	 [](float32_t v){ EEPROM.porog21.set(round(v*10)); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog22()/10;},
			 	 	 [](float32_t v){ EEPROM.porog22.set(v*10); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.range2()/10;},
			 	 	 [](float32_t v){ EEPROM.range2.set(round(v*10)); }
			 ),
	 };

Menu menu=Menu(sizeof(Menu::items)/sizeof(MenuItem));

void Menu::display(){

		SC39_show(getCurentItem()->getValue(curIndex<3? true:false ),digPos);

}
