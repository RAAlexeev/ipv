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


MenuItem  Menu::items[]={
			 MenuItem(
					 [](){ return  SignalChenal::getInstance(ADC1)->getVelocity();}
			 ,NULL
			 ,2
			 ),
			 MenuItem(
					 [](){ return SignalChenal::getInstance(ADC1)->getAcceleration(); }
			 ,NULL
			 ,3
			 ),

			 MenuItem(
					 [](){ return SignalChenal::getInstance(ADC2)->getVelocity(); }
			 ,NULL
			 ,2
			 ),
			 MenuItem(
					 [](){ return SignalChenal::getInstance(ADC2)->getAcceleration(); }
			 ,NULL
			 ,3
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog11.get()/10; },
					 [](float32_t v){ EEPROM.porog11.set(round(v*10)); }
					 ,4
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog12()/10;},
			 	 	 [](float32_t v){ EEPROM.porog12.set(round(v*10)); }
					 ,5
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.range1()/10;},
	 	 	 	 	 [](float32_t v){ EEPROM.range1.set(round(v*10)); }
					 ,1
			 ),

			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog21()/10;},
			 	 	 [](float32_t v){ EEPROM.porog21.set(round(v*10)); }
					 ,4
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog22()/10;},
			 	 	 [](float32_t v){ EEPROM.porog22.set(v*10); }
					 ,5
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.range2()/10;},
			 	 	 [](float32_t v){ EEPROM.range2.set(round(v*10)); }
					 ,1
			 ),
	 };

Menu menu=Menu(sizeof(Menu::items)/sizeof(MenuItem));

void Menu::display(){

		SC39_show(getCurentItem()->getValue( curIndex < 3 ),digPos);

}
