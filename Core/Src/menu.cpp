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



	 MenuItem const Menu::items[]={
			 MenuItem(
					 [](){ return SignalChenal::getInstance(ADC1)->getVelocity(); }
			 ),
			 MenuItem(
					 [](){ return SignalChenal::getInstance(ADC1)->getAcceleration(); }
			 ),
			 MenuItem(
					 [](){ return SignalChenal::getInstance(ADC2)->getVelocity(); }
			 ),
			 MenuItem(
					 [](){ return SignalChenal::getInstance(ADC2)->getAcceleration(); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog11.get(); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog12(); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.range1(); }
			 ),

			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog21(); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog22(); }
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.range2(); }
			 ),
	 };

Menu menu=Menu(sizeof(Menu::items));
void Menu::display(){

		SC39_show(getCurentItem()->getValue());

}
