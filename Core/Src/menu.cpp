/*
 * menu.cpp
 *
 *  Created on: 9 сент. 2020 г.
 *      Author: alekseev
 */
#include "menu.hpp"
#include "EEPROM.hpp"
Screen screen;
Menu menu;
uint16_t varOffset = 2;

	 MenuItem Menu::items[]={
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog11(); }
			 ),
	 };
