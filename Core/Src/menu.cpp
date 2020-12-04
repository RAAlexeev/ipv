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


uint8_t ServiceMenu::curCH=0;


MenuItem  ServiceMenu::items[]={
			 MenuItem(
					 [](){ return  (float32_t)EEPROM.kA4_20()/10;}
			 	 	,[](float32_t v){ EEPROM.kA4_20.set(round(v*10)); }
			 	 	,0,0,0,-100
			 	 	 ),
			 MenuItem(
					 [](){ return  (float32_t)EEPROM.kB4_20()/10;}
			 	 	,[](float32_t v){ EEPROM.kB4_20.set(round(v*10)); }
			 	 	,7,0,0,-100
			 ),
			 MenuItem(
					 [](){ return  (float32_t)EEPROM.relayDelay()/10;}
			 	 	,[](float32_t v){ EEPROM.relayDelay.set(round(v*10)); }
			 	 	,6,10,0,100
			 )
};

ServiceMenu serviceMenu=ServiceMenu(sizeof(ServiceMenu::items)/sizeof(MenuItem));

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
					 [](){ return (float32_t)EEPROM.porog11()/10; },
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
	SC39_show(getCurentItem()->getValue( curIndex < 3 ), digPos);

}
void ServiceMenu::display(){

		SC39_show(getCurentItem()->getValue(false), 0);

}

