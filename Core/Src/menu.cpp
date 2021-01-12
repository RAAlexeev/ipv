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






MenuItem  ServiceMenu::items[]={
			 MenuItem(
					 [](){ return EEPROM.K1()/10.f;},
	 	 	 	 	 [](float32_t v){ EEPROM.K1.set(round(v*10)); }
					 ,2,0,-100,100,1
			 ),
			 MenuItem(
					 [](){ return EEPROM.range1()/10.f;},
	 	 	 	 	 [](float32_t v){ EEPROM.range1.set(round(v*10)); }
					 ,1,0,0,99,1
			 ),
			 MenuItem(
					 [](){ return EEPROM.K2()/10.f;},
	 	 	 	 	 [](float32_t v){ EEPROM.K2.set(round(v*10)); }
					 ,2,0,-100,100,2
			 ),
			 MenuItem(
					 [](){ return EEPROM.range2()/10.f;},
	 	 	 	 	 [](float32_t v){ EEPROM.range2.set(round(v*10)); }
					 ,1,0,0,99,2
			 ),
			 MenuItem(
					 [](){ return  EEPROM.relayDelay()/10.f;}
			 	 	,[](float32_t v){ EEPROM.relayDelay.set(round(v*10)); }
			 	 	,0,10,0,100,0
			 ),
			 MenuItem(
					 [](){ return  0.f;}
			 	 	,[](float32_t v){if(round(v)==35){ EEPROM.reset(); NVIC_SystemReset();}}
			 	 	,3,0,0,100,0
			 ),
			 MenuItem(
					 [](){ return  EEPROM.kA4_20()/10.f;}
			 	 	,[](float32_t v){ EEPROM.kA4_20.set(round(v*10)); }
			 	 	,4,0,-100,100,0
			 	 	 ),
			 MenuItem(
					 [](){ return  EEPROM.kB4_20()/10.f;}
			 	 	,[](float32_t v){ EEPROM.kB4_20.set(round(v*10)); }
			 	 	,5,0,-100,100,0
			 ),

};

ServiceMenu serviceMenu = ServiceMenu(sizeof(ServiceMenu::items)/sizeof(MenuItem));

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
					 [](){ return (float32_t)EEPROM.porog21()/10;},
			 	 	 [](float32_t v){ EEPROM.porog21.set(round(v*10)); }
					 ,4
			 ),
			 MenuItem(
					 [](){ return (float32_t)EEPROM.porog22()/10;},
			 	 	 [](float32_t v){ EEPROM.porog22.set(v*10); }
					 ,5
			 ),

	 };

Menu menu=Menu(sizeof(Menu::items)/sizeof(MenuItem));



void Menu::display(){

	SC39_show(getCurentItem()->getValue( curIndex <= 3 ), digPos);


}
void ServiceMenu::display(){

	switch(items[curIndex].ch){
	case 1:
		led(7,true);
		led(6,false);
	break;
	case 2:
		led(7,false);
		led(6,true);
	break;
	default:
		led(7,false);
		led(6,false);
	};

	led(items[curIndex].diode,true);
	SC39_show(getCurentItem()->getValue(false), 0);
	static  uint8_t delay = 0,slow=0;
	if(clampMinus){
			if(++slow > 4 || slow&1  )minus();
			delay=0;
			return;
	}else{

		if(clampPlus){
			if(++slow > 4 || slow&1 )plus();
			delay=0;
			return;
		}
		else
			slow=0;
	}

			if( delay++ > 5){
				for(uint8_t i = 0; i < maxIndex; i++)
					items[i].saveValue();
				delay=0;
			}

}

