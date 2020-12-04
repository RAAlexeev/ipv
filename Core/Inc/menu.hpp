/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H__
#define __MENU_H__
#include "main.h"
#include "arm_math.h"
#include "myutils.hpp"
extern void led(uint8_t n, bool on, bool resetDelayMenuShow=true);
extern void scale(uint16_t percent, uint8_t mask=0xFF, void delay(uint16_t ms)=NULL);
extern class Screen {

	uint8_t curPos = 0;

public:
	inline uint8_t getCurPos() {
		return curPos;
	}

	inline void moveCurPos(int8_t mv) {
		if (curPos + mv >= 0 && curPos + mv <= 3)
			curPos += mv;
	};

	void display();

} screen;

#endif

typedef void (* const Action)(float32_t val);

typedef float32_t (* const GetValue)();



class MenuItem {

	bool editable = false;
	float32_t value = NAN;
	Action action, setVal;
	GetValue getVal;


public:
	int16_t def,min,max;
	uint8_t diode;
	MenuItem(const GetValue getValue = NULL, const Action setVal = NULL,uint8_t led=0xff,int16_t def=0,int16_t min=0, int16_t max=199,
			const Action act = NULL) :
			action(act), setVal(setVal), getVal(getValue), def(def),min(min),max(max) {
			if(led!=0xff ) diode = led;
	}

	float32_t getValue(bool read = true)  {

		if(getVal &&( read|| isnan(value)))
			value = getVal();
		return value;
	}

	void setValue(float32_t val) {
		value = val;

	}
	void saveValue() {
		led(diode,false);
		if (setVal)
			setVal(value);
	}
};

extern class Menu {

	uint8_t curIndex = 0;
	//uint8_t curCH=0;
	  const uint8_t maxIndex;
	int8_t digPos = 0;
	bool _firstRun=true;
public:
	static MenuItem  items[];


	 explicit  Menu(  uint8_t m ):maxIndex(m){


	}
	uint8_t getNch(){
		return (curIndex < 2 )?1:2;
	}
	inline bool doubleBtn(uint8_t &btn1, uint8_t &btn2) {
		if(btn1 >= 1 && btn2 >= 1){
			getCurentItem()->setValue(getCurentItem()->def);
			getCurentItem()->saveValue();
			btn1=0;
			btn2=0;
			return true;
		}
		return false;
	}

	void navigate(bool dig = false) {

		if (curIndex > 3){ //уставки
			if(dig){
				if(digPos <= 1)
					digPos++;
				else
					digPos = 1;
				return;
			}else{
				getCurentItem()->saveValue();
			if (curIndex == ((maxIndex - 3)/2 + 3 ))
				curIndex = 4;
			else if (curIndex == maxIndex - 1)
				curIndex = (maxIndex - 3) / 2 + 4;
			else
				curIndex++;
			}
		}else{
			led(getCurentItem()->diode,false);
			switch (curIndex) {
			case 0:
			case 2:
				curIndex++;
				break;
			default:
				curIndex--;
				break;
			}
		}
		led(items[curIndex].diode,true);
		getCurentItem()->getValue(true);
	}

	void switchCH_edit() {
		//curCH=!curCH;
		if (curIndex > 3) {//edit
			int8_t power = digPos - ((getCurentItem()->getValue(false)< 10 )?2:1);

			getCurentItem()->setValue(getCurentItem()->getValue(false)+myUtils::pow10(power));

		} else{

			if (curIndex < 2) {
				led(7,false);
				led(6,true);
				curIndex += 2;
			} else{
				led(6,false);
				led(7,true);
				curIndex -= 2;
			}
		}
	}
	bool isEdit = false;
	void enterExitSetting() {
		led(items[curIndex].diode,false);
		if (curIndex > 3) { //exit
			isEdit = false;
			getCurentItem()->saveValue();
			if (curIndex > (maxIndex - 3) / 2 + 3)
				curIndex = 2;
			else
				curIndex = 0;
			digPos=0;
		}else{
			isEdit = true;
			if (curIndex < 2) //enter
					curIndex = 4;
				else
					curIndex = (maxIndex - 3) / 2 + 4;
				digPos=1;
				getCurentItem()->getValue(true);
		 }
		led(items[curIndex].diode,true);
	}


	 MenuItem* getCurentItem() {

		return &(items[curIndex]);
	}

	 void firstRun(){
		if( _firstRun){
			led(getCurentItem()->diode,true);
		}
	 }
	void display();



}menu;

extern class ServiceMenu {

	uint8_t curIndex = 0;
const uint8_t maxIndex;
bool _firstRun=true;
public:
	static MenuItem  items[];
	static uint8_t curCH;


	 explicit  ServiceMenu(  uint8_t m ):maxIndex(m){
		//led(6,false);
		//led(7,true);
	}

		inline bool doubleBtn(uint8_t &btn1, uint8_t &btn2) {
			if(btn1 >= 1 && btn2 >= 1){
				getCurentItem()->setValue(getCurentItem()->def);
				getCurentItem()->saveValue();
				btn1=0;
				btn2=0;
				return true;
			}
			return false;
		}


	void plus(){
		MenuItem* item = getCurentItem();
		if(item->getValue(false) < item->max)
			item->setValue(item->getValue(false)+0.1);
	}

	void minus(){
		MenuItem* item = getCurentItem();
		if(item->getValue(false) > item->min)
			item->setValue(item->getValue(false)-0.1);
	}

	void longPlus(){
		getCurentItem()->saveValue();

		if(curIndex < maxIndex-1){
			curIndex++;
		}else{
			curIndex=0;
			curCH=!curCH;
		//	if(curCH){
		///		led(6,false);led(7,true);
		///	}else{
		//		led(7,false);led(6,true);
		//	}
		}

		led(getCurentItem()->diode,true);
	}

	void longMinus(){
		getCurentItem()->saveValue();

		if(curIndex > 0 ){
			curIndex--;
		}else{
			curIndex=maxIndex-1;
			curCH=!curCH;
			//if(curCH){
		//		led(6,false);led(7,true);
		//	}else{
		//		led(7,false);led(6,true);
		//	}
		}

		led(getCurentItem()->diode,true);
	}

	 MenuItem* getCurentItem() {

		return &(items[curIndex]);
	}
	uint8_t getCurentIndex(){
		 return curIndex ;
	 }
	 void firstRun(){
		if( _firstRun){
			led(getCurentItem()->diode,true);
		}
	 }
	void display();



} serviceMenu;

