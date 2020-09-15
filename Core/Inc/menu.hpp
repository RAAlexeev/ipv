/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H__
#define __MENU_H__
#include "main.h"
#include "arm_math.h"

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
	uint8_t diode;

public:
	MenuItem(const GetValue getValue = NULL, const Action setVal = NULL,
			const Action act = NULL) :
			action(act), setVal(setVal), getVal(getValue) {

	};
	float32_t getValue(bool read = true)  {
		if(getVal &&( read|| isnan(value)))
			value = getVal();
		return value;
	}

	void setValue(float32_t val) {
		value = val;

	}
	void saveValue() {
		if (setVal)
			setVal(value);
	}
};

extern class Menu {

	uint8_t curIndex = 0;
	//uint8_t curCH=0;
	  uint8_t maxIndex;
	int8_t digPos = 0;
public:
	static MenuItem  items[];


	constexpr explicit  Menu(  uint8_t m ):maxIndex(m){

	}

	inline bool doubleBtn(uint8_t &btn1, uint8_t &btn2) {
		if(btn1 >= 1 && btn2 >= 1){
			getCurentItem()->setValue(0);
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
			}else{
				getCurentItem()->saveValue();
			if (curIndex == ((maxIndex - 3)/2 + 3 ))
				curIndex = 4;
			else if (curIndex == maxIndex - 1)
				curIndex = (maxIndex - 3) / 2+4;
			else
				curIndex++;
			}
		}else
			switch (curIndex) {
			case 0:
			case 2:
				curIndex++;
				break;
			default:
				curIndex--;
				break;
			}
		getCurentItem()->getValue(true);
	}

	void switchCH_edit() {
		//curCH=!curCH;
		if (curIndex > 3) {//edit
			int8_t power = digPos - ((getCurentItem()->getValue(false)< 10 )?2:1);

			getCurentItem()->setValue(getCurentItem()->getValue(false)+pow(10,power));

		} else if (curIndex < 2) {
			curIndex += 2;
		} else
			curIndex -= 2;
	}

	void enterExitSetting() {
		if (curIndex > 3) { //exit
			getCurentItem()->saveValue();
			if (curIndex > (maxIndex - 3) / 2 + 3)
				curIndex = 2;
			else
				curIndex = 0;
			digPos=0;
		}else{
			if (curIndex < 2) //enter
					curIndex = 4;
				else
					curIndex = (maxIndex - 3) / 2 + 4;
				digPos=1;
				getCurentItem()->getValue(true);
		 }
	}

	void edit() {

	}
	 MenuItem* getCurentItem() {
		return &(items[curIndex]);
	}
	void display();



} menu;
