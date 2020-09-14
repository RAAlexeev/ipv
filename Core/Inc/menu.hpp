/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H__
#define __MENU_H__
#include "main.h"
#include "arm_math.h"

extern class Screen {

	uint8_t curPos = 0;

public:
	uint8_t getCurPos() {
		return curPos;
	}
	;
	inline void moveCurPos(int8_t mv) {
		if (curPos + mv >= 0 && curPos + mv <= 3)
			curPos += mv;
	}
	;

	void display();

} screen;

#endif

typedef void (* const Action)(float32_t val);
typedef float32_t (* const GetValue)();
class MenuItem {
	bool editable = false;
	float32_t value;
	Action action, setVal;
	GetValue getVal;
	uint8_t diode;
public:
	MenuItem(const GetValue getValue = NULL, const Action setVal = NULL,
			const Action act = NULL) :
			action(act), setVal(setVal), getVal(getValue) {
	}
	;
	float32_t getValue() const {
		return getVal ? getVal() : value;
	}
	;
	void setValue(float32_t val) {
		if (setVal)
			setVal(value);
		value = val;
	}
};

extern class Menu {
	uint8_t curIndex = 0;
	//uint8_t curCH=0;
	uint8_t maxIndex;
public:
	static MenuItem const items[];
	Menu(uint8_t size) {
		maxIndex = size;
	}

	void down() {

		if (curIndex > 3){ //уставки
			if (curIndex == ((maxIndex - 3)/2 + 3 - 1))
				curIndex = 4;
			else if (curIndex == maxIndex - 1)
				curIndex = 3 + (maxIndex - 3) / 2;
			else
				curIndex++;
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

	}
	void switchCH() {
		//curCH=!curCH;
		if (curIndex > 3) {
			if (curIndex < (maxIndex - 3) / 2 + 3)
				curIndex += (maxIndex - 3) / 2 + 3;
			else
				curIndex -= (maxIndex - 3) / 2 + 3;
		} else if (curIndex < 2) {
			curIndex += 2;
		} else
			curIndex -= 2;
	}

	void enterExitSetting() {
		if (curIndex > 3) {
			if (curIndex > (maxIndex - 3) / 2 + 3)
				curIndex = 2;
			else
				curIndex = 0;
		} else if (curIndex < 2)
			curIndex = 4;
		else
			curIndex = (maxIndex - 3) / 2 + 3;
	}

	void edit() {

	}
	const MenuItem* getCurentItem() {
		return &(items[curIndex]);
	}
	void display();



} menu;
