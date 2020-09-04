/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H__
#define __MENU_H__
#include "main.h"
#include "arm_math.h"

class Screen{

	uint8_t curPos=0;

public:
	uint8_t getCurPos(){ return curPos;	};
	inline void moveCurPos(int8_t mv){
		if(curPos+mv>=0 && curPos+mv<=3)
			curPos+=mv;
	};

	void display();

}screen;


 
#endif



typedef void (*const Action)(float32_t val);
typedef float32_t (*const GetValue)();
class MenuItem{
		bool editable = false;
		float32_t value;
		Action action,setVal;
		GetValue getVal;
		uint8_t diode;
		MenuItem(const Action act=NULL, const Action setVal=NULL, const GetValue getValue=NULL): action(act),setVal(setVal),getVal(getValue){
		};
		float32_t  getValue() const{
			return getVal?getVal():value;
		};
		void setValue(float32_t val){
			value=val;
			if(setVal)setVal(value);
		}
};

class Menu{
	uint8_t curItemIndex;
	static MenuItem items[];
	void move();
	void edit();
} menu;
