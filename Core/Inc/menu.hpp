/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H__
#define __MENU_H__
#include "main.h"
#include "arm_math.h"

extern class Screen{

	uint8_t curPos=0;

public:
	uint8_t getCurPos(){ return curPos;	};
	inline void moveCurPos(int8_t mv){
		if(curPos+mv>=0 && curPos+mv<=3)
			curPos+=mv;
	};

	void display();

} screen;
 
#endif



typedef void (*const Action)(float32_t val);
typedef float32_t (*const GetValue)();
class MenuItem{
		bool editable = false;
		float32_t value;
		Action action,setVal;
		GetValue getVal;
		uint8_t diode;
public:
		MenuItem(const GetValue getValue=NULL, const Action setVal=NULL, const Action act=NULL ): action(act),setVal(setVal),getVal(getValue){
		};
		float32_t  getValue() const{
			return getVal?getVal():value;
		};
		void setValue(float32_t val){
			if(setVal)setVal(value);
			value=val;
		}
};

extern struct Menu{
	uint8_t curIndex=0;
	uint8_t curCH=0;
	static MenuItem items[];
	static uint8_t maxIndex;
	void up(){
		if(curIndex < maxIndex/2)
		curIndex++;
		else
		curIndex=0;
	}
	void down(){
		if(curIndex)
		curIndex++;
		else
		curIndex=maxIndex;
	}
	void swCH(){
		curCH=!curCH;
	}

	void edit(){

	}
	MenuItem* getCurentItem(){
		return &(items[curIndex]);
	}
	void display();
}menu;
