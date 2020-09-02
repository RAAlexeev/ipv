/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H__
#define __MENU_H__
#include "main.h"


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
