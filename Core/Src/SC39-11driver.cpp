/* SC39-11 driver */
#include "main.h"
#include "SC39-11driver.h"

__IO uint32_t SC39_IO[2] = { 0 };

uint16_t SC39_get_dig(uint8_t n, bool dot) {

	switch (n) {
	case 1:
		return (B_Pin | C_Pin | (dot ? DP_Pin : 0));

	case 2:
		return (A_Pin | B_Pin | D_Pin | E_Pin | G_Pin | (dot ? DP_Pin : 0));

	case 3:
		return (A_Pin | B_Pin | C_Pin | D_Pin | G_Pin | (dot ? DP_Pin : 0));

	case 4:
		return (B_Pin | C_Pin | F_Pin | G_Pin | (dot ? DP_Pin : 0));

	case 5:
		return (A_Pin | C_Pin | D_Pin | F_Pin | G_Pin | (dot ? DP_Pin : 0));

	case 6:
		return (A_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin
				| (dot ? DP_Pin : 0));

	case 7:
		return (A_Pin | B_Pin | C_Pin | (dot ? DP_Pin : 0));

	case 8:
		return (A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin
				| (dot ? DP_Pin : 0));
	case 9:
		return (A_Pin | B_Pin | C_Pin | D_Pin | F_Pin | G_Pin
				| (dot ? DP_Pin : 0));
	case 0:
		return (A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin
				| (dot ? DP_Pin : 0));

		/*		      case 'C' :  HAL_GPIO_WritePin(GPIOD, B_Pin|C_Pin|G_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOD, A_Pin|D_Pin|E_Pin|F_Pin, GPIO_PIN_SET);
		 break;
		 case 'E' :  HAL_GPIO_WritePin(GPIOD, B_Pin|C_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOD, A_Pin|D_Pin|E_Pin|F_Pin|G_Pin, GPIO_PIN_SET);
		 break;
		 case 'P' :  HAL_GPIO_WritePin(GPIOD, C_Pin|D_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOD, A_Pin|B_Pin|E_Pin|F_Pin|G_Pin, GPIO_PIN_SET);
		 break;
		 case 'F' :  HAL_GPIO_WritePin(GPIOD, B_Pin|C_Pin|D_Pin|E_Pin|F_Pin|G_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOD, A_Pin, GPIO_PIN_SET);
		 break;
		 case 'b' :  HAL_GPIO_WritePin(GPIOD, A_Pin|B_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOD, C_Pin|D_Pin|E_Pin|F_Pin|G_Pin, GPIO_PIN_SET);
		 break;
		 case 'A' : // HAL_GPIO_WritePin(GPIOD, D_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOD, A_Pin|B_Pin|C_Pin|E_Pin|F_Pin|G_Pin, GPIO_PIN_SET);
		 break;
		 case 'H' :  HAL_GPIO_WritePin(GPIOD, A_Pin|D_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOD, B_Pin|C_Pin|E_Pin|F_Pin|G_Pin, GPIO_PIN_SET);
		 break;
		 */case 'Ï': return (A_Pin | B_Pin | C_Pin | E_Pin | F_Pin | (dot ? DP_Pin : 0));

	}
	return 0;
}


inline uint8_t getDig(float32_t val, int8_t dig){
	uint32_t pow10 = 1;
	for(uint8_t i=0; i < dig; i++) pow10 *= 10;

	return static_cast<uint8_t>(static_cast<uint32_t>(val)/pow10 % 10);
}

static struct{bool d1, d2;} strob;


void SC39_show(float32_t val, uint8_t strobe ) {

	uint8_t d1, d2; //intPart = static_cast<uint8_t>(round(n*10)/10);

	bool dot1=false,dot2=false;
	val=round(val*10)/10;
	if (val < 10) {
		d1=	getDig(val*10,1);
		d2 = getDig(val*10,0);//round((n - intPart) * 10);
		dot1=true;
	} else if (val < 100) {
		d1=	getDig(val,1);
		d2 = getDig(val,0);//round(intPart - intPart / 10 * 10);
	} else if (val < 200) {
		d1=	getDig(val,2);//intPart=static_cast<uint8_t>(round(n));
		d2 = getDig(val,1);//round(round(n) - intPart / 100 * 100)/10;
		dot2=true;
	} else {
		d1 = 'Ï';
		d2 = 'Ï';
		dot2=true;
	}

	switch(strobe){
	case 1: strob.d2 = !strob.d2;
			strob.d1 = false;
		break;
	case 2: strob.d1 = !strob.d1;
			strob.d2 = false;
		break;
	default:
		strob.d1 = false;
		strob.d2 = false;
	};


	d1=(strob.d1)?0:SC39_get_dig(d1,dot1);
	d2=(strob.d2)?0:SC39_get_dig(d2,dot2);
	SC39_IO[0] = ((0x200 | ((uint8_t) ~d1)) << 16) | d1 | GPIO_BSRR_BS8;
	SC39_IO[1] = ((0x100 | ((uint8_t) ~d2)) << 16) | d2 | GPIO_BSRR_BS9;
}

void ____SC39_showDig(uint8_t dig, GPIO_PinState dp, uint8_t hg) {

//  static uint8_t digLast = 255;
	HAL_GPIO_WritePin(GPIOC, HG_1_Pin | HG_2_Pin, GPIO_PIN_RESET);/* switch ( hg )
	 {
	 case 1 :
	 HAL_GPIO_WritePin(HG_2_GPIO_Port, HG_2_Pin, GPIO_PIN_RESET );
	 break;
	 case 2 :
	 HAL_GPIO_WritePin(HG_1_GPIO_Port, HG_1_Pin, GPIO_PIN_RESET );
	 }*/
	HAL_GPIO_WritePin(G_GPIO_Port, DP_Pin, dp);
	HAL_GPIO_WritePin(GPIOD,
			A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin,
			GPIO_PIN_RESET);
	// if ( digLast != dig )
	switch (dig) {
	case '1':
		HAL_GPIO_WritePin(GPIOD, A_Pin | D_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, B_Pin | C_Pin, GPIO_PIN_SET);
		// HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		// HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
		// HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
		// HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
		// HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
		break;
	case '2':
		HAL_GPIO_WritePin(GPIOD, C_Pin | F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | B_Pin | D_Pin | E_Pin | G_Pin,
				GPIO_PIN_SET);
		//HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOD, C_Pin|F_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		break;
	case '3':
		HAL_GPIO_WritePin(GPIOD, E_Pin | F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | B_Pin | C_Pin | D_Pin | G_Pin,
				GPIO_PIN_SET);
		//HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
		// HAL_GPIO_WritePin(GPIOD, E_Pin|F_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
		// HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		break;
	case '4':
		HAL_GPIO_WritePin(GPIOD, A_Pin | D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, B_Pin | C_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		//HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		break;
	case '5':
		HAL_GPIO_WritePin(GPIOD, B_Pin | E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | C_Pin | D_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOD, B_Pin|E_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		break;
	case '6':
		HAL_GPIO_WritePin(GPIOD, B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		// HAL_GPIO_WritePin(GPIOD, B_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		break;
	case '7':
		HAL_GPIO_WritePin(GPIOD, D_Pin | E_Pin | F_Pin | G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | B_Pin | C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin|E_Pin|F_Pin|G_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
		break;
	case '8': // HAL_GPIO_WritePin(GPIOD, D_pin,E_Pin,F_Pin,G_Pin GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,
				A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		//HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		break;
	case '9':
		HAL_GPIO_WritePin(GPIOD, E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | B_Pin | C_Pin | D_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		//HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
		// HAL_GPIO_WritePin(GPIOD, E_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		break;

	case '0':
		HAL_GPIO_WritePin(GPIOD, A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin,
				GPIO_PIN_SET);
		//HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
		// HAL_GPIO_WritePin(GPIOD, G_Pin, GPIO_PIN_RESET);
		break;

	case 'C':
		HAL_GPIO_WritePin(GPIOD, B_Pin | C_Pin | G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | D_Pin | E_Pin | F_Pin, GPIO_PIN_SET);
		break;
	case 'E':
		HAL_GPIO_WritePin(GPIOD, B_Pin | C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | D_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		break;
	case 'P':
		HAL_GPIO_WritePin(GPIOD, C_Pin | D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | B_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		break;
	case 'F':
		HAL_GPIO_WritePin(GPIOD, B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin, GPIO_PIN_SET);
		break;
	case 'b':
		HAL_GPIO_WritePin(GPIOD, A_Pin | B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, C_Pin | D_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		break;
	case 'A': // HAL_GPIO_WritePin(GPIOD, D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, A_Pin | B_Pin | C_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		break;
	case 'H':
		HAL_GPIO_WritePin(GPIOD, A_Pin | D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, B_Pin | C_Pin | E_Pin | F_Pin | G_Pin,
				GPIO_PIN_SET);
		break;

	}
	switch (hg) {
	case 1:
		HAL_GPIO_WritePin(HG_1_GPIO_Port, HG_1_Pin, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(HG_2_GPIO_Port, HG_2_Pin, GPIO_PIN_SET);
	}
	//  digLast = dig;
}

