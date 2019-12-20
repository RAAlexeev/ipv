/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H__
#define __MENU_H__
#include "main.h"




#define MENU_ITEMS (5)
typedef   struct sMenuItem MenuItem_t;
typedef   void(* pSaveParam)( MenuItem_t * item );
typedef   uint8_t(* pGetParam)( MenuItem_t * item );
typedef   MenuItem_t*(* pItem)( void );
typedef enum {eOK, eFAIL } Err;
typedef enum { iFirst = 0, iVrly = 0, iVpreRly, iVmin, iVmax, iCoef, iF_lo, iF_hi, iLast } MenuIndex;
typedef enum { DEFAULTmode = 0, VELOCITYmode = 0, MAXmode, MODES } Mode;
typedef struct   
{
  float velocity;
  Err error;
}CH_t;

  struct sMenuItem
{
	unsigned dot:1;
	uint8_t param;
	MenuIndex index;
  pSaveParam fSaveParam;
  pGetParam fGetParam;
  
};

 struct sMenu
{
  unsigned active:1;
  unsigned  edit:1;
  unsigned CHsw:1;
  unsigned  CHn:1;
  uint8_t  index;
  Mode mode;
  MenuItem_t items[2][7];
  pItem item; 
};
/*
class Menu{

	MenuItem_t items[2][iLast];
 	pItem curItem;
 	MenuItem_t next(void);

 };
*/
void saveParam( MenuItem_t * item);

MenuItem_t * item( void );

extern struct sMenu mainMenu;

void menuInit(void);

extern void setCoef( uint8_t CHn , uint8_t Coef );
 
#endif
