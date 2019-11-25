/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H__
#define __MENU_H__
#include "main.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"


#define MENU_ITEMS (5)
typedef   struct sMenuItem MenuItem_t;
typedef   void(* pSaveParam)( MenuItem_t * item );
typedef   uint8_t(* pGetParam)( MenuItem_t * item );
typedef   MenuItem_t*(* pItem)( void );

typedef struct   
{
  float32_t velocity;
  enum {eOK, eFAIL } error;   
}CH_t;

  struct sMenuItem
{
  bool dot; 
  uint8_t param;
  enum { iFirst = 0, iVrly = 0, iVpreRly, iVmin, iVmax, iCoef, iF_lo, iF_hi, iLast } index;
  pSaveParam fSaveParam;
  pGetParam fGetParam;
  
};

 struct sMenu
{
  bool active;
  bool edit;
  bool CHsw;
  bool CHn;
  uint8_t  index;
  enum { DEFAULTmode = 0, VELOCITYmode = 0, MAXmode, MODES }mode;
  MenuItem_t items[2][iLast];
  pItem item; 
};

void saveParam( MenuItem_t * item);

MenuItem_t * item( void );

extern struct sMenu mainMenu;

void menuInit(void);

extern void setCoef( uint8_t CHn , uint8_t Coef );
 
#endif
