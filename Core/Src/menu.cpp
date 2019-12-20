#include <menu.hpp>
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "modbus-master/modbus.h"
#include "EEPROM.h"
#define DOT_ON GPIO_PIN_SET
#define DOT_OFF GPIO_PIN_RESET

struct sMenu mainMenu = {0,0,0,0,0,VELOCITYmode,{1, 0, iFirst, saveParam, NULL}, item};

extern  CH_t CH[2];

extern void SC39_showDig( uint8_t dig, GPIO_PinState dp, uint8_t hg );


void display()
{
#define ms (13)
  static uint8_t dig_blink = 0;//, dot_blink = 0;
  if ( mainMenu.active ) 
  {
    if( mainMenu.edit )
    {
      if (++dig_blink < 255 / 2 )
      {
        SC39_showDig(' ', DOT_OFF, 1);
        osDelay(ms);
        SC39_showDig(' ', DOT_OFF, 2);
        osDelay(ms);
      }
      else if ( ++dig_blink < 255 )
      {
        SC39_showDig( mainMenu.item()->param / 10 , DOT_OFF, 1 );
        osDelay(ms);
        SC39_showDig( mainMenu.item()->param % 10 , DOT_OFF, 2 );
        osDelay(ms);
      }
      else dig_blink = 0;
    }
    else
    { 
      switch (mainMenu.index)
      { 

      case iVpreRly :
      case iVrly :  SC39_showDig( 'A', DOT_OFF, 1 );
      break;
      case iVmin :  SC39_showDig( 'L', DOT_OFF, 1 ); 
      break;
      case iVmax :  SC39_showDig( 'H', DOT_OFF, 1 );
      break;
      case iCoef :  SC39_showDig( 'C', DOT_OFF, 1 );
      break;
      case iF_hi : SC39_showDig( 'H', DOT_OFF, 1 );
      break;
      case iF_lo :  SC39_showDig( 'L', DOT_ON, 1 );
      break;

      default :  SC39_showDig( 'P', DOT_OFF, 1 );
      }
      osDelay(ms);
      SC39_showDig( mainMenu.index, DOT_OFF, 2 );
      osDelay(ms);

    }
  }else 
    for( uint8_t n = 100; n && mainMenu.CHsw; --n )//отоброжаем не долго выбраный канал
    {
      SC39_showDig( 'C' , DOT_OFF, 1 );
      osDelay(ms);
      SC39_showDig( mainMenu.CHn + 1, DOT_OFF, 2 );
      osDelay(ms);
    }
  mainMenu.CHsw = 0;
  GPIO_PinState dot;
  uint8_t dot_blink=0;
  switch( CH[mainMenu.CHn].error )
  {
  case eOK : 
            switch( mainMenu.mode )
            { 
            case VELOCITYmode:  

              if ( CH[mainMenu.CHn].velocity  < mainMenu.items[mainMenu.CHn][iVpreRly].param/10.f ) 
                 dot = DOT_ON;               
              else
               if (++dot_blink < 255/2)//маргаем точкой
                  dot = DOT_OFF;
                else if ( dot_blink < 255)
                    dot = DOT_ON;
                   else
                    dot_blink = 0;

              SC39_showDig((uint8_t)CH[mainMenu.CHn].velocity, dot, 1 );
              osDelay(ms);
              SC39_showDig((uint8_t)CH[mainMenu.CHn].velocity % 10, DOT_OFF, 2 );
              osDelay(ms);
              break;              
              
            }
    break;  

  case eFAIL:
          SC39_showDig( 'A' , DOT_OFF, 1 );
          osDelay(ms);
          SC39_showDig( 'A', DOT_OFF, 2 );
          osDelay(ms);
    break;      

    break; 
    
  }
}

uint16_t getItemEEaddr(MenuItem_t * item) 
{
#define offset (2) 
  if ( &mainMenu.items[1][0] <= item )  
    return 10 + item->index + offset;
 else 
    return item->index + offset; 
}

void saveParam( MenuItem_t * item )
{
    struct{
        uint16_t addr;
        uint8_t data;
    } data = {getItemEEaddr(item), item->param };


  extern I2C_HandleTypeDef hi2c1;

  if( HAL_OK  != HAL_I2C_Master_Transmit( &hi2c1, 0x28, (uint8_t *)&data, 3, 100 ) )
  {
	  _Error_Handler(__FILE__, __LINE__);
  }

}

void getParam( MenuItem_t * item )
{
 static   uint16_t addr; 
 addr = getItemEEaddr(item);
  extern I2C_HandleTypeDef hi2c1;
 if( HAL_OK  != HAL_I2C_Master_Receive_DMA(&hi2c1, 0x28, &item->param, 1) )
 {
     _Error_Handler(__FILE__, __LINE__);
 }
 if( HAL_OK  !=  HAL_I2C_Master_Transmit_DMA( &hi2c1, 0x28, (uint8_t *)&addr, 2) )
 {
    _Error_Handler(__FILE__, __LINE__);
 } 

}

MenuItem_t*  item ( void )
 {
    return &mainMenu.items[mainMenu.CHn][mainMenu.index];
 }


void saveCoifParam( MenuItem_t * item )
{
 setCoef(mainMenu.CHn, item->param);
  saveParam(item);
  if( mainMenu.CHn )
    ModBus_SetRegister( item->index+1, (uint16_t)(( item->param << 8 )| mainMenu.items[0][item->index].param ));
  else 
    ModBus_SetRegister( item->index+1, (uint16_t)( item->param |( mainMenu.items[0][item->index].param << 8 )) );
}

void menuInit(void)
{
  uint8_t data[2][10];
 
  extern I2C_HandleTypeDef hi2c1;

  EE_read(2, &data[0][0], 20);
  
  for(uint8_t i= 0; i < 2; ++i )
  {
    for(uint8_t j = iFirst; j < iLast; ++j )
      {
        if ( data[i][j] == 255 )
        switch( j )//значения по умолчанию
        {
          case iVpreRly: data[i][j] = 40;
          break;
          case iVrly: data[i][j] = 50;
          break;
          case iVmin: data[i][j] = 0;
          break;
          case iVmax: data[i][j] = 55;
          break;
          case iCoef: data[i][j] = 50;
          break;
          case iF_lo: data[i][j] = 0;
          break;
          case iF_hi: data[i][j] = 99;
          break;
        }
        mainMenu.items[i][j].index =  static_cast<MenuIndex>(j);
        mainMenu.items[i][j].param =  data[i][j];     // getParam(&mainMenu.items[i][j]);
      }
    mainMenu.items[i][iCoef].fSaveParam = saveCoifParam;
    
  }
     
}
