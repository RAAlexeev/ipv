
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "integrator.hpp"
#include "Menu.hpp"

extern osSemaphoreId myCountingSemBUT1Handle, myCountingSem_S01Handle, myCountingSem_S02Handle, myCountingSemBUT2Handle, myCountingSemTIM4Handle;
extern osTimerId myTimerBUT1Handle, myTimerBUT2Handle;
extern  void my_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
extern UART_HandleTypeDef huart1;
extern uint8_t service;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	 extern uint8_t but1pressed;
   switch( GPIO_Pin )
   {
     case BUT1_Pin:
        if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( BUT1_GPIO_Port, BUT1_Pin ) )
         {
              
               osTimerStart(myTimerBUT1Handle, 30 );

         } else if( but1pressed > 0 && but1pressed  < 7 ){//кнопка была нажата боле 30мс и  отпущена{


        	if(service && service < 5)
       			  service++;
       		else if(service)
       			  serviceMenu.minus();
           	 but1pressed = 0;
           	 osTimerStop(myTimerBUT1Handle);
         }
     break;

     case BUT2_Pin:
         if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( BUT2_GPIO_Port, BUT2_Pin ) )
         {
            osTimerStart(myTimerBUT2Handle, 30 );
           
         } else{ //кнопка была отпущена
           	 osTimerStop(myTimerBUT2Handle);//
           	 extern uint8_t but2pressed;


           	 if( but2pressed > 0 && (but2pressed < 7 || !menu.isEdit)){//кнопка была нажата боле 30мс и  отпущена
        	  if(!service)
        		  menu.switchCH_edit();
        	  else{
        		  if(service < 5)
        			  service++;
        		  else
        			  serviceMenu.plus();
        	  }
           	 }
           	 but2pressed = 0;
         }
           
       // osSemaphoreRelease(myCountingSemBUT1Handle);
     break;
   }
 }





void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	SignalChenal::HAL_ADC_ConvCpltCallback(hadc);
	//my_ADC_ConvCpltCallback(hadc);
	/*
  if( hadc->Instance == ADC1 )
  osSemaphoreRelease(myCountingSem_S01Handle);
  if( hadc->Instance == ADC2 )
  osSemaphoreRelease(myCountingSem_S02Handle);
  */
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
 (void)hspi;

}
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
//{ADC_DMAConvCplt
 // osSemaphoreRelease(myCountingSem_S01Handle);
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{

//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}




