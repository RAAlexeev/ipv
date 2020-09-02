
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern osSemaphoreId myCountingSemBUT1Handle, myCountingSem_S01Handle, myCountingSem_S02Handle, myCountingSemBUT2Handle, myCountingSemTIM4Handle;
extern osTimerId myTimerBUT1Handle, myTimerBUT2Handle;
extern  void my_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
extern UART_HandleTypeDef huart1;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
  
   switch( GPIO_Pin )
   {
     case BUT1_Pin:
        if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( BUT1_GPIO_Port, BUT1_Pin ) )
         {
              
               osTimerStart(myTimerBUT1Handle, 1000 );

         } else osTimerStop( myTimerBUT1Handle );
          
     break;
     case BUT2_Pin:
         if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( BUT2_GPIO_Port, BUT2_Pin ) )
         {
            osTimerStart(myTimerBUT2Handle, 100 );          
           
         } else osTimerStop(myTimerBUT2Handle);
         
           
       // osSemaphoreRelease(myCountingSemBUT1Handle);
     break;
   }
 }





void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	my_ADC_ConvCpltCallback(hadc);
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
 osSemaphoreRelease(myCountingSem_S02Handle);
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
  if ( htim->Instance == TIM4 )osSemaphoreRelease(myCountingSemTIM4Handle);
}
uint8_t uartBuf[2];
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{


	//if(&huart1==huart){ only one huart
	extern uint8_t mb_buf_in[256];
	HAL_GPIO_WritePin(U1_DE_GPIO_Port,U1_DE_Pin,GPIO_PIN_RESET);

//	  HAL_UART_Receive_IT(&huart1, &uartBuf , 2);
  	  HAL_NVIC_EnableIRQ(TIM4_IRQn);
	//}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	HAL_GPIO_WritePin(U1_DE_GPIO_Port,U1_DE_Pin,GPIO_PIN_SET);
//	HAL_UART_Transmit_IT( &huart1,&uartBuf,2 );
HAL_NVIC_DisableIRQ(TIM4_IRQn);
  extern TIM_HandleTypeDef htim4;
  HAL_TIM_Base_Start_IT(&htim4);
}
