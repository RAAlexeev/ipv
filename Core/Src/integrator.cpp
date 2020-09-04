#include "main.h"
#include <stdio.h>
#include  "FFT.h"
#include "buffer.hpp"
#include "myUtils.hpp"
#include "stm32f4xx.h"
#include "SC39-11driver.h"
#include "integrator.hpp"
#include "cmsis_os.h"
#include <array>
#define FILTER
#undef DEBUG
#define _DEBUG(x)// x
extern "C"  void my_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	SignalChenal::HAL_ADC_ConvCpltCallback(hadc);
}

extern osSemaphoreId  myCountingSem_S01Handle, myCountingSem_S02Handle;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void SignalChenal::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	  osSemaphoreRelease((hadc == &hadc1)?myCountingSem_S01Handle:myCountingSem_S02Handle);

	//	_DEBUG(myUtils::ITM_SendStr(( char*)((hadc->Instance == ADC1)?"\nS1":"\nS2")));
	//	_DEBUG(myUtils::ITM_SendStr(( char*)((getInstance(hadc)->buffer==getInstance(hadc)->buffer1)?"-1\n":"-2\n")));
}

SignalChenal::SignalChenal(){

}
void SignalChenal::init()
{


	if ( HAL_OK != HAL_ADC_Start_DMA( &hadc1, reinterpret_cast<uint32_t *>(SignalChenal::getInstance(&hadc1)->buffer), LEN ))
	  {
	     _Error_Handler(__FILE__, __LINE__);
	  }

	  if ( HAL_OK != HAL_ADC_Start_DMA( &hadc2, reinterpret_cast<uint32_t *>(SignalChenal::getInstance(&hadc2)->buffer), LEN ))
	  {
	     _Error_Handler(__FILE__, __LINE__);
	  }
}
/*void printDouble(double v, int decimalDigits)
{
  int i = 1;
  int intPart, fractPart;
  for (;decimalDigits!=0; i*=10, decimalDigits--);
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);
  if(fractPart < 0) fractPart *= -1;
  printf("%i.%i", intPart, fractPart);
}*/

extern inline void filter( float32_t  *pSrc, float32_t *pDst, uint32_t blockSize);

void * SignalChenal::swBuffer(){

    return buffer=(buffer==buffer2)?buffer1:buffer2;
}

void SignalChenal::calc() {
#define k (0.999f)
#define N (16000)
//	_DEBUG(myUtils::ITM_SendStr(getInstance(&hadc1)==this?"1":"2"));
	//uint16_t *srcInt = static_cast<uint16_t *>( _src);

	float *src=buffer;

    if ( HAL_OK != HAL_ADC_Start_DMA( &instances[0]==this?&hadc1:&hadc2, (uint32_t *)(swBuffer()), LEN ))
      {
         _Error_Handler(__FILE__, __LINE__);
      }





	//static  float32_t y=0 ;

	struct{
		float32_t  sumQ=0,sum=0,res;
	}V;

	struct{
		float32_t  sumQ=0,sum=0,res;
	}A;

	float32_t *bufOut_f32 = new float32_t[LEN];
while(bufOut_f32==NULL){
	;
}
	//for(uint16_t j=0;j < LEN/8 ;j++){
	//taskENTER_CRITICAL();
	 filter.exec(src/*+LEN/8*j*/, bufOut_f32,  LEN);
//taskEXIT_CRITICAL();

	 float32_t a;

	for(uint16_t i = 0; i < LEN; i++ ){
			a = bufOut_f32[i] * 9.82E+040;//0.668E+40;
			y = a + y*k;


			A.sum += a;
 		    A.sumQ += a*a;

			V.sum += y;
			V.sumQ += y*y;

			//bufOut_f32[i]=y;
	}

	delete[] bufOut_f32;
	if(ARM_MATH_SUCCESS == arm_sqrt_f32( ( A.sumQ*LEN - A.sum*A.sum )/(LEN*LEN), &A.res ) ){
		acceleration_.put( A.res );
	}


	if(ARM_MATH_SUCCESS == arm_sqrt_f32( ( V.sumQ*LEN - V.sum*V.sum )/(LEN*LEN), &V.res ) ){
		velocity_.put( V.res*0.0659 );
	}




		_DEBUG(uint16_t c =  uxSemaphoreGetCount((getInstance(&hadc1)==this)?myCountingSem_S01Handle:myCountingSem_S02Handle));
		_DEBUG(c+=0x30);

		_DEBUG(myUtils::ITM_SendStr(( char*)( &c )));





}
