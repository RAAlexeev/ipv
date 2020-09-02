#include "main.h"
#include <stdio.h>
#include  "FFT.h"
#include "buffer.hpp"
#include "myUtils.hpp"
#include "stm32f4xx.h"
#include "SC39-11driver.h"
#include "integrator.hpp"
#include "cmsis_os.h"
#include <vector>
#define FILTER
#undef DEBUG
#define _DEBUG(x) x
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
#define k (0.99f)
#define N (16000)
//	_DEBUG(myUtils::ITM_SendStr(getInstance(&hadc1)==this?"1":"2"));
	//uint16_t *srcInt = static_cast<uint16_t *>( _src);

	float *src=buffer;

    if ( HAL_OK != HAL_ADC_Start_DMA( &instances[0]==this?&hadc1:&hadc2, (uint32_t *)(swBuffer()), LEN ))
      {
         _Error_Handler(__FILE__, __LINE__);
      }





	//static  float32_t y=0 ;

	float32_t  sumQ=0,sum=0;

#ifdef FILTER
std::vector<float32_t> bufOut_f32(LEN);
	//float32_t * bufOut_f32 = new float32_t[LEN];
//for(uint16_t j=0;j < LEN/8 ;j++){
//taskENTER_CRITICAL();
	 filter.exec(src/*+LEN/8*j*/, bufOut_f32.data(),  LEN);
//taskEXIT_CRITICAL();


	for(uint16_t i = 0; i < LEN; i++ ){
			y = bufOut_f32[i]*0.668E+40 +y*k;
			bufOut_f32[i]=y;

#else

	for(uint16_t i = 0; i < LEN; i++ ){
		y = src[i]-aver/*sin(M_PI/100*i)*/+y*k;
#endif

#ifdef DEBUG
			if(i<400)tstbuf[i]=y;
#endif

			sum +=y;
			sumQ +=y*y;


}
	const float32_t d = sum/LEN;

		//static uint8_t n = N/fftLenReal;
		//if(n==0){
		float32_t v;

		if(ARM_MATH_SUCCESS == arm_sqrt_f32( (sumQ-sum*sum/LEN)/LEN, &v )){


			velocity_.put( v );
		}




		uint16_t c =  uxSemaphoreGetCount((getInstance(&hadc1)==this)?myCountingSem_S01Handle:myCountingSem_S02Handle);
		c+=0x30;

		_DEBUG(myUtils::ITM_SendStr(( char*)( &c )));

#ifdef DEBUG

			char out[30];

			int32_t velocityInt = static_cast<int32_t>(getVelocity());

			static unsigned  char ch='\n';

			if( ch=='\0' ) ch='\n'; else ch='\0';
			uint8_t end=sprintf(out,"%i,%i;",(int)velocityInt,(int)((getVelocity()-velocityInt)*100));
			out[end] = ch;
			out[end+1] = 0;
		myUtils::ITM_SendStr(out);
#endif
		//s=0;
		//n=N/fftLenReal;
		//}else --n;



}
