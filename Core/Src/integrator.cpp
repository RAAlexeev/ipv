#include "main.h"
#include <stdio.h>
#include  "FFT.h"
#include "buffer.hpp"
#include "myUtils.hpp"
#include "stm32f4xx.h"

#include "integrator.hpp"
#include "cmsis_os.h"

#define FILTER

extern "C"  void my_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	SignalChenal::HAL_ADC_ConvCpltCallback(hadc);
}

extern osSemaphoreId  myCountingSem_S01Handle, myCountingSem_S02Handle;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void SignalChenal::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	  osSemaphoreRelease((hadc->Instance == ADC1)?myCountingSem_S01Handle:myCountingSem_S02Handle);

	    if ( HAL_OK != HAL_ADC_Start_DMA( hadc, (uint32_t *)(getInstance(hadc)->buffer), LEN ))
	      {
	         _Error_Handler(__FILE__, __LINE__);
	      }

}

SignalChenal::SignalChenal(){

}
void SignalChenal::init()
{
	  if ( HAL_OK != HAL_ADC_Start_DMA( &hadc1, (uint32_t *)SignalChenal::getInstance(&hadc1)->buffer, LEN ))
	  {
	     _Error_Handler(__FILE__, __LINE__);
	  }
	  if ( HAL_OK != HAL_ADC_Start_DMA( &hadc2, (uint32_t *)SignalChenal::getInstance(&hadc2)->buffer, LEN ))
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

extern inline void filter( int16_t  *pSrc, float32_t *pDst, uint32_t blockSize);

void SignalChenal::calc() {
#define k (0.99f)
#define N (16000)

	//uint16_t *srcInt = static_cast<uint16_t *>( _src);


	int16_t *src=buffer;
    if(buffer==buffer1){
    	buffer=buffer2;
    }else{
    	buffer=buffer1;
    }
	//static  float32_t y=0 ;

	float32_t  sumQ=0;

#ifdef FILTER

	float32_t * bufOut_f32 = new float32_t[LEN];
//for(uint16_t j=0;j < LEN/8 ;j++){
	 filter(src/*+LEN/8*j*/, bufOut_f32,  LEN);
#ifdef DEBUG
	 float32_t  *tstbuf = new float32_t[400];
#endif
	for(uint16_t i = 0; i < LEN; i++ ){
			y = bufOut_f32[i] +y*k;

#else

	for(uint16_t i = 0; i < LEN; i++ ){
		y = src[i]-aver/*sin(M_PI/100*i)*/+y*k;
#endif

#ifdef DEBUG
			if(i<400)tstbuf[i]=y;
#endif
		//	printf("%i\n",(int)(sin(M_PI/800*i)*100)/*(srcInt[i]-2043)*/);
			//ITM_SendChar( 65 );
			//printf("%i\n",(int)y);
			sumQ+=y*y;
//	}
}

#ifdef FILTER
		delete[] bufOut_f32;
#endif
		//static uint8_t n = N/fftLenReal;
		//if(n==0){
		float32_t v;
		arm_sqrt_f32( sumQ/LEN, &v );
		(void)sumQ;
		velocity_.put(v);

		(void)v;
#ifdef DEBUG
		delete[] tstbuf;
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
