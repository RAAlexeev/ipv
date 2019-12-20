#include "main.h"
#include <stdio.h>
#include  "FFT.h"
#include "buffer.hpp"
#include "myUtils.hpp"
#include "stm32f4xx.h"

#include "integrator.hpp"
#define FILTER

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
#ifdef DEBUG
uint32_t buf[LEN];
#endif
extern  void filter( int16_t  *pSrc,float32_t *pDst,uint32_t blockSize);

void Recirculate::calc(int16_t  *srcInt, float32_t *velocity ) {
#define k (0.99f)
#define N (16000)

	//uint16_t *srcInt = static_cast<uint16_t *>( _src);


	CircularBuffer<float32_t,7> velocity_ =  CircularBuffer<float32_t,7>();
	//static  float32_t y=0 ;

	float32_t  sumQ=0;

#ifdef FILTER

	float32_t * bufOut_f32=new float32_t[LEN];
//for(uint16_t j=0;j < LEN/8 ;j++){
	 filter(srcInt/*+LEN/8*j*/,bufOut_f32,LEN);

	for(uint16_t i = 0; i < LEN; i++ ){
			y = bufOut_f32[i]+y*k;

#else
	uint32_t sum = 0;
	for(uint16_t i = 0; i < LEN; i++ ){
			sum+=srcInt[i];
	   	}
	float32_t aver = static_cast<float32_t>(sum)/LEN;
	for(uint16_t i = 0; i < LEN; i++ ){
		this.y = srcInt[i]-aver/*sin(M_PI/100*i)*/+y*k;
#endif

#ifdef DEBUG
		//	buf[i]=(int32_t)y+20000;
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
		arm_sqrt_f32( sumQ/LEN, &v);
		(void)sumQ;
		velocity_.put(v);
		*velocity = velocity_.average();
		(void)v;
#ifdef DEBUG
			char out[100];
			int32_t velocityInt = static_cast<int32_t>(*velocity);
			out[sprintf(out,"%i,%i\n",velocityInt,(int)((*velocity-velocityInt)*100))] = 0;

		myUtils::ITM_SendStr(out);
#endif
		//s=0;
		//n=N/fftLenReal;
		//}else --n;



}
