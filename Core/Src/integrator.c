#include "FFT.h"
#include <stdio.h>

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

uint32_t buf[fftLenReal];
struct AverFifo{
	uint8_t len;

};
void recirculate(void  * _src, float32_t * velocity ) {
#define k (0.99)
#define N (16000)

	  uint16_t   *srcInt = _src;

	static  float32_t y=0 ;
	float32_t  s=0;
	float32_t aver = 0;
	//=(uint32_t *)malloc(sizeof(uint32_t)*fftLenReal);//, *bufOut=(float32_t *)malloc(sizeof(float32_t)*fftLenReal);
	     // while(!bufIn );
		for(uint16_t i = 0; i < fftLenReal; i++ ){
			aver+=srcInt[i];
	   	}
		aver =aver/fftLenReal;
		for(uint16_t i = 0; i < fftLenReal; i++ )//    3 cycles * fftLenReal
	        {

			//bufIn[i] = srcInt[i];//-2027; //    3 cycles
			y = srcInt[i]-aver/*sin(M_PI/100*i)*/+y*k;
			buf[i]=(int32_t)y+20000;
		//	printf("%i\n",(int)(sin(M_PI/800*i)*100)/*(srcInt[i]-2043)*/);
			//ITM_SendChar( 65 );
			//printf("%i\n",(int)y);
			s+=y*y;
		}
		//static uint8_t n = N/fftLenReal;
		//if(n==0){
			arm_sqrt_f32( s/fftLenReal/*(N/fftLenReal * fftLenReal)*/, velocity);
			printf("%i,%i\n",(int)*velocity,(int)((*velocity-(int)*velocity)*100));
			//s=0;
		//n=N/fftLenReal;
		//}else --n;



}
