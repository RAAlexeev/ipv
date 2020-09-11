

#include <math.h>



#include <stdlib.h>
#define STM32F405xx
#define ARM_MATH_CM4
#include <stdio.h>
#include  "FFT.h"
#include "stm32f4xx.h"
#include "arm_math.h"


static arm_rfft_fast_instance_f32 S;
static arm_iir_lattice_instance_f32  arm_iir_lattice_inst_f32;

int rfft_init()
{
	  //  arm_status status = ARM_MATH_SUCCESS;   
		/* Initialize the CFFT/CIFFT module */  

       	//return arm_rfft_fast_init_f32(&S, fftLenReal);
  
  static  float32_t pkCoeffs[]= {
     //   0.01020853419306819500,
        0.04083413677227278000,
        0.06125120515840917100,
        0.04083413677227278000,
        0.01020853419306819500
     };
        float32_t tmp;
        for (int8_t c = sizeof(pkCoeffs)/sizeof(float32_t) - 1, d = 0; c > d; c--, d++){
          tmp = pkCoeffs[d];
          pkCoeffs[d] = pkCoeffs[c];
          pkCoeffs[c] = tmp;
        }
      static float32_t pvCoeffs[]={
        1.00000000000000000000,
        -1.96842778693851760000,
        1.73586070920888560000,
        -0.72447082950736208000,
        0.12038959989624438000
      };
      for (int8_t c = sizeof(pvCoeffs)/sizeof(float32_t) - 1, d = 0; c > d; c--, d++){
     
        tmp=pvCoeffs[d];
        pvCoeffs[d] = pvCoeffs[c];
        pvCoeffs[c]=tmp;
      }
     static float32_t pState[2+LEN];
   // arm_iir_lattice_init_f32(&arm_iir_lattice_inst_f32, 4, pkCoeffs, pvCoeffs, pState, fftLenReal);
        return ARM_MATH_SUCCESS;


}
  
void rfft(void  * _src, uint16_t begin, uint16_t end, float32_t * velocity ) 
{	 
  uint16_t   *srcInt = _src;
  float32_t *bufIn=(float32_t *)malloc(sizeof(float32_t)*LEN), *bufOut=(float32_t *)malloc(sizeof(float32_t)*LEN);
      while(!(bufIn && bufOut));
	for(uint16_t i = 0; i < LEN; i++ )//    3 cycles * fftLenReal
        {
		bufIn[i] = srcInt[i];//-2027; //    3 cycles
	}
         
        //   #define M_PI 3.14159265359
       
        for(int16_t i=0;i <LEN;i+=1)
         // for(int16_t j=0;i <80;i+=1)
          {
          bufIn[i]=sin(i*M_PI/8);
          };
//          bufIn[i]=0;
//          bufIn[i+1]=sin(M_PI/4);
//          bufIn[i+2]=1;
//          bufIn[i+3]=sin(M_PI/4);
//          bufIn[i+4]=0;
//          bufIn[i+5]=-sin(M_PI/4);
//          bufIn[i+6]=-1;
//          bufIn[i+7]=-sin(M_PI/4);
//        }
     //   for(uint16_t i=1; i<fftLenReal;i+=4)bufIn[i]=4;
      //   for(uint16_t i=3; i <fftLenReal;i+=4)bufIn[i]=-4;
      //  for(uint16_t i=2; i <fftLenReal;i+=2)bufIn[i]=0;
#define ifftFlagR 0 //flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform.
	/* Process the data through the CFFT/CIFFT module */ 
//	arm_rfft_fast_f32(&S, bufIn , bufOut, ifftFlagR);
//        arm_cmplx_mag_f32(bufOut+2, bufIn, fftLenReal/2); 
//        arm_scale_f32(bufIn, 2.f/fftLenReal, bufOut, fftLenReal/2);
        //arm_iir_lattice_f32(&arm_iir_lattice_inst_f32, bufIn,bufOut,fftLenReal);
       // free(bufIn);

	
        float32_t *data = bufOut, tmp,sum=0,a,b,c;
//
       *velocity=0;
        extern TIM_HandleTypeDef htim2;
#define SAMPL_F (84000000u/htim2.Init.Prescaler/htim2.Init.Period)//      
//      for( uint16_t i = 0u; i < fftLenReal/*2000u/SAMPL_F*/; ++i, ++data )
//      {
//
//      //#define T //period 1050 * fftLenReal/21MHz   = fftLenReal/20 KHz 
//      ////    
//      #define Uop (2.5)
//      //#define G (9.8) 
//
//      ////   
//      ////    // * data * Uop /4096 /( 2 * M_PI * fftLenReal/20000/i)=
//           tmp= *data;  //( 2 * M_PI * SAMPL_F*(i+1)/fftLenReal);
//           tmp*=tmp;
//           sum +=tmp;
//            
//
//       }
#define buf bufIn
        for(uint16_t i =1u , j = 0u; i < LEN; i+=2, ++j){
          
          a=buf[i-1];b = buf[i];c=buf[i+1];
          tmp =(a + 4*b + c)/6;//8000;
          sum+=tmp*tmp; 
        }
       // sum/=SAMPL_F;
       // sum/=6;
     arm_sqrt_f32( 2*sum/ LEN , velocity );
   //   arm_rms_f32(buf,fftLenReal,&tmp );
     // tmp /= fftLenReal;
     // *velocity*=1000;
      //  *velocity=tmp*Uop/4095;//1.254/2030;
  free(bufIn);
  free(bufOut);
}

void minMaxAvr(q15_t *data){
      q15_t max, min, mean,variance;
      uint32_t index;
//    arm_max_q15	(   data,
//                    8000,
//                    &max,
//                    &index  
//                );
//
//    arm_mean_q15 (   data,
//                     8000,
//                     &mean
//                 );	
//    arm_min_q15(    data,
//                    8000,
//                    &min,
//                    &index  
//                );	
//   arm_var_q15 (data, 8000,&variance);
//  
//   printf( "%d;%d;%d;%d\r\n",min, max, mean,variance);
   //  for(uint32_t i=0;i<8000;++i) printf("%d\r\n",data[i]);
}
