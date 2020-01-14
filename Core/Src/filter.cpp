/*
 * filter.c
 *
 *  Created on: 27 но€б. 2019 г.
 *      Author: alekseev
 */

#undef DEBUG
#include "main.h"
#include "arm_math.h"
#include  "FFT.h"
#include "myUtils.hpp"

#define b10 1
#define b11 1.1720877471195239
#define b12 1
#define a11 -1.9961542493973554
#define a12 0.99616829785517969
#define b20 1
#define b21 -1.999999840748488
#define b22 1
#define a21 -1.5228429402520176
#define a22 0.66085375658373646
#undef DEBUG
extern uint32_t buf[LEN];

 int  init(arm_biquad_cascade_df2T_instance_f32 *S,uint32_t  blockSize){
	// arm_biquad_cascade_df2T_init_f32  (S,1,a,b,state);
 return 0;
}

 void filter( int16_t  *pSrc, float32_t *pDst,  uint32_t  blockSize){
	float32_t  *state = new float32_t[8], *bufIn=new float32_t[LEN];
	arm_biquad_casd_df1_inst_f32   S; //{2,state,a,b};
//	 arm_iir_lattice_init_f32(&S,1,a,b,state,blockSize);
	//static const int v = init(&S);
	//(void)v;
	float32_t coeff[]={b10,b11,b12,-a11,-a12,b20,b21,b22,-a21,-a22};
	arm_biquad_cascade_df1_init_f32(&S, 2, coeff, state);

    float32_t sum = 0;

	for(uint16_t i = 0; i < LEN; i+=16 ){
			sum+=pSrc[i];
	   	}
	float32_t aver = sum*16/LEN;

//,*bufOut_f32=new(float32_t[blockSize]);

	//arm_q15_to_float(srcInt,bufIn_f32,LEN);
	for(uint16_t i =0; i < blockSize;i++){
		bufIn[i]= pSrc[i]-aver;
	}
	arm_biquad_cascade_df1_f32(&S, bufIn, pDst, blockSize);

	//q31_t * buf_Q31=(q31_t*)((void*)buf);
	//arm_float_to_q31(bufOut_f32,buf_Q31,LEN);
	delete[] state;
	delete[] bufIn;
// delete(bufOut_f32);
#ifdef DEBUG
	float32_t sq=0,sqr;
	for(uint16_t i =0; i < blockSize;i++){
		sq+= pDst[i]*pDst[i];
	}
	char out[100];
	arm_sqrt_f32( sq/LEN/*(N/fftLenReal * fftLenReal)*/, &sqr);
	int32_t sqr_int = static_cast<int32_t>(sqr);
	out[sprintf(out,"%i,%i\n",sqr_int, static_cast<int32_t>((sqr-sqr_int)*100))] = 0;
	myUtils::ITM_SendStr(out);

#endif


}
