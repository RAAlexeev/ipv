/*
 * Filter.cpp
 *
 *  Created on: 27 рту. 2020 у.
 *      Author: alekseev
 */
#undef DEBUG






#include <Filter.h>

float32_t const Filter::coaf[10] __attribute__((section(".ccmram")))={b10,b11,b12,-a11,-a12,b20,b21,b22,-a21,-a22};

Filter::Filter() {
	// TODO Auto-generated constructor stub
	arm_biquad_cascade_df1_init_f32(&S, 2,(float32_t*)coaf, state);
}

Filter::~Filter() {
	// TODO Auto-generated destructor stub
}




 void Filter::exec( float32_t  *pSrc, float32_t *pDst,  uint32_t  blockSize)const{
		float32_t   *bufIn = pSrc;//new float32_t[LEN];
	 //{2,state,a,b};
//	 arm_iir_lattice_init_f32(&S,1,a,b,state,blockSize);
	//static const int v = init(&S);
	//(void)v;


   // float32_t sum = 0;

	//for(uint16_t i = 0; i < LEN; i+=16 ){
	//		sum+=pSrc[i];
	//   	}
	//float32_t aver = sum*16/LEN;

//,*bufOut_f32=new(float32_t[blockSize]);

	//arm_q15_to_float(srcInt,bufIn_f32,LEN);
	//for(uint16_t i =0; i < blockSize;i++){
	//	bufIn[i]= pSrc[i]-aver;
	//}
	arm_biquad_cascade_df1_f32(&S, bufIn, pDst, blockSize);

	//q31_t * buf_Q31=(q31_t*)((void*)buf);
	//arm_float_to_q31(bufOut_f32,buf_Q31,LEN);
	//delete[] state;
	//delete[] bufIn;
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

