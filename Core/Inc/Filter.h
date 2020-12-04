/*
 * Filter.h
 *
 *  Created on: 27 рту. 2020 у.
 *      Author: alekseev
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_


#include "main.h"
#include "arm_math.h"
 float32_t const b10 = 1;
 float32_t const b11 = 1.17208778858184814453125;//1.1720877471195239
 float32_t const b12 = 1;
 float32_t const a11 = -1.996154308319091796875;//-1.9961542493973554
 float32_t const a12 = 0.996168315410614013671875;//0.99616829785517969
 float32_t const b20 = 1;
 float32_t const b21 = -1.99999988079071044921875;//-1.999999840748488
 float32_t const b22 = 1;
 float32_t const a21 = -1.522842884063720703125;//-1.5228429402520176
 float32_t const a22 = 0.66085374355316162109375;//0.66085375658373646

class Filter {


 static float32_t const coaf[10] __attribute__((section(".ccmram")));
float32_t state[8];
	arm_biquad_casd_df1_inst_f32   S;
public:
	Filter();
	virtual ~Filter();
		void exec(float32_t  *pSrc, float32_t *pDst,  uint32_t  blockSize)const;
};



#endif /* INC_FILTER_H_ */
