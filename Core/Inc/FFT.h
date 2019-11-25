
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FFT_H__
#define __FFT_H__
#include "stm32f4xx.h"
#include "arm_math.h"

#define fftLenReal  (4096) //	length of the FFT  Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096.

arm_status rfft_init(void);
  
void rfft( void * restrict _src, uint16_t begin, uint16_t end, float32_t * velocity ) ;
#endif
