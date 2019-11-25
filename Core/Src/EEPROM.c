#include "main.h"
extern I2C_HandleTypeDef hi2c1;
void  EE_write(uint16_t addr, void * pData, uint16_t len )
{
  if( HAL_OK  !=  HAL_I2C_Master_Transmit( &hi2c1, 0xA0, pData, len, 200))
  {
    _Error_Handler(__FILE__, __LINE__);
  } 
}
void EE_read(uint16_t addr, void * pData, uint16_t len )
 {
  if( HAL_OK  !=  HAL_I2C_Master_Transmit( &hi2c1, 0xA0, (uint8_t *)&addr, sizeof(addr), 200) )
  {
    _Error_Handler(__FILE__, __LINE__);
  } 
  if( HAL_OK  != HAL_I2C_Master_Receive( &hi2c1, 0xA0, pData, len, 200) )
  {
    _Error_Handler(__FILE__, __LINE__);
  }
 }
