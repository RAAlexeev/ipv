/**
Modbus slave implementation for STM32 HAL under FreeRTOS.
(c) 2017 Viacheslav Kaloshin, multik@multik.org
Licensed under LGPL. 
**/
#ifndef __modbus_H
#define __modbus_H
#ifdef __cplusplus
//extern "C" {
#endif
#include "cmsis_os.h"
// 3,5 character timeout or frame timeout
// This value for 9600 speed and 1000Hz tick counter
#define ModBus35 35
// How many holding registers we are serve?
#define ModBusRegisters (4*15) // 0-9
  extern uint8_t mb_buf_in[256];
   void ModBus_SetAddress(uint8_t addr);
   // set value of register
   void ModBus_SetRegister(uint8_t reg,uint16_t value);
   // grab value of register
   uint16_t ModBus_GetRegister(uint8_t reg);
   bool ModBusParse(uint8_t cnt);
   void ModBus_Init();

 //  #ifdef __cplusplus
//}

//#endif
#endif
