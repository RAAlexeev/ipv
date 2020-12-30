/**
Modbus slave implementation for STM32 HAL under FreeRTOS.
(c) 2017 Viacheslav Kaloshin, multik@multik.org
Licensed under LGPL. 
**/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "modbus.h"
#include "integrator.hpp"
#include "EEPROM.hpp"
// If you want directly send to usb-cdc
// #include "usbd_cdc_if.h"

osMessageQId ModBusInHandle;
osMessageQId ModBusOutHandle;
osThreadId ModBusTaskHandle;



//uint16_t _mb_reg[ModBusRegisters];

// Here is actual modbus data stores
uint8_t mb_buf_in[256]={0};
uint8_t mb_buf_in_count;
uint8_t mb_addr;
uint8_t mb_buf_out[256];
uint8_t mb_buf_out_count;
uint8_t* p_mb_buf_in = mb_buf_in;
#define mb_buf_in p_mb_buf_in
inline uint16_t mb_reg(uint32_t index, uint16_t value=0xFFFF ){
	uint16_t outCtrl(uint16_t x = 0xFFFF);
	static union fu_t{
		float32_t f;
		uint32_t ui;
	}b;
if(value==0xFFFF)
	switch(index){
		case 1:return EEPROM.uartSpeed();
		case 2:return uartMBparam::param.p.parity;
		case 3:return uartMBparam::param.p.stopBit;
		case 4:return uartMBparam::getAddr();
		case 5:return EEPROM.relayDelay();
		case 6:return EEPROM.K1();
		case 7:return EEPROM.K2();
		case 8: return outCtrl();
		case 10: return EEPROM.porog11();
		case 11: return EEPROM.porog12();
		case 12: return EEPROM.range1();
		case 13: return EEPROM.porog21();
		case 14: return EEPROM.porog22();
		case 15: return EEPROM.range2();
		case 16: b.f = SignalChenal::getInstance(ADC1)->getAcceleration();
			return (uint16_t)(b.ui>>16);
		case 17:// b.f = SignalChenal::getInstance(ADC1)->getAcceleration();
			return (uint16_t)(b.ui);

		case 18: b.f = SignalChenal::getInstance(ADC1)->getVelocity();
			return (uint16_t)(b.ui>>16);
		case 19: //b.f = SignalChenal::getInstance(ADC1)->getVelocity();
			return (uint16_t)(b.ui);

		case 20: b.f = SignalChenal::getInstance(ADC2)->getAcceleration();
			return (uint16_t)(b.ui>>16);
		case 21: b.f = SignalChenal::getInstance(ADC2)->getAcceleration();
			return (uint16_t)(b.ui);

		case 22: b.f = SignalChenal::getInstance(ADC2)->getVelocity();
			return (uint16_t)(b.ui>>16);
		case 23: b.f = SignalChenal::getInstance(ADC2)->getVelocity();
			return (uint16_t)(b.ui);
		default:return 0;
	}else switch(index){
			case 1:return EEPROM.uartSpeed.set(value);
			case 2:return uartMBparam::setParity(value);
			case 3:return uartMBparam::setStopBit(value);
			case 4:return uartMBparam::setAddr(value);
			case 5:return EEPROM.relayDelay.set(value);
			case 6:return EEPROM.K1.set(value);
			case 7:return EEPROM.K2.set(value);
			case 8:return outCtrl(value);
			case 10: return EEPROM.porog11.set(value);
			case 11: return EEPROM.porog12.set(value);
			case 12: return EEPROM.range1.set(value);
			case 13: return EEPROM.porog21.set(value);
			case 14: return EEPROM.porog22.set(value);
			case 15: return EEPROM.range2.set(value);
			default:return 0;
	}
	return 0;
}
/*
void ModBusTask( const * argument)
{
 // for(;;)
  {
    osEvent evt = osMessageGet(ModBusInHandle,ModBus35);
    // Frame end?
    if (evt.status == osEventTimeout)
      {
        if(mb_buf_in_count > 0) // ok, something in buffer exist, lets parse it
        {
          ModBusParse();
        }  
      mb_buf_in_count=0;
      }
    // Wow, something come!
    if (evt.status == osEventMessage)
      {
        uint8_t byte = (uint8_t) evt.value.v;
        // buffer has space for incoming?
        if(mb_buf_in_count<254)
        {
          mb_buf_in[mb_buf_in_count]=byte;
          mb_buf_in_count=mb_buf_in_count+1; // prevent opt/war on come compilers
        }
        else // oops, bad frame, by standard we should drop it and no answer
        {
          mb_buf_in_count=0;
        }
      }
  }
}
*/
void ModBus_Init(void)
{
 // osMessageQDef(ModBusIn, 256, uint8_t);
 // ModBusInHandle = osMessageCreate(osMessageQ(ModBusIn), NULL);
 // osMessageQDef(ModBusOut, 256, uint8_t);
 // ModBusOutHandle = osMessageCreate(osMessageQ(ModBusOut), NULL);
 // osThreadDef(ModBusTask, ModBusTask, osPriorityNormal, 0, 128);
 // ModBusTaskHandle = osThreadCreate(osThread(ModBusTask), NULL);
  mb_buf_in_count=0;
  mb_addr=247; // by default maximum possible adrress
  mb_buf_out_count=0;
//  for(int i=0;i<ModBusRegisters;i++)
 // {
  //  mb_reg[i]=0;
//  }
}

void ModBus_SetAddress(uint8_t addr)
{
  mb_addr = addr;
}

void CRC16_OUT(void);
uint8_t CRC16_IN(void);
extern void sendData(uint8_t *data, uint8_t len);
// parse something in incoming buffer 
bool ModBusParse(uint8_t  count)
{

	mb_buf_in_count=count;
   if(mb_buf_in_count == 0) // call as by mistake on empty buffer?
   {
    	//sendData(mb_buf_out, 0);
    	return false;
    }
    
  //  if(mb_buf_in[0] != mb_addr) // its not our address!
//    {
 //   	sendData(mb_buf_out, 0);
//    	return;
  //  }
    // check CRC
    if(CRC16_IN()==0){
      mb_buf_out_count = 0;
      uint16_t st,nu;
      uint8_t func = mb_buf_in[1];
      uint8_t i;
      switch(func)
      {
        case 3:
        case 4:
          // read holding registers. by bytes addr func starth startl totalh totall
          st=mb_buf_in[2]*256+mb_buf_in[3];
          nu=mb_buf_in[4]*256+mb_buf_in[5];
          if( (st+nu) > ModBusRegisters) // dont ask more, that we has!
            {
              mb_buf_out[mb_buf_out_count++]=mb_addr;
              mb_buf_out[mb_buf_out_count++]=func+0x80;
              mb_buf_out[mb_buf_out_count++]=2;
            }
            else
            {
              mb_buf_out[mb_buf_out_count++]=mb_addr;
              mb_buf_out[mb_buf_out_count++]=func;
              mb_buf_out[mb_buf_out_count++]=(nu)*2; // how many bytes we will send?
              for(i=st; i < st+nu ; i++)
                {
                  mb_buf_out[mb_buf_out_count++]=( mb_reg(i) >> 8 ) & 0xFF; // hi part
                  mb_buf_out[mb_buf_out_count++]=mb_reg(i) & 0xFF; // lo part
                }
            }
          break;
        case 16: 
          // write holding registers. by bytes addr func starth startl totalh totall num_bytes regh regl ...
          st=mb_buf_in[2]*256+mb_buf_in[3];
          nu=mb_buf_in[4]*256+mb_buf_in[5];
          if( (st+nu) > ModBusRegisters) // dont ask more, that we has!
            {
              mb_buf_out[mb_buf_out_count++]=mb_addr;
              mb_buf_out[mb_buf_out_count++]=func+0x80;
              mb_buf_out[mb_buf_out_count++]=2;
            }
            else
              { // ATTN : skip num_bytes
              for(i=st;i<st+nu;i++)
                {
                  mb_reg(i,mb_buf_in[7+i-st]*256+mb_buf_in[8+i-st]);
                }
              mb_buf_out[mb_buf_out_count++]=mb_addr;
              mb_buf_out[mb_buf_out_count++]=func;
              mb_buf_out[mb_buf_out_count++]=mb_buf_in[2]; // how many registers ask, so many wrote
              mb_buf_out[mb_buf_out_count++]=mb_buf_in[3];
              mb_buf_out[mb_buf_out_count++]=mb_buf_in[4];
              mb_buf_out[mb_buf_out_count++]=mb_buf_in[5];
            }
          break;
        default:  
          // Exception as we does not provide this function
          mb_buf_out[mb_buf_out_count++]=mb_addr;
          mb_buf_out[mb_buf_out_count++]=func+0x80;
          mb_buf_out[mb_buf_out_count++]=1;
          break;
      }
      
      CRC16_OUT();
      
     // If you want directly to USB-CDC 
     //CDC_Transmit_FS(&mb_buf_out[0], mb_buf_out_count);
      sendData(mb_buf_out, mb_buf_out_count);
      return true;
  //   for(int i=0;i<mb_buf_out_count;i++)
   //     {
    //      osMessagePut(ModBusOutHandle,mb_buf_out[i],0);
     //   }
    }else
    	return false;

    // Ok, we parsed buffer, clean up
    mb_buf_in_count=0;
    mb_buf_out_count=0;
}

// set value of register
void ModBus_SetRegister(uint8_t reg,uint16_t value)
{
  if(reg<ModBusRegisters)
  {
    mb_reg(reg,value);
  }
}
// grab value of register
uint16_t ModBus_GetRegister(uint8_t reg)
{
  if(reg<ModBusRegisters)
  {
    return mb_reg(reg);
  }
  return 0;
}

uint16_t CRC16(uint8_t * data, uint16_t len ){
		static const uint16_t crc16Table[]  __attribute__((section(".ccmram")))=
		    {
		        0x0000, 0xC1C0, 0x81C1, 0x4001, 0x01C3, 0xC003, 0x8002, 0x41C2,
		        0x01C6, 0xC006, 0x8007, 0x41C7, 0x0005, 0xC1C5, 0x81C4, 0x4004,
		        0x01CC, 0xC00C, 0x800D, 0x41CD, 0x000F, 0xC1CF, 0x81CE, 0x400E,
		        0x000A, 0xC1CA, 0x81CB, 0x400B, 0x01C9, 0xC009, 0x8008, 0x41C8,
		        0x01D8, 0xC018, 0x8019, 0x41D9, 0x001B, 0xC1DB, 0x81DA, 0x401A,
		        0x001E, 0xC1DE, 0x81DF, 0x401F, 0x01DD, 0xC01D, 0x801C, 0x41DC,
		        0x0014, 0xC1D4, 0x81D5, 0x4015, 0x01D7, 0xC017, 0x8016, 0x41D6,
		        0x01D2, 0xC012, 0x8013, 0x41D3, 0x0011, 0xC1D1, 0x81D0, 0x4010,
		        0x01F0, 0xC030, 0x8031, 0x41F1, 0x0033, 0xC1F3, 0x81F2, 0x4032,
		        0x0036, 0xC1F6, 0x81F7, 0x4037, 0x01F5, 0xC035, 0x8034, 0x41F4,
		        0x003C, 0xC1FC, 0x81FD, 0x403D, 0x01FF, 0xC03F, 0x803E, 0x41FE,
		        0x01FA, 0xC03A, 0x803B, 0x41FB, 0x0039, 0xC1F9, 0x81F8, 0x4038,
		        0x0028, 0xC1E8, 0x81E9, 0x4029, 0x01EB, 0xC02B, 0x802A, 0x41EA,
		        0x01EE, 0xC02E, 0x802F, 0x41EF, 0x002D, 0xC1ED, 0x81EC, 0x402C,
		        0x01E4, 0xC024, 0x8025, 0x41E5, 0x0027, 0xC1E7, 0x81E6, 0x4026,
		        0x0022, 0xC1E2, 0x81E3, 0x4023, 0x01E1, 0xC021, 0x8020, 0x41E0,
		        0x01A0, 0xC060, 0x8061, 0x41A1, 0x0063, 0xC1A3, 0x81A2, 0x4062,
		        0x0066, 0xC1A6, 0x81A7, 0x4067, 0x01A5, 0xC065, 0x8064, 0x41A4,
		        0x006C, 0xC1AC, 0x81AD, 0x406D, 0x01AF, 0xC06F, 0x806E, 0x41AE,
		        0x01AA, 0xC06A, 0x806B, 0x41AB, 0x0069, 0xC1A9, 0x81A8, 0x4068,
		        0x0078, 0xC1B8, 0x81B9, 0x4079, 0x01BB, 0xC07B, 0x807A, 0x41BA,
		        0x01BE, 0xC07E, 0x807F, 0x41BF, 0x007D, 0xC1BD, 0x81BC, 0x407C,
		        0x01B4, 0xC074, 0x8075, 0x41B5, 0x0077, 0xC1B7, 0x81B6, 0x4076,
		        0x0072, 0xC1B2, 0x81B3, 0x4073, 0x01B1, 0xC071, 0x8070, 0x41B0,
		        0x0050, 0xC190, 0x8191, 0x4051, 0x0193, 0xC053, 0x8052, 0x4192,
		        0x0196, 0xC056, 0x8057, 0x4197, 0x0055, 0xC195, 0x8194, 0x4054,
		        0x019C, 0xC05C, 0x805D, 0x419D, 0x005F, 0xC19F, 0x819E, 0x405E,
		        0x005A, 0xC19A, 0x819B, 0x405B, 0x0199, 0xC059, 0x8058, 0x4198,
		        0x0188, 0xC048, 0x8049, 0x4189, 0x004B, 0xC18B, 0x818A, 0x404A,
		        0x004E, 0xC18E, 0x818F, 0x404F, 0x018D, 0xC04D, 0x804C, 0x418C,
		        0x0044, 0xC184, 0x8185, 0x4045, 0x0187, 0xC047, 0x8046, 0x4186,
		        0x0182, 0xC042, 0x8043, 0x4183, 0x0041, 0xC181, 0x8180, 0x4040
		    };
		 uint16_t crc = 0xFFFF;
	     for (uint16_t i = 0; i < len; i++)
	         crc = (uint16_t)((crc << 8) ^ crc16Table[(crc >> 8) ^ data[i]]);

	 //uint8_t hi
	 //    crc16[0] = crc & 0xFF;
	// uint8_t lo
	 //    crc16[1]= ( crc >> 8 ) & 0xFF;
	     return crc;

 }
// Calculate CRC for outcoming buffer
// and place it to end.
void CRC16_OUT(void)
{
	uint16_t crc = CRC16(mb_buf_out,mb_buf_out_count);
	uint8_t hi = crc & 0xFF;
	uint8_t lo = ( crc >> 8 ) & 0xFF;
  
  mb_buf_out[mb_buf_out_count++] = lo;
  mb_buf_out[mb_buf_out_count++] = hi;
}

// Calculate CRC fro incoming buffer
// Return 0 - if CRC is correct, overwise return 0 
uint8_t CRC16_IN()
{

 uint16_t crc = CRC16(mb_buf_in,mb_buf_in_count-2);
 uint8_t hi = crc & 0xFF;
 uint8_t lo = ( crc >> 8 ) & 0xFF;
  if( (mb_buf_in[mb_buf_in_count-2] == lo) && 
       (mb_buf_in[mb_buf_in_count-1] == hi) )
    {
      return 0;
    }
  return 1;
}

