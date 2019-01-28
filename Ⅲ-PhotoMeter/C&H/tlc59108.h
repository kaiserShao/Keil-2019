#ifndef TLC59108_H
#define TLC59108_H
#include "stm32f4xx_hal.h"

//SWRST CMD DATA
#define SWRST_CMD_ADDRESS      0x96
#define SWRST_CMD_BYTE1        0xA5
#define SWRST_CMD_BYTE2        0x5A


#define SLAVE_ADDR             0x92         
#define CONTRAL_REGISTER       0xA2        //
//Register value
#define PWM_REGISTER0          220
#define PWM_REGISTER1          75
#define PWM_REGISTER2          100
#define PWM_REGISTER3          125
#define PWM_REGISTER4          150
#define PWM_REGISTER5          175
#define PWM_REGISTER6          200
#define PWM_REGISTER7          225
uint8_t LED_PWM( uint8_t LDT1, uint8_t LDT2, uint8_t LDT3, uint8_t LDT4 )
#endif
