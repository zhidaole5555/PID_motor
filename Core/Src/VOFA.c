#include "VOFA.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include <string.h>
#include "usart.h"
#include "PID.h"




float VofaGet(void){
  int16_t data=(int16_t)__HAL_TIM_GET_COUNTER(&htim4);//顺序不能变，因为先float再int16_t会超范围，用int是保证有+有-，用16是因为ARR是uint16_t
  __HAL_TIM_SET_COUNTER(&htim4,0);//重置为0，因为想要的是10ms里面转过的1角度，即速度
  
  float rpm = (float)((int32_t)data) * 60000.0f
     / (ENCODER_PPR * ENCODER_MULT * dt);//将10ms的脉冲数转换为1分钟的转速

  
  return rpm;
}
