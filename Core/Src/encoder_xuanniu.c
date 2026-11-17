#include "encoder_xuanniu.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "VOFA.h"
#include "PID.h"
 int32_t ReadKnobCcr(void){
  static int16_t cnt = 0;   //必须保留 static：计数器清零后靠它累计旋钮位置 
  cnt += (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  __HAL_TIM_SET_COUNTER(&htim3, 0);   //每次读完清零，下一拍拿到的才是增量 
   
   
  if (cnt > 100)
    cnt = 100;
  else if (cnt < -100)
    cnt = -100;
  return (int32_t)cnt * 3600 / 100;   // ±100 格 → ±3600 占空比刻度 
  
  
  
  
}


