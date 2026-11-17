#include <stdio.h>
#include "PID.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "VOFA.h"
#include "encoder_xuanniu.h"
#include "tim.h"
#include <string.h>


#define PID_I_OUT_MAX 3500                 /* 积分项贡献 */
#define PID_I_MAX (PID_I_OUT_MAX / pid.Ki) /* 动态算，改 Ki 不用改这里 */
#define PID_SPEED_MAX 4000
#define PID_ERR_DEADBAND 50.0f /* 误差死区：一个测速台阶(50rpm)，以内视为0 */
// 定义变量：全局只有这一份，分配实际内存
_pid pid = {0};
static char cmd_buf[16];
static uint8_t cmd_len = 0;


void PID_init() { //初始化PID
  pid.SetSpeed = 0.0;
  pid.ActualSpeed = 0.0;
  pid.err = 0.0;
  pid.err_last = 0.0;
  pid.voltage = 0.0;
  pid.integral = 0.0;
  pid.err_PrevError = 0.0;
  pid.Kp = 1.0;
  pid.Ki = 0.0;
  pid.Kd = 0.0;
}

void PID_realize(void) {
  pid.SetSpeed =
    ReadKnobCcr() / 3600.0f * PID_SPEED_MAX; //旋钮占空比*最大转速=目标转速，单位是r/min
  pid.ActualSpeed = VofaGet();               //拿到实际转速
  pid.err = pid.SetSpeed - pid.ActualSpeed;  //err的单位是一分钟的圈数

  pid.voltage =
    pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last) / dt; //要r/min
  pid.err_last = pid.err;

  if (pid.voltage > 3600.0f) {
    pid.voltage = 3600.0f;
  }
  if (pid.voltage < -3600.0f) {
    pid.voltage = -3600.0f;
  }


  //限制积分大小的两个玩意——积分限幅和条件积分，一般两个要一起用，能多一道保险


  if (pid.Ki * pid.integral >= PID_I_OUT_MAX &&
      pid.err > 0) /*条件积分，避免积分项Ki*pid.integral所贡献的占空比过大，即避免积分溢出*/
  {
  } else if (pid.Ki * pid.integral <= -PID_I_OUT_MAX && pid.err < 0) {
  } else if (pid.voltage >= 3600.0f) {
  } else if (pid.voltage <= -3600.0f) {
  } else {
    pid.integral += pid.err * dt;
  } //这个*dt能使得无论采样周期是几ms，pid.integral的值相同，使得程序更具有普遍性，到时候调Pi就行了

  if (pid.integral > PID_I_MAX)
    pid.integral = PID_I_MAX; /*积分限幅*/
  if (pid.integral < -PID_I_MAX)
    pid.integral = -PID_I_MAX;


  if (pid.voltage >= 0) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pid.voltage); //正转
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
  } else if (pid.voltage < 0) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); //反转
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -pid.voltage);
  }
  uint8_t a[12];
  static const uint8_t end[] = {0x00, 0x00, 0x80, 0x7F};
  memcpy(&a[0], (const void *)&pid.ActualSpeed, 4);
  memcpy(&a[4], (const void *)&pid.SetSpeed, 4);
  memcpy(&a[8], end, 4);
  HAL_UART_Transmit(&huart2, a, sizeof(a), 3);
}


/*下面这两个函数是为了在VOFA里面直接改PID*/
static void Cmd_Parse(char *s) {
  char c;
  float v;

  if (sscanf(s, " %c%f", &c, &v) == 2) //因为要把s分配给两个变量，所以要==2
  {
    switch (c) {
    case 'P':
      pid.Kp = v;
      break;
    case 'I':
      pid.Ki = v;
      pid.integral = 0;
      break;
    case 'D':
      pid.Kd = v;
      break;
    }
  } else if (s[0] == 'R') { /* 单独一个 R：清积分 */
    pid.integral = 0.0f;
  }
}

void PID_Change(void) {
  uint8_t ch;

  while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) /* 超时 0，不阻塞 */
  {
    if (ch == '\n' || ch == '\r') //如果是换行符
    {
      cmd_buf[cmd_len] = '\0';
      if (cmd_len > 0)
        Cmd_Parse(cmd_buf);
      cmd_len = 0;
    } else if (cmd_len < sizeof(cmd_buf) - 1) //如果是普通数字，同时数组还有空位
    {
      cmd_buf[cmd_len++] = ch;
    }
  }
}
