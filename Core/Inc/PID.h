#ifndef __PID_H__
#define __PID_H__



typedef struct {
  volatile float SetSpeed;//目标值
  volatile float ActualSpeed;//代码算出来的值，即实际值
  volatile float err;//k次的偏差
  volatile float err_last;//k-1次的偏差
  volatile float err_PrevError;//k-2次的偏差(增量式公式要用到)
  volatile float Kp,Ki,Kd;//比例系数；积分系数；微分系数
  volatile float voltage;//PID输出的量
  volatile float integral;//累计偏差，用来处理积分的
}_pid;

extern _pid pid;
#define dt 10


void PID_init();
void PID_realize(void);

void PID_Change(void);


#endif
