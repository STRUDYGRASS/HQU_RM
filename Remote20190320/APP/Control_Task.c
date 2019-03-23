#include "main.h"
#include "Control_Task.h"
#include "CloudMotor.h"
#include "Encoder.h"
#include "stm32f4xx.h"
#include "ChassisMotor.h"
#include "shoot.h"
#include "pid.h"
#include "fric.h"
#include "delay.h"

#include "usart.h"

#include "arm_math.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

static fp32 pwm1=1000;
static fp32 count[6]= {0,0,0,0,0,0};
//todo 把遥控器数据传输改到各个模块内部
void Control_Task(RC_ctrl_t *Rc)
{
    //ch    0   1   2   3
    //通道 右x 右y 左x 左y
    if(Rc->key.v & KEY_PRESSED_OFFSET_Q)
    {
        if(count[4]>-660)
        {
            count[4]-=10;
        }
        Rc->rc.ch[0]=count[4];
    }
    else
    {
        count[4]=0;
    }
    if(Rc->key.v & KEY_PRESSED_OFFSET_E)
    {
        if(count[5]<660)
        {
            count[5]+=10;
        }
        Rc->rc.ch[0]=count[5];
    }
    else
    {
        count[5]=0;
    }

    GMYaw_Positionloop.set = (Rc->rc.ch[4]+660)*8192/1320;

    if(Rc->rc.s[1] == 1)
    {
        if(pwm1<1600)//1180)
        {
            pwm1+=2;
        }
        fric1_on(pwm1);
        fric2_on(pwm1);
    }
    else
    {
        fric1_on(800);
        fric2_on(800);
        pwm1=1000;
    }
    chassis_control_loop();
		shoot_control_loop();
    CloudMotor_Ctrl();
}
