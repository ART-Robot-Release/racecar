
//
// Created by Steven Zhang on 18-12-14.
// art racecar
//

#ifndef ART_RACECAR_DRIVER
#define ART_RACECAR_DRIVER
#include <stdint.h>
#include <unistd.h>

#if defined(__cplusplus)
extern "C" {
#endif




int Open_Serial_Dev(char *dev);

int art_racecar_init(int speed,char *dev);//设置波特率和串口设备


unsigned char send_cmd(uint16_t motor_pwm,uint16_t servo_pwm);//发送指令(电机PWM,舵机PWM),单位为us.







#if defined(__cplusplus)
}
#endif
#endif
