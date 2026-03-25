#pragma once
#include <Arduino.h>
#include <stdlib.h>

#define PR_PWMA 6      // 用于PWM控制
#define PR_IN1 5
#define PR_IN2 4
#define PR_STBY 7      // 数字引脚，用于开关量控制
#define PR_ENCODER 2   // 编码器采集引脚
#define PR_DIRECTION 3 // 方向判断引脚
#define PULSE_TOTAL 26715//总脉冲数
#define PULSE_LIM 21800//脉冲数限制21800 总行程122mm


class PushRod {
public:
    // 初始化推杆电机
    void init();
    
    // PWM控制调速 (-255到+255)
    void setPWM(int16_t pwm);
    
    // 移动相对当前位置的位移
    void move(int16_t displacement, int16_t pwm);

    // 移动到指定位置（毫米）
    void movetoMM(int16_t pos, int16_t pwm);
    
    // 获取当前编码器计数值
    int16_t getEncoderValue() const;
    
    // 获取错误标志
    uint8_t getError() const;

    // 获取位移完成标志
    bool getMoveFin() const;
    
    // 重置编码器计数
    void resetEncoder();

    // 重置推杆位置
    void resetPushrod(bool dir);
    //
    void printPos();
    
private:
    volatile int16_t encoderSum = 0;    // 推杆编码器计数
    int16_t encoderSumExpected = -PULSE_TOTAL;     // 推杆编码器计数期望             
    uint8_t error = 0;                  // 推杆模块错误标志
    bool moveFin = 1;                   //位移完成标志
    
    // 编码器中断处理函数
    static void handleEncoderInterrupt();
    static PushRod* instance;           // 用于中断处理的实例指针
};
