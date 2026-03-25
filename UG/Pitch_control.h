#ifndef PITCH_CONTROL_H
#define PITCH_CONTROL_H

#include "IMU.h"
#include "Push_rod.h"

class PitchControl {
public:
    PitchControl(H30_IMU& imu, PushRod& pushrod);
    
    void init();
    void update();  // 在定时中断中调用
    void setTargetPitch(float target);
    float getCurrentPitch() const;
    float getTargetPitch() const;
    void setPIDParams(float angle_Kp, float angle_Ki, float angle_Kd,
                     float rate_Kp, float rate_Ki, float rate_Kd);
    
private:
    H30_IMU& imu_;
    PushRod& pushrod_;
    
    // PID参数
    struct {
        float Kp, Ki, Kd;
        float integral;
        float prev_error;
    } angle_pid_, rate_pid_;
    
    float target_pitch_;  // 目标俯仰角(度)
    float max_output_;    // 最大输出限制
    uint32_t last_update_time_;
    
    void resetPID();
    float computeAnglePID(float current_angle, float dt);
    float computeRatePID(float current_rate, float dt);
};

#endif