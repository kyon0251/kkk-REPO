#include "Pitch_control.h"
#include <Arduino.h>

PitchControl::PitchControl(H30_IMU& imu, PushRod& pushrod)
    : imu_(imu), pushrod_(pushrod), target_pitch_(0), max_output_(255) {
    // 默认PID参数
    angle_pid_ = {2.0, 0.05, 0.5, 0, 0};
    rate_pid_ = {0.8, 0.02, 0.1, 0, 0};
    last_update_time_ = micros();
}

void PitchControl::init() {
    resetPID();
    pushrod_.init();
}

void PitchControl::update() {
    // 获取当前时间并计算时间间隔(秒)
    uint32_t now = micros();
    float dt = (now - last_update_time_) * 1e-6f;
    last_update_time_ = now;
    
    // 确保IMU数据已更新
    // if (!imu_.isDataReady()) {
    //     return;
    // }
    
    // 获取当前俯仰角和角速度
    float current_pitch = imu_.getPitch();
    float current_rate = imu_.getGyroY();  // Y轴角速度对应俯仰
    
    // 外环PID计算 - 角度控制
    float rate_target = computeAnglePID(current_pitch, dt);
    
    // 内环PID计算 - 角速度控制
    float output = computeRatePID(current_rate - rate_target, dt);
    
    // 限制输出范围
    output = constrain(output, -max_output_, max_output_);
    
    // 控制推杆
    if (abs(output) > 50) {  // 死区控制
        pushrod_.setPWM(static_cast<int16_t>(output));
    } else {
        pushrod_.setPWM(0);
    }
}

void PitchControl::setTargetPitch(float target) {
    target_pitch_ = target;
}

float PitchControl::getCurrentPitch() const {
    return imu_.getPitch();
}

float PitchControl::getTargetPitch() const {
    return target_pitch_;
}

void PitchControl::setPIDParams(float angle_Kp, float angle_Ki, float angle_Kd,
                               float rate_Kp, float rate_Ki, float rate_Kd) {
    angle_pid_.Kp = angle_Kp;
    angle_pid_.Ki = angle_Ki;
    angle_pid_.Kd = angle_Kd;
    
    rate_pid_.Kp = rate_Kp;
    rate_pid_.Ki = rate_Ki;
    rate_pid_.Kd = rate_Kd;
    
    resetPID();
}

void PitchControl::resetPID() {
    angle_pid_.integral = 0;
    angle_pid_.prev_error = 0;
    rate_pid_.integral = 0;
    rate_pid_.prev_error = 0;
}

float PitchControl::computeAnglePID(float current_angle, float dt) {
    float error = target_pitch_ - current_angle;
    
    // 比例项
    float P = angle_pid_.Kp * error;
    
    // 积分项(带抗饱和)
    angle_pid_.integral += error * dt;
    angle_pid_.integral = constrain(angle_pid_.integral, -10.0f, 10.0f);  // 限制积分项
    float I = angle_pid_.Ki * angle_pid_.integral;
    
    // 微分项
    float D = angle_pid_.Kd * (error - angle_pid_.prev_error) / dt;
    angle_pid_.prev_error = error;
    
    // 返回目标角速度(度/秒)
    return constrain(P + I + D, -20.0f, 20.0f);  // 限制目标角速度
}

float PitchControl::computeRatePID(float rate_error, float dt) {
    // 比例项
    float P = rate_pid_.Kp * rate_error;
    
    // 积分项(带抗饱和)
    rate_pid_.integral += rate_error * dt;
    rate_pid_.integral = constrain(rate_pid_.integral, -5.0f, 5.0f);
    float I = rate_pid_.Ki * rate_pid_.integral;
    
    // 微分项
    float D = rate_pid_.Kd * (rate_error - rate_pid_.prev_error) / dt;
    rate_pid_.prev_error = rate_error;
    
    // 返回控制输出
    return P + I + D;
}