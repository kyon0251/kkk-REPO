// #include "Roll_control.h"
// #include <Arduino.h>

// // 构造函数
// RollControlVelocity::RollControlVelocity(H30_IMU& imu, Motor& motor, uint8_t motorAddr)
//     : imu_(imu), motor_(motor), motorAddr_(motorAddr),
//       target_roll_(0), min_roll_limit_(-180.0f), max_roll_limit_(180.0f),
//       max_speed_rpm_(300.0f), max_acceleration_(50.0f), control_interval_(20) {
//     resetPID(); // 初始化PID参数
// }

// // 初始化控制器
// void RollControlVelocity::init(uint32_t controlInterval) {
//     control_interval_ = controlInterval;
//     motor_.begin(115200); // 初始化电机串口
//     motor_.En_Control(motorAddr_, true, false); // 使能电机
//     motor_.Modify_Ctrl_Mode(motorAddr_, true, 2); // 设置为闭环速度模式
//     resetPID();
// }

// // 主控制循环
// void RollControlVelocity::update() {
//     // 计算时间间隔（秒）
//     uint32_t now = micros();
//     float dt = (now - last_update_time_) * 1e-6f;
//     if (dt < 0.001f) return; // 防止时间间隔过小
//     last_update_time_ = now;

//     // 获取当前状态
//     float current_roll = imu_.getRoll();
//     float current_rate = imu_.getGyroX();

//     // 计算角度误差并限制在[-180,180]度范围内
//     float angle_error = target_roll_ - current_roll;
//     if (angle_error > 180.0f) angle_error -= 360.0f;
//     if (angle_error < -180.0f) angle_error += 360.0f;

//     // 外环PID计算（角度环）
//     float rate_target = computeAnglePID(angle_error, dt);
    
//     // 内环PID计算（角速度环）
//     float output_vel = computeRatePID(rate_target - current_rate, dt);
    
//     // 限制输出速度
//     output_vel = constrain(output_vel, -max_speed_rpm_, max_speed_rpm_);
    
//     // 发送速度命令
//     sendVelocityCommand(output_vel);
// }

// // 发送速度命令到电机
// void RollControlVelocity::sendVelocityCommand(float velocity_rpm) {
//     uint8_t direction = (velocity_rpm >= 0) ? 0 : 1; // 0=CW, 1=CCW
//     uint16_t speed = static_cast<uint16_t>(fabs(velocity_rpm));
    
//     motor_.Vel_Control(
//         motorAddr_, 
//         direction,
//         speed,
//         static_cast<uint8_t>(max_acceleration_), // 加速度
//         false       // 不同步
//     );
// }

// // PID参数设置
// void RollControlVelocity::setPIDParams(float angle_Kp, float angle_Ki, float angle_Kd,
//                              float rate_Kp, float rate_Ki, float rate_Kd) {
//     angle_pid_ = {angle_Kp, angle_Ki, angle_Kd, 0, 0};
//     rate_pid_ = {rate_Kp, rate_Ki, rate_Kd, 0, 0};
//     resetPID();
// }

// // 角度环PID计算
// float RollControlVelocity::computeAnglePID(float error, float dt) {
//     angle_pid_.integral += error * dt;
//     angle_pid_.integral = constrain(angle_pid_.integral, -15.0f, 15.0f); // 积分限幅
//     float derivative = (error - angle_pid_.prev_error) / dt;
//     angle_pid_.prev_error = error;
    
//     // 输出为角速度目标值(度/秒)
//     return constrain(
//         angle_pid_.Kp * error + angle_pid_.Ki * angle_pid_.integral + angle_pid_.Kd * derivative, 
//         -30.0f, 30.0f  // 限制角度环输出
//     );
// }

// // 角速度环PID计算
// float RollControlVelocity::computeRatePID(float error, float dt) {
//     rate_pid_.integral += error * dt;
//     rate_pid_.integral = constrain(rate_pid_.integral, -10.0f, 10.0f); // 积分限幅
//     float derivative = (error - rate_pid_.prev_error) / dt;
//     rate_pid_.prev_error = error;
    
//     // 输出为电机转速(RPM)
//     return rate_pid_.Kp * error + rate_pid_.Ki * rate_pid_.integral + rate_pid_.Kd * derivative;
// }

// // 重置PID状态
// void RollControlVelocity::resetPID() {
//     angle_pid_.integral = angle_pid_.prev_error = 0;
//     rate_pid_.integral = rate_pid_.prev_error = 0;
// }

// void RollControlVelocity::setTargetRoll(float target) {
//     // 将目标角度限制在指定范围内
//     target_roll_ = constrain(target, min_roll_limit_, max_roll_limit_);
// }

// void RollControlVelocity::setRollLimits(float min_deg, float max_deg) {
//     min_roll_limit_ = min_deg;
//     max_roll_limit_ = max_deg;
// }

// float RollControlVelocity::getCurrentRoll() const { 
//     return imu_.getRoll(); 
// }

// float RollControlVelocity::getTargetRoll() const { 
//     return target_roll_; 
// }

// void RollControlVelocity::setSpeedLimits(float max_speed_rpm) {
//     max_speed_rpm_ = max_speed_rpm;
// }

// void RollControlVelocity::emergencyStop() { 
//     motor_.Stop_Now(motorAddr_, false); 
// }