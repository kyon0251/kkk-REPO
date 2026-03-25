// #ifndef ROLL_CONTROL_VELOCITY_H
// #define ROLL_CONTROL_VELOCITY_H

// #include "IMU.h"
// #include "Motor.h"

// class RollControlVelocity {
// public:
//     // 构造函数：初始化IMU和电机对象
//     RollControlVelocity(H30_IMU& imu, Motor& motor, uint8_t motorAddr = 1);
    
//     // 初始化控制器
//     void init(uint32_t controlInterval = 20);
    
//     // 主控制循环更新函数
//     void update();
    
//     // 设置目标横滚角（度）
//     void setTargetRoll(float target);
    
//     // 设置横滚角限制（度）
//     void setRollLimits(float min_deg, float max_deg);
    
//     // 获取当前横滚角（度）
//     float getCurrentRoll() const;
    
//     // 获取目标横滚角（度）
//     float getTargetRoll() const;
    
//     // 设置PID参数
//     void setPIDParams(float angle_Kp, float angle_Ki, float angle_Kd,
//                     float rate_Kp, float rate_Ki, float rate_Kd);
    
//     // 紧急停止
//     void emergencyStop();

//     // 设置最大速度限制
//     void setSpeedLimits(float max_speed_rpm);

// private:
//     H30_IMU& imu_;          // IMU传感器引用
//     Motor& motor_;          // 电机控制器引用
//     uint8_t motorAddr_;     // 电机地址
    
//     // PID参数结构体
//     struct PidParams {
//         float Kp, Ki, Kd;   // PID增益
//         float integral;      // 积分项
//         float prev_error;    // 上次误差
//     };
    
//     // 控制参数
//     float target_roll_;      // 目标横滚角（度）
//     float min_roll_limit_;   // 最小横滚角限制（度）
//     float max_roll_limit_;   // 最大横滚角限制（度）
//     float max_speed_rpm_;    // 最大转速（RPM）
//     float max_acceleration_; // 最大加速度
//     uint32_t control_interval_; // 控制周期（ms）
//     uint32_t last_update_time_; // 上次更新时间
    
//     // PID控制器
//     PidParams angle_pid_;    // 角度环PID
//     PidParams rate_pid_;     // 角速度环PID
    
//     // 私有方法
//     void resetPID();                          // 重置PID状态
//     float computeAnglePID(float error, float dt); // 角度环计算
//     float computeRatePID(float error, float dt);  // 角速度环计算
//     void sendVelocityCommand(float velocity_rpm);  // 发送速度命令
// };

// #endif