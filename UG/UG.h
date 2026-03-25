#ifndef UG_H
#define UG_H

#include "Pitch_control.h"
#include "Push_rod.h"
#include "Motor.h"
#include "IMU.h"

// 硬件定义
#define IMU_SERIAL Serial1
#define MOTOR_SERIAL Serial2

class UG {
public:
    UG(H30_IMU& imu, Motor& motor, PushRod& pushrod, PitchControl& pitchControl);
    
    void init();
    
    void movePosMode(uint8_t posmax, uint8_t posmin, uint32_t interval);
    void movePIDMode(int8_t degreemax, int8_t degreemin, uint32_t interval, uint32_t PIDinterval);
    void turnMode(uint8_t degree, uint32_t turn_interval, uint32_t setO_interval, uint8_t tCount);
    void imuMotorModify();
    void motorModify(bool dir);
    void prTest();
    void motorTest();
    void imuTest();
    void softwareReset();

private:
    H30_IMU& _imu;
    Motor& _motor;
    PushRod& _pushrod;
    PitchControl& _pitchControl;
    
    bool _turnStatus; // 0为回正，1为转向
    uint32_t _lastTurnSwitchTime;
    uint8_t _turnCount;//转弯次数
    bool _moveStatus; // 0为下潜，1为上浮
    uint32_t _lastSwitchTime;
    uint32_t _lastPitchControl;

    void turn(bool status, bool dir, uint16_t vel, uint16_t acc, float degree);
    void moveInit();
    void turnInit();
};

#endif