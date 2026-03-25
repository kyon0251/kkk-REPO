#include "UG.h"

UG::UG(H30_IMU& imu, Motor& motor, PushRod& pushrod, PitchControl& pitchControl) 
    : _imu(imu), _motor(motor), _pushrod(pushrod), _pitchControl(pitchControl),
      _turnStatus(0), _lastTurnSwitchTime(0), _turnCount(0), _moveStatus(0), _lastSwitchTime(0), _lastPitchControl(0) {
}

void UG::init() {
    // 初始化串口
    Serial.begin(115200);      // 调试串口
    IMU_SERIAL.begin(115200);  // IMU串口
    MOTOR_SERIAL.begin(115200); // 电机串口

    _motor.init();
    moveInit();
    turnInit();
}

void UG::moveInit() {
   _pitchControl.init();
   _pushrod.resetPushrod(0); 
   _pushrod.movetoMM(122,255);
   delay(10000);
   _pushrod.movetoMM(60,255);
   delay(5000);
   // 设置PID参数
   _pitchControl.setPIDParams(
        2.5f, 0, 0,   // 角度环PID
        10.0f, 0.1, 0     // 角速度环PID
   );
   _pitchControl.setTargetPitch(0);
}

void UG::turnInit() {
    while (IMU_SERIAL.available()) {
        _imu.processSerialData(IMU_SERIAL.read());
    }
    _motor.Origin_Set_O(1, 0);
    delay(500);
}

void UG::turnMode(uint8_t degree, uint32_t turn_interval, uint32_t setO_interval, uint8_t tCount) {
    if ((tCount != _turnCount) && (millis() - _lastTurnSwitchTime >= turn_interval) && (_turnStatus == 0)){
        _lastTurnSwitchTime = millis();
        _turnStatus = 1;
        turn(1, 0, 200, 150, 30);
    }
    if ((millis() - _lastTurnSwitchTime >= setO_interval) && (_turnStatus == 1)){
        _lastTurnSwitchTime = millis();
        _turnStatus = 0;
        turn(0, 0, 200, 150, 30);
        _turnCount++;
    }
}

void UG::turn(bool status, bool dir, uint16_t vel, uint16_t acc, float degree) {
    if (status) _motor.Pos_Control(status, dir, vel, acc, round(constrain(degree*5/0.1125, 0, 1500)), 0, 0);
    else _motor.Origin_Trigger_Return(1, 0, 0);
}

void UG::movePosMode(uint8_t posmax, uint8_t posmin, uint32_t interval) {

    if ((millis() - _lastSwitchTime) >= interval){
        _lastSwitchTime = millis();
        if (_moveStatus) _pushrod.movetoMM(posmax, 255);
        else _pushrod.movetoMM(posmin, 255);
        _moveStatus = !_moveStatus;

    }
}

void UG::movePIDMode(int8_t degreemax, int8_t degreemin, uint32_t interval, uint32_t PIDinterval) {
    if (IMU_SERIAL.available()) {
        _imu.processSerialData(IMU_SERIAL.read());
    }
    if ((millis() - _lastSwitchTime) >= interval){
        _lastSwitchTime = millis();
        if (_moveStatus) _pitchControl.setTargetPitch(degreemax);
        else _pitchControl.setTargetPitch(degreemin);
        _moveStatus = !_moveStatus;
    }
     if ((millis() - _lastPitchControl) >= PIDinterval){
        _lastPitchControl = millis();
        _pitchControl.update();
    }
    //    // 状态监测（每500ms）
    // static uint32_t lastPrint = 0;
    // if (millis() - lastPrint >= 500) {
    //     lastPrint = millis();
    //     Serial.print("Target: ");
    //     Serial.print(_pitchControl.getTargetPitch());
    //     Serial.print("° Current: ");
    //     Serial.print(_pitchControl.getCurrentPitch());
    //     Serial.println("°");
    
    // }
}


void UG::imuMotorModify(){
    if (IMU_SERIAL.available()) {
        _imu.processSerialData(IMU_SERIAL.read());
    }
    // 定时控制更新
    static uint32_t lastModify = 0;
    if (millis() - lastModify >= 5000) {
        lastModify = millis();
        float roll = _imu.getRoll();
        turn(1, constrain(roll,0,1), 20, 50, 2*roll);
        delay(1000);
        turn(1, constrain(roll,0,1), 20, 50, 2 * roll);
        delay(1000);
        turn(1, constrain(roll,0,1), 20, 50, roll);
        delay(1000);
    }
}

void UG::motorModify(bool dir){//1为顺时针
    while(1){
        turn(1, dir, 20, 10, 1);
        delay(200);
    }
}

void UG::prTest(){
    _pushrod.movetoMM(100,255);
    delay(3000);
    _pushrod.movetoMM(40,255);
    delay(3000);
}

void UG::motorTest(){
    _motor.Pos_Control(1, 0, 200, 200, 1400, 0, 0);
    delay(2000);
    _motor.Origin_Trigger_Return(1, 0, 0);
    delay(2000);
    _motor.Pos_Control(1, 1, 200, 200, 1400, 0, 0);
    delay(2000);
    _motor.Origin_Trigger_Return(1, 0, 0);
    delay(2000);
}

void UG::imuTest(){
    // 定时控制更新
    static uint32_t lastControl = 0;
    if (millis() - lastControl >= 1000) {
        lastControl = millis();
        _imu.printData();
    }
}

void UG::softwareReset() {
    void (*restFunc)(void) = 0;
    restFunc();
}