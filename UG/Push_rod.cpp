#include "Push_rod.h"

PushRod* PushRod::instance = nullptr;

void PushRod::init() {
    pinMode(PR_IN1, OUTPUT);
    pinMode(PR_IN2, OUTPUT);
    pinMode(PR_STBY, OUTPUT);
    pinMode(PR_PWMA, OUTPUT);

    digitalWrite(PR_IN1, 1);
    digitalWrite(PR_IN2, 0);
    digitalWrite(PR_STBY, 1);
    analogWrite(PR_PWMA, 0);
    
    pinMode(PR_ENCODER, INPUT);
    pinMode(PR_DIRECTION, INPUT);
    
    // 设置实例指针用于中断处理
    instance = this;
    attachInterrupt(digitalPinToInterrupt(PR_ENCODER), handleEncoderInterrupt, RISING);
}

void PushRod::setPWM(int16_t pwm) {
    if(pwm < 0) {
        digitalWrite(PR_IN1, 1);
        digitalWrite(PR_IN2, 0);
        analogWrite(PR_PWMA, -pwm);
    } else {
        digitalWrite(PR_IN1, 0);
        digitalWrite(PR_IN2, 1);
        analogWrite(PR_PWMA, pwm);
    }
}

void PushRod::move(int16_t displacement, int16_t pwm) {
    encoderSumExpected = encoderSum + displacement;
    moveFin = 0;
    if (displacement >= 0) {
        setPWM(pwm);
    } else {
        setPWM(-pwm);
    }
}
void PushRod::movetoMM(int16_t pos, int16_t pwm){
    encoderSumExpected = round((pos / 150.0) * PULSE_TOTAL);//折算脉冲数，行程150mm
    moveFin = 0;
    if ((encoderSumExpected - encoderSum)>= 0) {
        setPWM(pwm);
    } else {
        setPWM(-pwm);
    }
}

int16_t PushRod::getEncoderValue() const {
    return encoderSum;
}

uint8_t PushRod::getError() const {
    return error;
}

bool PushRod::getMoveFin() const {
    return moveFin;
}

void PushRod::resetEncoder() {
    encoderSum = 0;
}
 
void PushRod::resetPushrod(bool dir) {
    moveFin = 0;
    if (dir) setPWM(255);
    else setPWM(-255);
    encoderSum = -1;//防止直接跳出循环
    while(abs(encoderSum)) {
        encoderSum = 0;
        delay(100);
    }
    setPWM(0);
    encoderSum = 0;
    moveFin = 1;
    Serial.println("重置推杆完成!");
}

void PushRod::printPos() {
    float PR_Pos = encoderSum * 150.0 / PULSE_TOTAL;
    Serial.print("目前位移");
    Serial.println(PR_Pos);
}

void PushRod::handleEncoderInterrupt() {
    if (instance) {
        if (digitalRead(PR_DIRECTION)) {
            instance->encoderSum--;
        } else {
            instance->encoderSum++;
        }
        
        if ((instance->encoderSum == instance->encoderSumExpected)  || (instance->encoderSum > PULSE_LIM)) {
            instance->setPWM(0);
            instance->moveFin = 1;
            instance->encoderSumExpected = -26715;
        }
    }
}