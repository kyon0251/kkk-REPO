
#include "UG.h"



// 全局对象
H30_IMU imu;
Motor motor;
PushRod pushrod;
PitchControl pitchControl(imu, pushrod);
UG ug(imu, motor, pushrod, pitchControl);

void setup() {
    ug.init();
}

void loop() {
    ug.movePosMode(0, 122, 10000);
    ug.turnMode(45,5000,5000,2);
    //ug.motorModify(1);
}
