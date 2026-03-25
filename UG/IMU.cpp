#include "IMU.h"
#include <string.h>

H30_IMU::H30_IMU() 
    : frameRecvIndex(0), state(H30_WAITHEAD), lastRecv(0), dataReady(false),dataNum(0),
      gyroX(0), gyroY(0), gyroZ(0),
      accelX(0), accelY(0), accelZ(0),
      pitch(0), roll(0), yaw(0),
      quatW(0), quatX(0), quatY(0), quatZ(0) 
{
    memset(&frame, 0, sizeof(frame));
    memset(frameBuf, 0, sizeof(frameBuf));
}

uint16_t H30_IMU::checkSum(uint8_t* data, uint16_t len) {
    uint8_t ck1 = 0, ck2 = 0;
    for (uint16_t i = 0; i < len; i++) {
        ck1 += data[i];
        ck2 += ck1;
    }
    uint16_t ck = (ck1 << 8) | ck2;
    return ck;
}

void H30_IMU::updateProcessedData() {
    dataNum =frame.FrameNum;
    // 解析角速度（单位：根据传感器手册， deg/s）
    gyroX = (float)frame.gyro.gx * 0.000001f;
    gyroY = (float)frame.gyro.gy * 0.000001f;
    gyroZ = (float)frame.gyro.gz * 0.000001f;

    // 解析加速度（单位：可能是 m/s² 或 g）
    accelX = (float)frame.accel.ax * 0.000001f;
    accelY = (float)frame.accel.ay * 0.000001f;
    accelZ = (float)frame.accel.az * 0.000001f;

    // 解析欧拉角（单位：度）
    pitch = (float)frame.attitude.pitch * 0.000001f;
    roll  = (float)frame.attitude.roll  * 0.000001f;
    yaw   = (float)frame.attitude.yaw   * 0.000001f;

    // 解析四元数（通常已归一化）
    quatW = (float)frame.quaternion.q0 * 0.000001f;
    quatX = (float)frame.quaternion.q1 * 0.000001f;
    quatY = (float)frame.quaternion.q2 * 0.000001f;
    quatZ = (float)frame.quaternion.q3 * 0.000001f;
}

bool H30_IMU::processSerialData(uint8_t recv) {
    dataReady = false;
    
    switch (state) {
        case H30_WAITHEAD:
            if (lastRecv == 0x59 && recv == 0x53) {
                frameBuf[0] = 0x59; // 帧头
                frameBuf[1] = 0x53;
                frameRecvIndex = 2;
                state = H30_STARTRECV;
            }
            break;
            
        case H30_STARTRECV:
            frameBuf[frameRecvIndex++] = recv;
            if (frameRecvIndex == sizeof(H30FrameType_t)) {
                state = H30_CHECKSUM;
            }
            break;
            
        case H30_CHECKSUM: {
            uint16_t checksumVal = checkSum(&frameBuf[2], sizeof(H30FrameType_t) - 4); // 除去帧头和本身的校验位
            if ((checksumVal >> 8 & 0xff) == frameBuf[sizeof(H30FrameType_t) - 2] && 
                (checksumVal & 0xff) == frameBuf[sizeof(H30FrameType_t) - 1]) {
                state = H30_HANDLEDATA;
            } else {
                state = H30_WAITHEAD;
            }
            break;
        }
            
        case H30_HANDLEDATA:
            memcpy(&frame, frameBuf, sizeof(H30FrameType_t));
            updateProcessedData(); // 解析数据到浮点数
            state = H30_WAITHEAD;
            dataReady = true;
            
        default:
            break;
    }
    
    lastRecv = recv;
    return dataReady;
}

bool H30_IMU::isDataReady() const {
    return dataReady;
}

const H30FrameType_t& H30_IMU::getFrameData() const {
    return frame;
}

void H30_IMU::printData() const {
    Serial.print("DataNum:");
    Serial.println(dataNum); // 数据帧号
    Serial.print("Gyro (X,Y,Z): ");
    Serial.print(gyroX); Serial.print(", ");
    Serial.print(gyroY); Serial.print(", ");
    Serial.println(gyroZ);

    Serial.print("Accel (X,Y,Z): ");
    Serial.print(accelX); Serial.print(", ");
    Serial.print(accelY); Serial.print(", ");
    Serial.println(accelZ);

    Serial.print("Attitude (Pitch,Roll,Yaw): ");
    Serial.print(pitch); Serial.print(", ");
    Serial.print(roll);  Serial.print(", ");
    Serial.println(yaw);

    Serial.print("Quaternion (W,X,Y,Z): ");
    Serial.print(quatW); Serial.print(", ");
    Serial.print(quatX); Serial.print(", ");
    Serial.print(quatY); Serial.print(", ");
    Serial.println(quatZ);

    Serial.println();
    Serial.println();
}

bool H30_IMU::checkUARTOverflow() const {
 if (UCSR1A & (1 << DOR1)) {
   return 1;
 }
 else{
   return 0;
 }
}
////导航信息更新
//void INS_Update(){
//  uint8_t wait_for_ready = 0;//解析完成标志
//  InertialNavType_t last_NavInf = NAV_Information;//上次导航信息
//  uint32_t detT;//更新时间间隔
//  IMU_Error = checkUARTOverflow();//传输过快，缓冲区溢出
//  //接收并解析数据
//  while(Serial.available() && (!wait_for_ready))
//  {
//    wait_for_ready = H30_Callback(Serial.read());
//  }
//
//  if( wait_for_ready ) //完成数据的解析接收
//  {
//    
//    NAV_Information.gx = (float)H30Frame.gyro.gx*0.000001; //更新三轴角速度
//    NAV_Information.gy = (float)H30Frame.gyro.gy*0.000001;
//    NAV_Information.gz = (float)H30Frame.gyro.gz*0.000001;
//    
//    NAV_Information.ax = (float)H30Frame.accel.ax*0.000001;//更新三轴加速度
//    NAV_Information.ay = (float)H30Frame.accel.ay*0.000001;
//    NAV_Information.az = (float)H30Frame.accel.az*0.000001;
//    
//    NAV_Information.pitch = (float)H30Frame.attitude.pitch*0.000001;//更新欧拉角
//    NAV_Information.roll = (float)H30Frame.attitude.roll*0.000001;
//    NAV_Information.yaw = (float)H30Frame.attitude.yaw*0.000001;
//    
//
//    NAV_Information.q0 = (float)H30Frame.quaternion.q0*0.000001;//更新四元数
//    NAV_Information.q1 = (float)H30Frame.quaternion.q1*0.000001;
//    NAV_Information.q2 = (float)H30Frame.quaternion.q2*0.000001;
//    NAV_Information.q3 = (float)H30Frame.quaternion.q3*0.000001;
//
//
//    NAV_Information.timeStamp = micros(); //更新时间戳
//    detT =(NAV_Information.timeStamp - last_NavInf.timeStamp)*0.000001;//计算间隔
//
//
//    NAV_Information.vx = (NAV_Information.ax + last_NavInf.ax)/2 * detT + last_NavInf.vx;//更新线速度
//    NAV_Information.vy = (NAV_Information.ay + last_NavInf.ay)/2 * detT + last_NavInf.vy;
//    NAV_Information.vz = (NAV_Information.az + last_NavInf.az)/2 * detT + last_NavInf.vz; 
//
//
//    NAV_Information.x = (NAV_Information.vx + last_NavInf.vx)/2 * detT + last_NavInf.x;//更新位移
//    NAV_Information.y = (NAV_Information.vy + last_NavInf.vy)/2 * detT + last_NavInf.y;
//    NAV_Information.z = (NAV_Information.vz + last_NavInf.vz)/2 * detT + last_NavInf.z;  
//
//    
//  }
//
//  else{
//    IMU_Error = 2;//IMU传输频率过慢
//  }
//  
//}
