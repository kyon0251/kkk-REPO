#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <stdint.h>

//////////////////////////// H30数据格式定义 ///////////////////////////////
#pragma pack(1)
typedef struct {
    uint8_t dataId;
    uint8_t dataLen;
    int32_t ax;
    int32_t ay;
    int32_t az;
} AccelRawType_t;

typedef struct {
    uint8_t dataId;
    uint8_t dataLen;
    int32_t gx;
    int32_t gy;
    int32_t gz;
} GyroRawType_t;

typedef struct {
    uint8_t dataId;
    uint8_t dataLen;
    int32_t pitch;
    int32_t roll;
    int32_t yaw;
} AttitudeType_t;

typedef struct {
    uint8_t dataId;
    uint8_t dataLen;
    int32_t q0;
    int32_t q1;
    int32_t q2;
    int32_t q3;
} QuaternionType_t;

typedef struct {
    uint8_t head1;     // 帧头
    uint8_t head2;
    uint16_t FrameNum;  // 帧号
    uint8_t packLen;    // 数据总长

    AccelRawType_t accel;     // 数据区,新增数据时注意顺序  //14 bytes
    GyroRawType_t gyro;       //14 bytes
    AttitudeType_t attitude;  //14 bytes
    QuaternionType_t quaternion; //18 bytes

    uint8_t ck1; // 数据区校验和
    uint8_t ck2;
} H30FrameType_t;
#pragma pack()
//////////////////////////// H30数据格式定义 END///////////////////////////////

class H30_IMU {
public:
    H30_IMU();
    bool processSerialData(uint8_t recv);
    bool isDataReady() const;
    const H30FrameType_t& getFrameData() const;
    void printData() const;
    bool checkUARTOverflow() const;

    // 新增：获取解析后的数据
    uint16_t getDataNum() const { return dataNum; }

    float getGyroX() const { return gyroX; }  // 角速度 (rad/s 或 deg/s)
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }

    float getAccelX() const { return accelX; } // 加速度 (m/s² 或 g)
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }

    float getPitch() const { return pitch; }   // 欧拉角 (度或弧度)
    float getRoll() const { return roll; }
    float getYaw() const { return yaw; }

    float getQuatW() const { return quatW; }   // 四元数 (q0, q1, q2, q3)
    float getQuatX() const { return quatX; }
    float getQuatY() const { return quatY; }
    float getQuatZ() const { return quatZ; }

private:
      // H30接收函数状态机
    enum H30Serial_Status {
        H30_WAITHEAD = 0,
        H30_STARTRECV,
        H30_CHECKSUM,
        H30_HANDLEDATA
    };


    uint16_t checkSum(uint8_t* data, uint16_t len);
    void updateProcessedData(); // 新增：更新解析后的数据

    H30FrameType_t frame;
    uint8_t frameBuf[sizeof(H30FrameType_t)];
    uint16_t frameRecvIndex;
    H30Serial_Status state;
    uint8_t lastRecv;
    bool dataReady;

    // 新增：存储解析后的数据
    uint16_t dataNum;
    float gyroX, gyroY, gyroZ;     // 角速度
    float accelX, accelY, accelZ;   // 加速度
    float pitch, roll, yaw;         // 欧拉角
    float quatW, quatX, quatY, quatZ; // 四元数 (q0=W, q1=X, q2=Y, q3=Z)
};



#endif // H30_IMU_H