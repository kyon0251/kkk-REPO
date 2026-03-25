#include "Motor.h"
#include <Arduino.h>


/**
  * @brief    将电机初始化
  * @param    
  * @retval   
  */
void Motor::init() {
    _degrees_per_step = 0.1125;
    _degree_lim = 180;
    Origin_Set_O(1, 0);
}
/**
  * @brief    令电机转过给定的角度
  * @param    
  * @retval   
  */
void Motor::sendPosDegree(uint8_t degree, bool dir){
  Pos_Control(1, dir, 300, 50, constrain(round(degree / _degrees_per_step), 0, (_degree_lim / _degrees_per_step)), false, false);
}
/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Reset_CurPos_To_Zero(uint8_t addr) {
    uint8_t cmd[4] = {addr, 0x0A, 0x6D, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}
/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Reset_Clog_Pro(uint8_t addr) {
    uint8_t cmd[4] = {addr, 0x0E, 0x52, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}
/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Read_Sys_Params(uint8_t addr, SysParams_t s) {
    uint8_t cmd[4] = {0};
    uint8_t i = 0;
    
    cmd[i++] = addr;
    
    switch(s) {
        case S_VER  : cmd[i++] = 0x1F; break;
        case S_RL   : cmd[i++] = 0x20; break;
        case S_PID  : cmd[i++] = 0x21; break;
        case S_VBUS : cmd[i++] = 0x24; break;
        case S_CPHA : cmd[i++] = 0x27; break;
        case S_ENCL : cmd[i++] = 0x31; break;
        case S_TPOS : cmd[i++] = 0x33; break;
        case S_VEL  : cmd[i++] = 0x35; break;
        case S_CPOS : cmd[i++] = 0x36; break;
        case S_PERR : cmd[i++] = 0x37; break;
        case S_FLAG : cmd[i++] = 0x3A; break;
        case S_ORG  : cmd[i++] = 0x3B; break;
        case S_Conf : cmd[i++] = 0x42; cmd[i++] = 0x6C; break;
        case S_State: cmd[i++] = 0x43; cmd[i++] = 0x7A; break;
        default: break;
    }
    
    cmd[i++] = 0x6B;
    Serial2.write(cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode) {
    uint8_t cmd[6] = {addr, 0x46, 0x69, (uint8_t)svF, ctrl_mode, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::En_Control(uint8_t addr, bool state, bool snF) {
    uint8_t cmd[6] = {addr, 0xF3, 0xAB, (uint8_t)state, (uint8_t)snF, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}
/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF) {
    uint8_t cmd[8] = {
        addr, 0xF6, dir, 
        (uint8_t)(vel >> 8), (uint8_t)(vel >> 0), 
        acc, (uint8_t)snF, 0x6B
    };
    Serial2.write(cmd, sizeof(cmd));
}
/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)

 {
    uint8_t cmd[13] = {
        addr, 0xFD, dir, 
        (uint8_t)(vel >> 8), (uint8_t)(vel >> 0), 
        acc,
        (uint8_t)(clk >> 24), (uint8_t)(clk >> 16), 
        (uint8_t)(clk >> 8), (uint8_t)(clk >> 0),
        (uint8_t)raF, (uint8_t)snF, 0x6B
    };
    Serial2.write(cmd, sizeof(cmd));
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Stop_Now(uint8_t addr, bool snF) {
    uint8_t cmd[5] = {addr, 0xFE, 0x98, (uint8_t)snF, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Synchronous_motion(uint8_t addr) {
    uint8_t cmd[4] = {addr, 0xFF, 0x66, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Origin_Set_O(uint8_t addr, bool svF) {
    uint8_t cmd[5] = {addr, 0x93, 0x88, (uint8_t)svF, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}
/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, 
                                uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, 
                                uint16_t sl_ma, uint16_t sl_ms, bool potF) {
    uint8_t cmd[20] = {
        addr, 0x4C, 0xAE, (uint8_t)svF, o_mode, o_dir,
        (uint8_t)(o_vel >> 8), (uint8_t)(o_vel >> 0),
        (uint8_t)(o_tm >> 24), (uint8_t)(o_tm >> 16), 
        (uint8_t)(o_tm >> 8), (uint8_t)(o_tm >> 0),
        (uint8_t)(sl_vel >> 8), (uint8_t)(sl_vel >> 0),
        (uint8_t)(sl_ma >> 8), (uint8_t)(sl_ma >> 0),
        (uint8_t)(sl_ms >> 8), (uint8_t)(sl_ms >> 0),
        (uint8_t)potF, 0x6B
    };
    Serial2.write(cmd, sizeof(cmd));
}
/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF) {
    uint8_t cmd[5] = {addr, 0x9A, o_mode, (uint8_t)snF, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Motor::Origin_Interrupt(uint8_t addr) {
    uint8_t cmd[4] = {addr, 0x9C, 0x48, 0x6B};
    Serial2.write(cmd, sizeof(cmd));
}
/**
  * @brief    接收数据
  * @param    rxCmd   : 接收到的数据缓存在该数组
  * @param    rxCount : 接收到的数据长度
  * @retval   无
  */
void Motor::Receive_Data(uint8_t *rxCmd, uint8_t *rxCount) {
    int i = 0;
    unsigned long lTime = millis();
    unsigned long cTime = lTime;

    while(1) {
        if(Serial2.available() > 0) {
            if(i <= 128) {
                rxCmd[i++] = Serial2.read();
                lTime = millis();
            }
        } else {
            cTime = millis();
            if((int)(cTime - lTime) > 100) {
                *rxCount = i;
                break;
            }
        }
    }
}