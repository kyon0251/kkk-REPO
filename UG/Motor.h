#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// 系统参数枚举
typedef enum {
  S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
  S_RL    = 1,      /* 读取读取相电阻和相电感 */
  S_PID   = 2,      /* 读取PID参数 */
  S_VBUS  = 3,      /* 读取总线电压 */
  S_CPHA  = 5,      /* 读取相电流 */
  S_ENCL  = 7,      /* 读取经过线性化校准后的编码器值 */
  S_TPOS  = 8,      /* 读取电机目标位置角度 */
  S_VEL   = 9,      /* 读取电机实时转速 */
  S_CPOS  = 10,     /* 读取电机实时位置角度 */
  S_PERR  = 11,     /* 读取电机位置误差角度 */
  S_FLAG  = 13,     /* 读取使能/到位/堵转状态标志位 */
  S_Conf  = 14,     /* 读取驱动参数 */
  S_State = 15,     /* 读取系统状态参数 */
  S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
} SysParams_t;

class Motor {
public:    
    // 电机控制函数
    void init();
    void sendPosDegree(uint8_t degree, bool dir);
    static void Reset_CurPos_To_Zero(uint8_t addr);
    static void Reset_Clog_Pro(uint8_t addr);
    static void Read_Sys_Params(uint8_t addr, SysParams_t s);
    static void Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);
    static void En_Control(uint8_t addr, bool state, bool snF);
    static void Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
    static void Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
    static void Stop_Now(uint8_t addr, bool snF);
    static void Synchronous_motion(uint8_t addr);
    static void Origin_Set_O(uint8_t addr, bool svF);
    static void Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
    static void Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
    static void Origin_Interrupt(uint8_t addr);
    
    // 数据接收函数
    static void Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);
    
    // 辅助宏定义
    #define ABS(x) ((x) > 0 ? (x) : -(x))

private: 
    float _degrees_per_step;//每个脉冲对应的角度
    int8_t _degree_lim;//最大角度约束
};

#endif // MOTOR_H