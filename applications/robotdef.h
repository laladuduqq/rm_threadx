#ifndef __ROBOTDEF_H
#define __ROBOTDEF_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"{
#endif


/* 机器人类型定义 */
#define ROBOT_TYPE  7  // 这里可以根据需要修改为其他机器人类型 //1 HERO 2 ENGINEER 3 INFANTRY 6 DRONE 7 SENTRY

#if ROBOT_TYPE == 1
#define HERO_MODE
#elif ROBOT_TYPE == 2
#define ENGINEER_MODE
#elif ROBOT_TYPE == 3
#define INFANTRY_MODE
#elif ROBOT_TYPE == 6
#define DRONE_MODE
#elif ROBOT_TYPE == 7
#define SENTRY_MODE
#else
#error "Please define the robot type."
#endif


/* 机器人通讯定义*/

#define CONTROL_SOURCE 1  //dt7遥控器为0 sbus遥控器为1，图传0304为2，图传遥控器为3

//单板与多板定义
//#define ONE_BOARD // 单板控制整车

#ifndef ONE_BOARD // 多板控制整车 （注意只能有一个生效）
    #if defined (ENGINEER_MODE) || defined (INFANTRY_MODE) || defined (SENTRY_MODE) || defined (HERO_MODE)
        //#define CHASSIS_BOARD //底盘板
        #define GIMBAL_BOARD  //云台板

        #define GIMBAL_ID 0X310
        #define CHASSIS_ID 0X311
        // 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
        #if (defined(CHASSIS_BOARD) + defined(GIMBAL_BOARD)!=1)
        #error Conflict board definition! You can only define one board type.
        #endif
    #endif
#endif

/* 机器人关键参数定义 ,注意根据不同机器人进行修改 */  //1 表示开启 0 表示关闭



//裁判系统
#define USING_REFREE_SYSTEM 1
//功率控制开关与超电开关
#define USING_POWER_CTRL 1
#define USING_SUPER_CAP  0

//机器人参数定义
#if defined (HERO_MODE) || defined (ENGINEER_MODE) || defined (INFANTRY_MODE)

    // 云台参数        
    #define YAW_CHASSIS_ALIGN_ECD 6357  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
    #define YAW_ECD_GREATER_THAN_4096 1 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
    #define PITCH_HORIZON_ANGLE 0.0f        // 云台处于水平位置时角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
    #define PITCH_MAX_ANGLE 10.0f           // 云台竖直方向最大角度   (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
    #define PITCH_MIN_ANGLE -20.0f          // 云台竖直方向最小角度   (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

    // 发射参数
    #define ONE_BULLET_DELTA_ANGLE 8192    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
    #define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比(36.0f),英雄需要修改为3508的19.0f
    #define NUM_PER_CIRCLE 6             // 拨盘一圈的装载量
    // 机器人底盘修改的参数,单位为mm(毫米)
    #define CHASSIS_TYPE 2               // 1 麦克纳姆轮底盘 2 全向轮底盘 3 舵轮底盘 4 平衡底盘
    #define WHEEL_BASE 202.5f              // 纵向轴距(前进后退方向)
    #define TRACK_WIDTH 202.5f             // 横向轮距(左右平移方向)
    #define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
    #define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
    #define RADIUS_WHEEL 76             // 轮子半径
    #define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

    #define GYRO_TO_GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
    #define GYRO_TO_GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
    #define GYRO_TO_GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

#endif 

#if defined (DRONE_MODE)
    // 云台参数        
    #define YAW_MAX_ANGLE_ECD -150.0f
    #define YAW_MIN_ANGLE_ECD 50.0f  
    #define YAW_HORIZON_ECD 3412        // 云台处于水平位置时编码器值,若对云台有机械改动需要修改   
    #define YAW_HORIZON_ANGLE (YAW_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // 对齐时的角度,0-360     
    #define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
    #define PITCH_MAX_ANGLE 20.0f       // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
    #define PITCH_MIN_ANGLE -20.0f      // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
    #define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360
    // 发射参数
    #define ONE_BULLET_DELTA_ANGLE 8192    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
    #define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比(36.0f),英雄需要修改为3508的19.0f
    #define NUM_PER_CIRCLE 6             // 拨盘一圈的装载量

#endif 

#if defined (SENTRY_MODE)
    #define SMALL_YAW_ALIGN_ANGLE 0.0f
    #define SMALL_YAW_MIN_ANGLE -90.0f
    #define SMALL_YAW_MAX_ANGLE 90.0f
    #define SMALL_YAW_PITCH_HORIZON_ANGLE 0.0f     // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
    #define SMALL_YAW_PITCH_MAX_ANGLE 20.0f           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
    #define SMALL_YAW_PITCH_MIN_ANGLE -25.0f           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

    #define YAW_CHASSIS_ALIGN_ECD 4088  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
    #define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
    #define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
    // 发射参数
    #define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
    #define ONE_BULLET_DELTA_ANGLE 60.0f   // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
    #define NUM_PER_CIRCLE 6             // 拨盘一圈的装载量
    // 机器人底盘修改的参数,单位为mm(毫米)
    #define CHASSIS_TYPE 2               // 1 麦克纳姆轮底盘 2 全向轮底盘 3 舵轮底盘 4 平衡底盘
    #define WHEEL_R  500                    //投影点距离地盘中心为r
    #define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
    #define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
    #define RADIUS_WHEEL 0.07             // 轮子半径(单位:m)
    #define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

    #define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
    #define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
    #define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

#endif

#pragma pack(1)

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_KEEPING_BIG_YAW, //云台保持模式，检录和debug用
    GIMBAL_KEEPING_SMALL_YAW, //云台保持模式，检录和debug用
    GIMBAL_AUTO_MODE,  //自瞄导航模式
} gimbal_mode_e;


// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{   // 云台角度控制
    float yaw;
#if defined (SENTRY_MODE)
    float small_yaw;//针对哨兵大小yaw
#endif
    float pitch;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct
{
  float yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;


// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;
typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    uint16_t rest_heat;
    uint8_t shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_FOLLOW_GIMBAL_YAW, // 底盘跟随云台
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_ROTATE_REVERSE,    // 小陀螺模式反转
    CHASSIS_AUTO_MODE,         // 导航模式
} chassis_mode_e;

// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;

} Chassis_Ctrl_Cmd_s;

typedef struct
{
    uint8_t Robot_Color;
    uint16_t projectile_allowance_17mm;  //剩余发弹量
    uint8_t power_management_shooter_output; // 功率管理 shooter 输出
    uint16_t current_hp_percent; // 机器人当前血量百分比
    uint16_t outpost_HP;     //前哨站血量
    uint16_t base_HP;        //基地血量
    uint8_t game_progess;
    uint16_t game_time;
} Chassis_referee_Upload_Data_s;

typedef struct
{
    float vx;
    float vy;//真实速度
    float wz;
} Chassis_Upload_Data_s;

#pragma pack()

#ifdef __cplusplus
}
#endif

#endif // ROBOTDEF_H
