#ifndef __CAMERA_PLATFORM_H__
#define __CAMERA_PLATFORM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h"


// PWM limits
#define PWM_MIN 500
#define PWM_MAX 2500

// 定义宏
#define PID_DT 0.01 // PID 计算的时间间隔
// 定义kp上下限
#define PROPORTIONAL_LIMIT_MIN_H -80.0f // 水平kp下限
#define PROPORTIONAL_LIMIT_MAX_H 80.0f // 水平kp上限
#define PROPORTIONAL_LIMIT_MIN_V -40.0f // 垂直kp下限
#define PROPORTIONAL_LIMIT_MAX_V 40.0f // 垂直kp上限
// 定义积分上下限
#define INTEGRAL_LIMIT_MIN_H -100.0f // 水平积分下限
#define INTEGRAL_LIMIT_MAX_H 100.0f // 水平积分上限
#define INTEGRAL_LIMIT_MIN_V -50.0f // 垂直积分下限
#define INTEGRAL_LIMIT_MAX_V 50.0f // 垂直积分上限
// 定义微分上下限
#define DERIVATIVE_LIMIT_MIN_H -40.0f // 水平积分下限
#define DERIVATIVE_LIMIT_MAX_H 40.0f // 水平积分上限
#define DERIVATIVE_LIMIT_MIN_V -20.0f // 垂直积分下限
#define DERIVATIVE_LIMIT_MAX_V 20.0f // 垂直积分上限

// 定义PID输出上下限
#define HORIZONTAL_MIN -135
#define HORIZONTAL_MAX 135
#define VERTICAL_MIN -85
#define VERTICAL_MAX 85

#define CAMERA_ERROR_X 0x8010 // 相机X坐标误差
#define CAMERA_ERROR_Y 0x8011 // 相机Y坐标误差
#define CAMERA_H_ANGLE 0x8012 // 相机水平角度
#define CAMERA_V_ANGLE 0x8013 // 相机垂直角度
#define CAMERA_TARGET_DETECTED 0x8014 // 目标检测状态指令宏定义

// 新增控制器选择枚举（移到前面）
typedef enum {
    PID_MODE_POSITION = 0,  // 位置式PID
    PID_MODE_INCREMENT = 1  // 增量式PID
} PID_Mode;

// PID limit structure
typedef struct 
{
    float kp_limit_min;    // P项输出下限
    float kp_limit_max;    // P项输出上限
    float ki_limit_min;    // I项输出下限
    float ki_limit_max;    // I项输出上限
    float kd_limit_min;    // D项输出下限
    float kd_limit_max;    // D项输出上限
    float output_min;      // 总输出下限
    float output_max;      // 总输出上限
} PID_Limit;

// PID structure
typedef struct {
    float Kp;          // Proportional gain
    float Ki;          // Integral gain
    float Kd;          // Derivative gain
    float error;       // 当前误差
    float prev_error;  // Previous error
    float integral;    // 积分项
    float derivative;  // 微分项
    float p_term;      // 比例项输出
    float i_term;      // 积分项输出
    float d_term;      // 微分项输出
    float output;      // Output value
    float ramp_step;   // 最大步长限制
    const PID_Limit *limit;   // PID限制参数指针
} PID_Controller;

// 增量式PID结构体
typedef struct {
    float Kp;          // Proportional gain
    float Ki;          // Integral gain
    float Kd;          // Derivative gain
    float error;       // 当前误差
    float prev_error;  // 上次误差
    float prev_prev_error;  // 上上次误差
    float output;      // 当前输出角度
    float delta_output; // 输出增量
    float ramp_step;   // 最大步长限制
    const PID_Limit *limit;   // PID限制参数指针
} Inc_PID_Controller;

// 新增平台状态结构体
typedef struct {
    float x_error;    // 水平误差
    float y_error;    // 垂直误差
    float h_angle;    // 目标水平角度
    float v_angle;    // 目标垂直角度
    bool error_updated; // 误差更新标志
    bool angle_updated; // 角度更新标志
    bool target_detected; // 是否检测到目标
    bool target_locked;   // 目标锁定状态
    bool cmd_received;    // 是否接收到新指令
    PID_Mode pid_mode;    // PID控制器模式选择
} Platform_Status;

// 声明全局平台状态变量
extern Platform_Status platform_status;

// Function prototypes
void set_platform(uint16_t h_pwm, uint16_t v_pwm);
void set_platform_angle(float h_angle, float v_angle);
void camera_platform_init(void);
void update_platform_position(float x_error, float y_error);
float limit_integral(float integral, float min, float max); // 新增限制函数声明
void set_platform_x_error(float x_err);
void set_platform_y_error(float y_err);
void set_platform_h_angle(float h_ang);
void set_platform_v_angle(float v_ang);
void set_target_locked(bool detected);
void camera_10ms_task(void);

// 新增函数声明
void pid_controller_init(PID_Controller *pid, float kp, float ki, float kd, float ramp_step, const PID_Limit *limit);
float pid_calculate(PID_Controller *pid, float error);
void inc_pid_controller_init(Inc_PID_Controller *pid, float kp, float ki, float kd, float ramp_step, const PID_Limit *limit);
float inc_pid_calculate(Inc_PID_Controller *pid, float error);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_PLATFORM_H__ */ 